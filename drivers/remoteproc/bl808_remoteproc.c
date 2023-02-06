// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote processor machine-specific module for bl808
 *
 * Copyright (C) 2023 Justin Hammond
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/mailbox_client.h>

#include "remoteproc_internal.h"

//#define MBOX_NB_MBX 1

struct bl808_mbox {
	const unsigned char name[10];
	struct mbox_client client;
	struct mbox_chan *chan;
	struct work_struct vq_work;
	int vq_id;
};


/**
 * struct bl808_rproc - bl808 remote processor instance state
 * @rproc: rproc handle
 * @mbox: the mailbox channel
 * @client: the mailbox client
 */
struct bl808_rproc {
	struct rproc *rproc;
	struct bl808_mbox mbox;
	struct workqueue_struct *workqueue;
};



/* The feature bitmap for virtio rpmsg */
#define VIRTIO_RPMSG_F_NS	0 /* RP supports name service notifications */

#define FW_RSC_U32_ADDR_ANY 0xFFFFFFFFUL

#define RPMSG_VDEV_DFEATURES        (1 << VIRTIO_RPMSG_F_NS)

/* VirtIO rpmsg device id */
#define VIRTIO_ID_RPMSG_             7

/* Resource table entries */
#define NUM_VRINGS                  0x02
#define VRING_ALIGN                 0x1000
#define RING_TX                     FW_RSC_U32_ADDR_ANY
#define RING_RX                     FW_RSC_U32_ADDR_ANY
#define VRING_SIZE                  8

#define NUM_TABLE_ENTRIES           1

#define NO_RESOURCE_ENTRIES         1

/* this is normally in the header of the ELF files for firmware
 * but since M0 is already running and there isn't a ELF file on
 * flash for it, we fudge the resource table header to statically
 * specify our virtio rings etc
 */
struct remote_resource_table {
	u32 version;
	u32 num;
	u32 reserved[2];
	u32 offset[NO_RESOURCE_ENTRIES];
	/* rpmsg vdev entry */
	u32 type;
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring rpmsg_vring0;
	struct fw_rsc_vdev_vring rpmsg_vring1;
} __packed;

/* this is our fudged ELF Resource Table Header setup with
 * one rpmsg virtio device and two virtio rings
 */
struct remote_resource_table resources = {
	/* Version */
	.version = 1,

	/* NUmber of table entries */
	.num = NUM_TABLE_ENTRIES,

	/* reserved fields */
	.reserved = {0, 0,},

	/* Offsets of rsc entries */
	.offset[0] = offsetof(struct remote_resource_table, type),

	.type = RSC_VDEV,
	/* Virtio device entry */
	{
	 VIRTIO_ID_RPMSG_, 0, RPMSG_VDEV_DFEATURES, 0, 0, 0,
	 NUM_VRINGS, {0, 0},
	},

	/* Vring rsc entry - part of vdev rsc entry */
	{0x22048000, VRING_ALIGN, VRING_SIZE, 0, 0x22048000},
	{0x2204C000, VRING_ALIGN, VRING_SIZE, 1, 0x2204C000},
};

/* return a pointer to our resource table
 */
struct resource_table *bl808_rproc_get_loaded_rsc_table(struct rproc *rproc, size_t *size)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "bl808_rproc_get_loaded_rsc_table");

	*size = sizeof(resources);
	return (struct resource_table *)&resources;
}

/* allocate vdev0buffer */
static int bl808_rproc_mem_alloc(struct rproc *rproc,
			      struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "Allocating memory region");

	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "Unable to map memory region: %pa+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;
	dev_dbg(dev, "Allocated memory region: %pa+%zx -> %p", &mem->dma, mem->len, mem->va);

	return 0;
}

/* release vdev0buffer */
static int bl808_rproc_mem_release(struct rproc *rproc,
				struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "release memory region");

	iounmap(mem->va);

	return 0;
}

/*
 * Pull the memory ranges for virtio from the device tree and register them
 */
static int bl808_rproc_setupmem(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	struct of_phandle_iterator it;
	int index = 0;

	dev_dbg(dev, "bl808_rproc_parse_fw %s", np->name);

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		dev_dbg(dev, "memory-region %s", it.node->name);
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}
		dev_dbg(dev, "memory-region %s", it.node->name);

		/*  No need to map vdev buffer */
		if (strcmp(it.node->name, "vdev0buffer")) {
			/* Register memory region */
			dev_dbg(dev, "registering memory region %s %llx", it.node->name, rmem->base);
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, rmem->base,
						   bl808_rproc_mem_alloc,
						   bl808_rproc_mem_release,
						   it.node->name);
		} else {
			/* Register reserved memory for vdev buffer allocation */
			dev_dbg(dev, "registering reserved memory region %s %llx", it.node->name, rmem->base);
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   rmem->base,
							   it.node->name);
		}

		if (!mem) {
			dev_err(dev, "unable to allocate memory entry %s", it.node->name);
			return -ENOMEM;
		}
		rproc_add_carveout(rproc, mem);
		index++;
	}
	return 0;
}


/* M0 is already started. Do Nothing
 */
static int bl808_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "bl808_rproc_start");

	return 0;
}

/* We don't want to stop M0, as it will crash. Do Nothing */
static int bl808_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "bl808_rproc_stop");

	return 0;
}

/* kick the virtqueue to let M0 know there is a update to the vring */
static void bl808_rproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = rproc->dev.parent;
	struct bl808_rproc *drproc = (struct bl808_rproc *)rproc->priv;

	/* Kick the other CPU to let it know the vrings are updated */
	dev_dbg(dev, "%s %d", __func__, vqid);
	mbox_send_message(drproc->mbox.chan, (void *)vqid);
}

static void bflb_rproc_mb_vq_work(struct work_struct *work)
{
	struct bl808_mbox *mb = container_of(work, struct bl808_mbox, vq_work);
	struct rproc *rproc = dev_get_drvdata(mb->client.dev);

	if (rproc_vq_interrupt(rproc, mb->vq_id) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message found in vq%d\n", mb->vq_id);
}


/* M0 signaled us there is a update on the vring, check it
 */
static void bflb_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct device *dev = client->dev;
	struct rproc *rproc = dev_get_drvdata(dev);
	struct bl808_rproc *drproc = (struct bl808_rproc *)rproc->priv;
	struct bl808_mbox *mb = &drproc->mbox;

	mb->vq_id = (u32)data;

	dev_dbg(dev, "%s %d", __func__, mb->vq_id);

	queue_work(drproc->workqueue, &mb->vq_work);
}

/* M0 is already running when we boot
 * so just attach to it.
 * we also register a mailbox to get kicks from M0 when vrings are updated
 */
static int bl808_rproc_attach(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct bl808_rproc *drproc = (struct bl808_rproc *)rproc->priv;
	struct mbox_client *vq1_mbox = &drproc->mbox.client;
	struct bl808_mbox *chan = &drproc->mbox;
	int ret;

	/* request the mailboxs */
	vq1_mbox->dev = dev->parent;
	vq1_mbox->tx_done = NULL;
	vq1_mbox->rx_callback = bflb_rproc_mbox_callback;
	vq1_mbox->tx_block = false;
	vq1_mbox->knows_txdone = false;

	chan->chan = mbox_request_channel(vq1_mbox, 0);
	if (IS_ERR(chan->chan)) {
		ret = -EBUSY;
		dev_err(dev, "mbox_request_channel failed: %ld\n",
			PTR_ERR(chan->chan));
		return ret;
	}
	INIT_WORK(&chan->vq_work, bflb_rproc_mb_vq_work);

	dev_dbg(dev, "%s: Attaching to %s", __func__, rproc->name);
	return 0;
}

/* Detach. Do Nothing? */
static int bl808_rproc_detach(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "%s: Detaching from %s", __func__, rproc->name);
	return 0;
}

static const struct rproc_ops bl808_rproc_ops = {
	.start = bl808_rproc_start,
	.stop = bl808_rproc_stop,
	.attach = bl808_rproc_attach,
	.detach = bl808_rproc_detach,
	.kick = bl808_rproc_kick,
	.prepare = bl808_rproc_setupmem,
	.get_loaded_rsc_table = bl808_rproc_get_loaded_rsc_table,
};


static int bl808_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bl808_rproc *drproc;
	struct rproc *rproc;
	int ret;

	rproc = rproc_alloc(dev, "M0", &bl808_rproc_ops, NULL,
		sizeof(*drproc));
	if (!rproc) {
		ret = -ENOMEM;
		goto free_mem;
	}

	/* error recovery is not supported at present */
	rproc->recovery_disabled = true;

	/* M0 is running when linux boots */
	//atomic_inc(&rproc->power);
	rproc->state = RPROC_DETACHED;

	drproc = rproc->priv;
	drproc->rproc = rproc;
	rproc->has_iommu = false;

	drproc->workqueue = create_workqueue(dev_name(dev));
	if (!drproc->workqueue) {
		dev_err(dev, "cannot create workqueue\n");
		ret = -ENOMEM;
		goto free_wkq;
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto free_rproc;
	}

	platform_set_drvdata(pdev, rproc);

	dev_dbg(dev, "rproc_add success");

	return 0;

free_wkq:
	destroy_workqueue(drproc->workqueue);
free_rproc:
	rproc_free(rproc);
free_mem:
	if (dev->of_node)
		of_reserved_mem_device_release(dev);
	return ret;
}

static int bl808_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct bl808_rproc *drproc = (struct bl808_rproc *)rproc->priv;
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "bl808_rproc_remove");

	/*XXX TODO: we need to unregister our mailbox? */

	destroy_workqueue(drproc->workqueue);

	rproc_del(rproc);
	rproc_free(rproc);
	if (dev->of_node)
		of_reserved_mem_device_release(dev);

	return 0;
}

static const struct of_device_id davinci_rproc_of_match[] __maybe_unused = {
	{ .compatible = "bflb,bl808-rproc", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, davinci_rproc_of_match);

static struct platform_driver bl808_rproc_driver = {
	.probe = bl808_rproc_probe,
	.remove = bl808_rproc_remove,
	.driver = {
		.name = "bl808-rproc",
		.of_match_table = of_match_ptr(davinci_rproc_of_match),
	},
};

module_platform_driver(bl808_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("justin@dynam.ac");
MODULE_DESCRIPTION("bl808 Remote Processor control driver");
