/*
 * ZynqMP SDHCI controller.
 *
 * Copyright (c) 2013 Xilinx Inc
 * Copyright (c) 2013 Peter Crosthwaite <peter.crosthwaite@xilinx.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"

#include "qemu/bitops.h"
#include "qapi/qmp/qerror.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"

#include "hw/fdt_generic_util.h"

#include "hw/sd/sd.h"
#include "hw/sd/sdhci.h"
#include "qapi/error.h"
#include "sdhci-internal.h"

#include "qemu/module.h"

//#define DEBUG_LOGS	1

#ifndef ZYNQMP_SDHCI_ERR_DEBUG
#define ZYNQMP_SDHCI_ERR_DEBUG 0
#endif

#define TYPE_ZYNQMP_SDHCI "xilinx.zynqmp-sdhci"

#define ZYNQMP_SDHCI(obj) \
     OBJECT_CHECK(ZynqMPSDHCIState, (obj), TYPE_ZYNQMP_SDHCI)

#define ZYNQMP_SDHCI_PARENT_CLASS \
    object_class_get_parent(object_class_by_name(TYPE_PCI_SDHCI))

#if 0
typedef struct ZynqMPSDHCIState {
    /*< private >*/
    SDHCIState parent_obj;

    /*< public >*/
    SDState *card;
    uint8_t drive_index;
    bool is_mmc;
} ZynqMPSDHCIState;
#endif
static void zynqmp_sdhci_slottype_handler(void *opaque, int n, int level)
{
//    ZynqMPSDHCIState *s = ZYNQMP_SDHCI(opaque);
    SDHCIState *s = SYSBUS_SDHCI(opaque);

    assert(n == 0);

    if (!s->card) {
        /* Card Not Connected */
        return;
    }

    if (level != s->is_mmc) {
        qemu_log_mask(LOG_GUEST_ERROR, "WARNING: Inserted %s Card but"
                      " Slot configured as %s\n",
                      s->is_mmc ? "MMC" : "SD",
                      level ? "MMC" : "SD");
    }
}

static void zynqmp_sdhci_reset(DeviceState *dev)
{
    DeviceClass *dc_parent = DEVICE_CLASS(ZYNQMP_SDHCI_PARENT_CLASS);
    SDHCIState *p = SYSBUS_SDHCI(dev);

    dc_parent->reset(dev);
    if (p->is_mmc) {
        p->capareg = deposit64(p->capareg, R_SDHC_CAPAB_SLOT_TYPE_SHIFT,
                  R_SDHC_CAPAB_SLOT_TYPE_LENGTH, 0x01);
    }
#ifdef DEBUG_LOGS
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
#endif
}

static void zynqmp_sdhci_realize(PCIDevice *dev, Error **errp)
{
//    DeviceClass *dc_parent = DEVICE_CLASS(ZYNQMP_SDHCI_PARENT_CLASS);
    SDHCIState *s = PCI_SDHCI(dev);
    DriveInfo *di_sd, *di_mmc;
    DeviceState *carddev_sd;
    static int index_offset = 0;
    DeviceState *pci_ds = &dev->qdev;

    sdhci_initfn(s);

    if (!s->memattr) {
        s->memattr = g_malloc0(sizeof(MemTxAttrs));
        *s->memattr =  MEMTXATTRS_UNSPECIFIED;
    }

    sdhci_common_realize(s, errp);

    /* Xilinx: This device is used in some Zynq-7000 devices which don't
     * set the drive-index property. In order to avoid errors we increament
     * the drive index each time we call this.
     * The other solution could be to just ignore the error returned when
     * connecting the drive. That seems risky though.
     */
    if (!s->drive_index) {
        s->drive_index += index_offset;
        index_offset++;
    }
#ifdef DEBUG_LOGS
    fprintf(stderr, "%s, %s, %d drive_index: %d\n", __FILE__, __func__, __LINE__, s->drive_index);
#endif

    qdev_prop_set_uint8(pci_ds, "sd-spec-version", 3);
//    qdev_prop_set_uint64(pci_ds, "capareg", 0x280737ec6481);
    qdev_prop_set_uint64(pci_ds, "capareg", 0x280737ecC881);
    qdev_prop_set_uint8(pci_ds, "uhs", UHS_I);
    carddev_sd = qdev_new(TYPE_SD_CARD);
    object_property_add_child(OBJECT(pci_ds), "sd-card",
                              OBJECT(carddev_sd));
    object_property_set_bool(OBJECT(carddev_sd), "spi", false, &error_fatal);

    /*
     * drive_index is used to attach a card in SD mode.
     * drive_index + 2 is used to attach a card in MMC mode.
     *
     * If the user attaches a card to both slots, we bail out.
     */

    di_sd = drive_get_by_index(IF_SD , s->drive_index);
    di_mmc = drive_get_by_index(IF_SD, (s->drive_index + 2));

    if (di_sd && di_mmc) {
        if (!di_sd->is_default && !di_mmc->is_default) {
            error_setg(&error_fatal, "Cannot attach both an MMC"
                                     " and an SD card into the same slot");
        }
    }

    if (di_sd) {
#ifdef DEBUG_LOGS
	    fprintf(stderr, "%s: %s: %d di_sd->bus:%d unit:%d default:%d type:%d\n", __FILE__, __func__, __LINE__, di_sd->bus, di_sd->unit, 
			    di_sd->is_default, di_sd->type);
#endif
        qdev_prop_set_drive(carddev_sd, "drive", blk_by_legacy_dinfo(di_sd));
        object_property_set_bool(OBJECT(carddev_sd), "mmc", false, &error_fatal);
    }

    if (di_mmc) {
#ifdef DEBUG_LOGS
	    fprintf(stderr, "%s: %s: %d di_mmC->bus:%d unit:%d default:%d type:%d\n",  __FILE__, __func__, __LINE__, di_mmc->bus, 
			    di_mmc->unit, di_mmc->is_default, di_mmc->type);
#endif
        qdev_prop_set_drive(carddev_sd, "drive", blk_by_legacy_dinfo(di_mmc));
        object_property_set_bool(OBJECT(carddev_sd), "mmc", true, &error_fatal);
        s->is_mmc = true;
    }


#ifdef DEBUG_LOGS
    BusState *bus = qdev_get_child_bus(DEVICE(pci_ds), "sd-bus");
    fprintf(stderr, "%s, %s, %d bus->name:%s\n", __FILE__, __func__, __LINE__, bus->name);
#endif

    qdev_realize(carddev_sd,
                           qdev_get_child_bus(DEVICE(pci_ds), "sd-bus"),
                           &error_abort);

    qdev_init_gpio_in_named(pci_ds, zynqmp_sdhci_slottype_handler, "SLOTTYPE", 1);
    s->card = SD_CARD(carddev_sd);

//    dc_parent->realize(pci_ds, errp);

    dev->config[PCI_CLASS_PROG] = 0x01; /* Standard Host supported DMA */
    dev->config[PCI_INTERRUPT_PIN] = 0x01; /* interrupt pin A */
    s->irq = pci_allocate_irq(dev);
    s->dma_as = pci_get_address_space(dev);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->iomem);

    if(s->irq) {
	    fprintf(stderr, "%s, %s, %d: Irq VALID \n", __FILE__, __func__, __LINE__);
    } else {
	    fprintf(stderr, "%s, %s, %d: Irq INVALID \n", __FILE__, __func__, __LINE__);
    }

    if(s->dma_as) {
	    fprintf(stderr, "%s, %s, %d: dma_as VALID \n", __FILE__, __func__, __LINE__);
    } else {
	    fprintf(stderr, "%s, %s, %d: dma_as INVALID \n", __FILE__, __func__, __LINE__);
    }
}

static Property zynqmp_sdhci_properties[] = {
//    DEFINE_SDHCI_COMMON_PROPERTIES(SDHCIState),
    DEFINE_PROP_UINT8("sd-spec-version", SDHCIState, sd_spec_version, 3),
    DEFINE_PROP_UINT8("uhs", SDHCIState, uhs_mode, UHS_NOT_SUPPORTED),
    DEFINE_PROP_UINT64("capareg", SDHCIState, capareg, SDHC_CAPAB_REG_DEFAULT),
    DEFINE_PROP_UINT8("drive-index", SDHCIState, drive_index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void zynqmp_sdhci_exit(PCIDevice *dev)
{
    SDHCIState *s = PCI_SDHCI(dev);

    sdhci_common_unrealize(s);
    sdhci_uninitfn(s);
#ifdef DEBUG_LOGS
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
#endif
}

static void zynqmp_sdhci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = zynqmp_sdhci_realize;
    k->vendor_id = PCI_VENDOR_ID_REDHAT;
    k->device_id = PCI_DEVICE_ID_REDHAT_SDHCI;
    k->class_id = PCI_CLASS_SYSTEM_SDHCI;
    k->exit = zynqmp_sdhci_exit;
    dc->reset = zynqmp_sdhci_reset;
 
    device_class_set_props(dc, zynqmp_sdhci_properties);

    sdhci_common_class_init(klass, data);
#ifdef DEBUG_LOGS
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
#endif
}

static const TypeInfo zynqmp_sdhci_info = {
    .name          = TYPE_PCI_SDHCI,
    .parent        = TYPE_PCI_DEVICE,
    .class_init    = zynqmp_sdhci_class_init,
    .instance_size = sizeof(SDHCIState),
    .interfaces = (InterfaceInfo[]) {
     { INTERFACE_CONVENTIONAL_PCI_DEVICE },
     { }, 
     },
};

static void zynqmp_sdhci_register_types(void)
{
    type_register_static(&zynqmp_sdhci_info);
}

type_init(zynqmp_sdhci_register_types)
