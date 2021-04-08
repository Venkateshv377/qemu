/*
 * QEMU model of the CPM_CRCPM CPM Clock and Reset Control Registers
 *
 * Copyright (c) 2019 Xilinx Inc.
 *
 * Autogenerated by xregqemu.py 2019-07-17.
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
#include "hw/register.h"
#include "hw/irq.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"

#ifndef XILINX_CPM_CRCPM_ERR_DEBUG
#define XILINX_CPM_CRCPM_ERR_DEBUG 0
#endif

#define TYPE_XILINX_CPM_CRCPM "xlnx.versal_cpm_crcpm"

#define XILINX_CPM_CRCPM(obj) \
     OBJECT_CHECK(CPM_CRCPM, (obj), TYPE_XILINX_CPM_CRCPM)

REG32(REG_CTRL, 0x0)
    FIELD(REG_CTRL, SLVERR_ENABLE, 0, 1)
REG32(IR_STATUS, 0x4)
    FIELD(IR_STATUS, ADDR_DECODE_ERR, 0, 1)
REG32(IR_MASK, 0x8)
    FIELD(IR_MASK, ADDR_DECODE_ERR, 0, 1)
REG32(IR_ENABLE, 0xc)
    FIELD(IR_ENABLE, ADDR_DECODE_ERR, 0, 1)
REG32(IR_DISABLE, 0x10)
    FIELD(IR_DISABLE, ADDR_DECODE_ERR, 0, 1)
REG32(WPROT, 0x30)
    FIELD(WPROT, ACTIVE, 0, 1)
REG32(CPLL_CTRL, 0x40)
    FIELD(CPLL_CTRL, POST_SRC, 24, 3)
    FIELD(CPLL_CTRL, PRE_SRC, 20, 3)
    FIELD(CPLL_CTRL, CLKOUTDIV, 16, 2)
    FIELD(CPLL_CTRL, FBDIV, 8, 8)
    FIELD(CPLL_CTRL, BYPASS, 3, 1)
    FIELD(CPLL_CTRL, RESET, 0, 1)
REG32(CPLL_CFG, 0x44)
    FIELD(CPLL_CFG, LOCK_DLY, 25, 7)
    FIELD(CPLL_CFG, LOCK_CNT, 13, 10)
    FIELD(CPLL_CFG, LFHF, 10, 2)
    FIELD(CPLL_CFG, CP, 5, 4)
    FIELD(CPLL_CFG, RES, 0, 4)
REG32(PLL_STATUS, 0x50)
    FIELD(PLL_STATUS, CPLL_STABLE, 2, 1)
    FIELD(PLL_STATUS, CPLL_LOCK, 0, 1)
REG32(CPM_CORE_REF_CTRL, 0x100)
    FIELD(CPM_CORE_REF_CTRL, CLKACT, 25, 1)
    FIELD(CPM_CORE_REF_CTRL, DIVISOR0, 8, 10)
REG32(CPM_LSBUS_REF_CTRL, 0x104)
    FIELD(CPM_LSBUS_REF_CTRL, CLKACT, 25, 1)
    FIELD(CPM_LSBUS_REF_CTRL, DIVISOR0, 8, 10)
REG32(CPM_DBG_REF_CTRL, 0x108)
    FIELD(CPM_DBG_REF_CTRL, CLKACT, 25, 1)
    FIELD(CPM_DBG_REF_CTRL, DIVISOR0, 8, 10)
REG32(SAFETY_CHK, 0x150)
REG32(RST_DBG, 0x300)
    FIELD(RST_DBG, RESET, 0, 1)
REG32(RST_PCIE_CONFIG, 0x304)
    FIELD(RST_PCIE_CONFIG, RESET, 0, 1)
REG32(RST_PCIE_CORE0, 0x308)
    FIELD(RST_PCIE_CORE0, RESET, 0, 1)
REG32(RST_PCIE_CORE1, 0x30c)
    FIELD(RST_PCIE_CORE1, RESET, 0, 1)
REG32(RST_CMN, 0x314)
    FIELD(RST_CMN, RESET, 0, 1)
REG32(RST_L2_0, 0x318)
    FIELD(RST_L2_0, RESET, 0, 1)
REG32(RST_ADDR_REMAP, 0x320)
    FIELD(RST_ADDR_REMAP, RESET, 0, 1)
REG32(RST_CPI0, 0x324)
    FIELD(RST_CPI0, RESET, 0, 1)
REG32(RST_CPI1, 0x328)
    FIELD(RST_CPI1, RESET, 0, 1)

#define CPM_CRCPM_R_MAX (R_RST_CPI1 + 1)

typedef struct CPM_CRCPM {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq_ir;

    uint32_t regs[CPM_CRCPM_R_MAX];
    RegisterInfo regs_info[CPM_CRCPM_R_MAX];
} CPM_CRCPM;

static void ir_update_irq(CPM_CRCPM *s)
{
    bool pending = s->regs[R_IR_STATUS] & ~s->regs[R_IR_MASK];
    qemu_set_irq(s->irq_ir, pending);
}

static void ir_status_postw(RegisterInfo *reg, uint64_t val64)
{
    CPM_CRCPM *s = XILINX_CPM_CRCPM(reg->opaque);
    ir_update_irq(s);
}

static uint64_t ir_enable_prew(RegisterInfo *reg, uint64_t val64)
{
    CPM_CRCPM *s = XILINX_CPM_CRCPM(reg->opaque);
    uint32_t val = val64;

    s->regs[R_IR_MASK] &= ~val;
    ir_update_irq(s);
    return 0;
}

static uint64_t ir_disable_prew(RegisterInfo *reg, uint64_t val64)
{
    CPM_CRCPM *s = XILINX_CPM_CRCPM(reg->opaque);
    uint32_t val = val64;

    s->regs[R_IR_MASK] |= val;
    ir_update_irq(s);
    return 0;
}

static const RegisterAccessInfo cpm_crcpm_regs_info[] = {
    {   .name = "REG_CTRL",  .addr = A_REG_CTRL,
        .rsvd = 0xfffffffe,
        .ro = 0xfffffffe,
    },{ .name = "IR_STATUS",  .addr = A_IR_STATUS,
        .rsvd = 0xfffffffe,
        .ro = 0xfffffffe,
        .w1c = 0x1,
        .post_write = ir_status_postw,
    },{ .name = "IR_MASK",  .addr = A_IR_MASK,
        .reset = 0x1,
        .rsvd = 0xfffffffe,
        .ro = 0xffffffff,
    },{ .name = "IR_ENABLE",  .addr = A_IR_ENABLE,
        .rsvd = 0xfffffffe,
        .ro = 0xfffffffe,
        .pre_write = ir_enable_prew,
    },{ .name = "IR_DISABLE",  .addr = A_IR_DISABLE,
        .rsvd = 0xfffffffe,
        .ro = 0xfffffffe,
        .pre_write = ir_disable_prew,
    },{ .name = "WPROT",  .addr = A_WPROT,
    },{ .name = "CPLL_CTRL",  .addr = A_CPLL_CTRL,
        .reset = 0x14809,
        .rsvd = 0xf88c00f6,
        .ro = 0xf88c00f6,
    },{ .name = "CPLL_CFG",  .addr = A_CPLL_CFG,
        .reset = 0x2000000,
        .rsvd = 0x1801210,
        .ro = 0x1801210,
    },{ .name = "PLL_STATUS",  .addr = A_PLL_STATUS,
        .reset = R_PLL_STATUS_CPLL_STABLE_MASK |
                 R_PLL_STATUS_CPLL_LOCK_MASK,
        .rsvd = 0xfa,
        .ro = 0xff,
    },{ .name = "CPM_CORE_REF_CTRL",  .addr = A_CPM_CORE_REF_CTRL,
        .reset = 0x2002500,
        .rsvd = 0xfdfc00ff,
        .ro = 0xfdfc00ff,
    },{ .name = "CPM_LSBUS_REF_CTRL",  .addr = A_CPM_LSBUS_REF_CTRL,
        .reset = 0x2002500,
        .rsvd = 0xfdfc00ff,
        .ro = 0xfdfc00ff,
    },{ .name = "CPM_DBG_REF_CTRL",  .addr = A_CPM_DBG_REF_CTRL,
        .reset = 0x2002500,
        .rsvd = 0xfdfc00ff,
        .ro = 0xfdfc00ff,
    },{ .name = "SAFETY_CHK",  .addr = A_SAFETY_CHK,
    },{ .name = "RST_DBG",  .addr = A_RST_DBG,
        .reset = 0x1,
    },{ .name = "RST_PCIE_CONFIG",  .addr = A_RST_PCIE_CONFIG,
        .reset = 0x1,
    },{ .name = "RST_PCIE_CORE0",  .addr = A_RST_PCIE_CORE0,
        .reset = 0x1,
    },{ .name = "RST_PCIE_CORE1",  .addr = A_RST_PCIE_CORE1,
        .reset = 0x1,
    },{ .name = "RST_CMN",  .addr = A_RST_CMN,
        .reset = 0x1,
    },{ .name = "RST_L2_0",  .addr = A_RST_L2_0,
        .reset = 0x1,
    },{ .name = "RST_ADDR_REMAP",  .addr = A_RST_ADDR_REMAP,
        .reset = 0x1,
    },{ .name = "RST_CPI0",  .addr = A_RST_CPI0,
        .reset = 0x1,
    },{ .name = "RST_CPI1",  .addr = A_RST_CPI1,
        .reset = 0x1,
    }
};

static void cpm_crcpm_reset(DeviceState *dev)
{
    CPM_CRCPM *s = XILINX_CPM_CRCPM(dev);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }

    ir_update_irq(s);
}

static const MemoryRegionOps cpm_crcpm_ops = {
    .read = register_read_memory,
    .write = register_write_memory,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void cpm_crcpm_realize(DeviceState *dev, Error **errp)
{
    /* Delete this if you don't need it */
}

static void cpm_crcpm_init(Object *obj)
{
    CPM_CRCPM *s = XILINX_CPM_CRCPM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    RegisterInfoArray *reg_array;

    memory_region_init(&s->iomem, obj, TYPE_XILINX_CPM_CRCPM, CPM_CRCPM_R_MAX * 4);
    reg_array =
        register_init_block32(DEVICE(obj), cpm_crcpm_regs_info,
                              ARRAY_SIZE(cpm_crcpm_regs_info),
                              s->regs_info, s->regs,
                              &cpm_crcpm_ops,
                              XILINX_CPM_CRCPM_ERR_DEBUG,
                              CPM_CRCPM_R_MAX * 4);
    memory_region_add_subregion(&s->iomem,
                                0x0,
                                &reg_array->mem);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq_ir);
}

static const VMStateDescription vmstate_cpm_crcpm = {
    .name = TYPE_XILINX_CPM_CRCPM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, CPM_CRCPM, CPM_CRCPM_R_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static void cpm_crcpm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = cpm_crcpm_reset;
    dc->realize = cpm_crcpm_realize;
    dc->vmsd = &vmstate_cpm_crcpm;
}

static const TypeInfo cpm_crcpm_info = {
    .name          = TYPE_XILINX_CPM_CRCPM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CPM_CRCPM),
    .class_init    = cpm_crcpm_class_init,
    .instance_init = cpm_crcpm_init,
};

static void cpm_crcpm_register_types(void)
{
    type_register_static(&cpm_crcpm_info);
}

type_init(cpm_crcpm_register_types)
