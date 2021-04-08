/*
 * QEMU model of the ME_NPI NPI registers for ME Array
 *
 * Copyright (c) 2021 Xilinx Inc.
 *
 * Autogenerated by xregqemu.py 2021-04-07.
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
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "migration/vmstate.h"

#ifndef XILINX_ME_NPI_ERR_DEBUG
#define XILINX_ME_NPI_ERR_DEBUG 0
#endif

#define TYPE_XILINX_ME_NPI "xlnx.me-npi"

#define XILINX_ME_NPI(obj) \
     OBJECT_CHECK(ME_NPI, (obj), TYPE_XILINX_ME_NPI)

REG32(REG_PCSR_MASK, 0x0)
    FIELD(REG_PCSR_MASK, ME_SHIM_RESET, 27, 1)
    FIELD(REG_PCSR_MASK, ME_ARRAY_RESET, 26, 1)
    FIELD(REG_PCSR_MASK, SCAN_CLEAR_LOWPOW_EN, 25, 1)
    FIELD(REG_PCSR_MASK, ME_IPOR, 24, 1)
    FIELD(REG_PCSR_MASK, MEM_CLEAR_EN_ALL, 23, 1)
    FIELD(REG_PCSR_MASK, OD_BIST_SETUP_1, 22, 1)
    FIELD(REG_PCSR_MASK, OD_MBIST_ASYNC_RESET_N, 21, 1)
    FIELD(REG_PCSR_MASK, TEST_SAFE, 20, 1)
    FIELD(REG_PCSR_MASK, SLVERREN, 19, 1)
    FIELD(REG_PCSR_MASK, MEM_CLEAR_TRIGGER, 18, 1)
    FIELD(REG_PCSR_MASK, SYS_RST_MASK, 15, 3)
    FIELD(REG_PCSR_MASK, PWRDN, 14, 1)
    FIELD(REG_PCSR_MASK, DISNPICLK, 13, 1)
    FIELD(REG_PCSR_MASK, APBEN, 12, 1)
    FIELD(REG_PCSR_MASK, SCAN_CLEAR_TRIGGER, 11, 1)
    FIELD(REG_PCSR_MASK, STARTCAL, 10, 1)
    FIELD(REG_PCSR_MASK, FABRICEN, 9, 1)
    FIELD(REG_PCSR_MASK, TRISTATE, 8, 1)
    FIELD(REG_PCSR_MASK, HOLDSTATE, 7, 1)
    FIELD(REG_PCSR_MASK, INITSTATE, 6, 1)
    FIELD(REG_PCSR_MASK, ODISABLE, 2, 4)
    FIELD(REG_PCSR_MASK, GATEREG, 1, 1)
    FIELD(REG_PCSR_MASK, PCOMPLETE, 0, 1)
REG32(REG_PCSR_CONTROL, 0x4)
    FIELD(REG_PCSR_CONTROL, ME_SHIM_RESET, 27, 1)
    FIELD(REG_PCSR_CONTROL, ME_ARRAY_RESET, 26, 1)
    FIELD(REG_PCSR_CONTROL, SCAN_CLEAR_LOWPOW_EN, 25, 1)
    FIELD(REG_PCSR_CONTROL, ME_IPOR, 24, 1)
    FIELD(REG_PCSR_CONTROL, MEM_CLEAR_EN_ALL, 23, 1)
    FIELD(REG_PCSR_CONTROL, OD_BIST_SETUP_1, 22, 1)
    FIELD(REG_PCSR_CONTROL, OD_MBIST_ASYNC_RESET_N, 21, 1)
    FIELD(REG_PCSR_CONTROL, TEST_SAFE, 20, 1)
    FIELD(REG_PCSR_CONTROL, SLVERREN, 19, 1)
    FIELD(REG_PCSR_CONTROL, MEM_CLEAR_TRIGGER, 18, 1)
    FIELD(REG_PCSR_CONTROL, SYS_RST_MASK, 15, 3)
    FIELD(REG_PCSR_CONTROL, PWRDN, 14, 1)
    FIELD(REG_PCSR_CONTROL, DISNPICLK, 13, 1)
    FIELD(REG_PCSR_CONTROL, APBEN, 12, 1)
    FIELD(REG_PCSR_CONTROL, SCAN_CLEAR_TRIGGER, 11, 1)
    FIELD(REG_PCSR_CONTROL, STARTCAL, 10, 1)
    FIELD(REG_PCSR_CONTROL, FABRICEN, 9, 1)
    FIELD(REG_PCSR_CONTROL, TRISTATE, 8, 1)
    FIELD(REG_PCSR_CONTROL, HOLDSTATE, 7, 1)
    FIELD(REG_PCSR_CONTROL, INITSTATE, 6, 1)
    FIELD(REG_PCSR_CONTROL, ODISABLE, 2, 4)
    FIELD(REG_PCSR_CONTROL, GATEREG, 1, 1)
    FIELD(REG_PCSR_CONTROL, PCOMPLETE, 0, 1)
REG32(REG_PCSR_STATUS, 0x8)
    FIELD(REG_PCSR_STATUS, ME_PWR_SUPPLY, 15, 1)
    FIELD(REG_PCSR_STATUS, OD_MBIST_GO, 14, 1)
    FIELD(REG_PCSR_STATUS, HARD_FAIL_OR, 11, 3)
    FIELD(REG_PCSR_STATUS, HARD_FAIL_AND, 8, 3)
    FIELD(REG_PCSR_STATUS, MEM_CLEAR_PASS, 7, 1)
    FIELD(REG_PCSR_STATUS, MEM_CLEAR_DONE, 6, 1)
    FIELD(REG_PCSR_STATUS, CALERROR, 5, 1)
    FIELD(REG_PCSR_STATUS, CALDONE, 4, 1)
    FIELD(REG_PCSR_STATUS, INCAL, 3, 1)
    FIELD(REG_PCSR_STATUS, SCAN_CLEAR_PASS, 2, 1)
    FIELD(REG_PCSR_STATUS, SCAN_CLEAR_DONE, 1, 1)
    FIELD(REG_PCSR_STATUS, PCSRLOCK, 0, 1)
REG32(REG_PCSR_LOCK, 0xc)
    FIELD(REG_PCSR_LOCK, STATE, 0, 1)
REG32(ME_ISR, 0x30)
    FIELD(ME_ISR, INTERRUPT4, 3, 1)
    FIELD(ME_ISR, INTERRUPT3, 2, 1)
    FIELD(ME_ISR, INTERRUPT2, 1, 1)
    FIELD(ME_ISR, INTERRUPT1, 0, 1)
REG32(ME_ITR, 0x34)
    FIELD(ME_ITR, INTERRUPT4, 3, 1)
    FIELD(ME_ITR, INTERRUPT3, 2, 1)
    FIELD(ME_ITR, INTERRUPT2, 1, 1)
    FIELD(ME_ITR, INTERRUPT1, 0, 1)
REG32(ME_IMR0, 0x38)
    FIELD(ME_IMR0, INTERRUPT4, 3, 1)
    FIELD(ME_IMR0, INTERRUPT3, 2, 1)
    FIELD(ME_IMR0, INTERRUPT2, 1, 1)
    FIELD(ME_IMR0, INTERRUPT1, 0, 1)
REG32(ME_IER0, 0x3c)
    FIELD(ME_IER0, INTERRUPT4, 3, 1)
    FIELD(ME_IER0, INTERRUPT3, 2, 1)
    FIELD(ME_IER0, INTERRUPT2, 1, 1)
    FIELD(ME_IER0, INTERRUPT1, 0, 1)
REG32(ME_IDR0, 0x40)
    FIELD(ME_IDR0, INTERRUPT4, 3, 1)
    FIELD(ME_IDR0, INTERRUPT3, 2, 1)
    FIELD(ME_IDR0, INTERRUPT2, 1, 1)
    FIELD(ME_IDR0, INTERRUPT1, 0, 1)
REG32(ME_IMR1, 0x44)
    FIELD(ME_IMR1, INTERRUPT4, 3, 1)
    FIELD(ME_IMR1, INTERRUPT3, 2, 1)
    FIELD(ME_IMR1, INTERRUPT2, 1, 1)
    FIELD(ME_IMR1, INTERRUPT1, 0, 1)
REG32(ME_IER1, 0x48)
    FIELD(ME_IER1, INTERRUPT4, 3, 1)
    FIELD(ME_IER1, INTERRUPT3, 2, 1)
    FIELD(ME_IER1, INTERRUPT2, 1, 1)
    FIELD(ME_IER1, INTERRUPT1, 0, 1)
REG32(ME_IDR1, 0x4c)
    FIELD(ME_IDR1, INTERRUPT4, 3, 1)
    FIELD(ME_IDR1, INTERRUPT3, 2, 1)
    FIELD(ME_IDR1, INTERRUPT2, 1, 1)
    FIELD(ME_IDR1, INTERRUPT1, 0, 1)
REG32(ME_IMR2, 0x50)
    FIELD(ME_IMR2, INTERRUPT4, 3, 1)
    FIELD(ME_IMR2, INTERRUPT3, 2, 1)
    FIELD(ME_IMR2, INTERRUPT2, 1, 1)
    FIELD(ME_IMR2, INTERRUPT1, 0, 1)
REG32(ME_IER2, 0x54)
    FIELD(ME_IER2, INTERRUPT4, 3, 1)
    FIELD(ME_IER2, INTERRUPT3, 2, 1)
    FIELD(ME_IER2, INTERRUPT2, 1, 1)
    FIELD(ME_IER2, INTERRUPT1, 0, 1)
REG32(ME_IDR2, 0x58)
    FIELD(ME_IDR2, INTERRUPT4, 3, 1)
    FIELD(ME_IDR2, INTERRUPT3, 2, 1)
    FIELD(ME_IDR2, INTERRUPT2, 1, 1)
    FIELD(ME_IDR2, INTERRUPT1, 0, 1)
REG32(ME_IMR3, 0x5c)
    FIELD(ME_IMR3, INTERRUPT4, 3, 1)
    FIELD(ME_IMR3, INTERRUPT3, 2, 1)
    FIELD(ME_IMR3, INTERRUPT2, 1, 1)
    FIELD(ME_IMR3, INTERRUPT1, 0, 1)
REG32(ME_IER3, 0x60)
    FIELD(ME_IER3, INTERRUPT4, 3, 1)
    FIELD(ME_IER3, INTERRUPT3, 2, 1)
    FIELD(ME_IER3, INTERRUPT2, 1, 1)
    FIELD(ME_IER3, INTERRUPT1, 0, 1)
REG32(ME_IDR3, 0x64)
    FIELD(ME_IDR3, INTERRUPT4, 3, 1)
    FIELD(ME_IDR3, INTERRUPT3, 2, 1)
    FIELD(ME_IDR3, INTERRUPT2, 1, 1)
    FIELD(ME_IDR3, INTERRUPT1, 0, 1)
REG32(ME_IOR, 0x6c)
    FIELD(ME_IOR, OFFSET, 0, 5)
REG32(MPLL_CTRL, 0x100)
    FIELD(MPLL_CTRL, POST_SRC, 24, 3)
    FIELD(MPLL_CTRL, PRE_SRC, 20, 3)
    FIELD(MPLL_CTRL, CLKOUTDIV, 16, 2)
    FIELD(MPLL_CTRL, FBDIV, 8, 8)
    FIELD(MPLL_CTRL, BYPASS, 3, 1)
    FIELD(MPLL_CTRL, RESET, 0, 1)
REG32(MPLL_CFG, 0x104)
    FIELD(MPLL_CFG, LOCK_DLY, 25, 7)
    FIELD(MPLL_CFG, LOCK_CNT, 13, 10)
    FIELD(MPLL_CFG, LFHF, 10, 2)
    FIELD(MPLL_CFG, CP, 5, 4)
    FIELD(MPLL_CFG, RES, 0, 4)
REG32(MPLL_FRAC_CFG, 0x108)
    FIELD(MPLL_FRAC_CFG, ENABLED, 31, 1)
    FIELD(MPLL_FRAC_CFG, SEED, 22, 3)
    FIELD(MPLL_FRAC_CFG, ALGRTHM, 19, 1)
    FIELD(MPLL_FRAC_CFG, ORDER, 18, 1)
    FIELD(MPLL_FRAC_CFG, DATA, 0, 16)
REG32(ME_PLL_STATUS, 0x10c)
    FIELD(ME_PLL_STATUS, MPLL_STABLE, 2, 1)
    FIELD(ME_PLL_STATUS, MPLL_LOCK, 0, 1)
REG32(ME_PLL_REG0, 0x110)
    FIELD(ME_PLL_REG0, FREQ_COMP, 22, 3)
    FIELD(ME_PLL_REG0, LOCK_SAT_HIGH, 12, 10)
    FIELD(ME_PLL_REG0, UNLOCK_CNT, 2, 10)
REG32(ME_PLL_REG1, 0x114)
    FIELD(ME_PLL_REG1, EN_VCO_DIV1, 29, 1)
    FIELD(ME_PLL_REG1, EN_VCO_DIV6, 28, 1)
    FIELD(ME_PLL_REG1, EN_SYNC_CK_TEST, 27, 1)
    FIELD(ME_PLL_REG1, VCO_STARTUP_HYST_DISABLE, 25, 1)
    FIELD(ME_PLL_REG1, VCO_STARTUP_ALT_EN, 24, 1)
    FIELD(ME_PLL_REG1, VCO_STARTUP_ADJ, 23, 1)
    FIELD(ME_PLL_REG1, VCO_KICK_DISABLE, 22, 1)
    FIELD(ME_PLL_REG1, VCO_GATE_CCI_B, 21, 1)
    FIELD(ME_PLL_REG1, VCO_HIGH_RANGE_EN, 20, 1)
    FIELD(ME_PLL_REG1, PFD_STARTUP, 19, 1)
    FIELD(ME_PLL_REG1, PFD, 12, 7)
    FIELD(ME_PLL_REG1, SEL_SLIPD, 11, 1)
    FIELD(ME_PLL_REG1, SENSE_TEST_EN, 10, 1)
    FIELD(ME_PLL_REG1, SPARE_ANALOG, 5, 5)
    FIELD(ME_PLL_REG1, SPARE_DIGITAL, 0, 5)
REG32(ME_PLL_REG2, 0x118)
    FIELD(ME_PLL_REG2, SUP_SEL_VCCINT, 28, 1)
    FIELD(ME_PLL_REG2, SUP_SEL_VCCAUX, 27, 1)
    FIELD(ME_PLL_REG2, SUP_SEL_VBG, 26, 1)
    FIELD(ME_PLL_REG2, SUP_SEL_AVDD, 25, 1)
    FIELD(ME_PLL_REG2, SUP_SEL_VBGHALF, 24, 1)
    FIELD(ME_PLL_REG2, VLF_HIGH_SEL, 18, 2)
    FIELD(ME_PLL_REG2, VLF_VALID_SEL, 15, 3)
    FIELD(ME_PLL_REG2, VLF_VALID_PWDN, 14, 1)
    FIELD(ME_PLL_REG2, VLF_HIGH_PWDN_B, 13, 1)
    FIELD(ME_PLL_REG2, LF_LOW_SEL, 12, 1)
    FIELD(ME_PLL_REG2, LF_NEN, 10, 2)
    FIELD(ME_PLL_REG2, LF_PEN, 8, 2)
    FIELD(ME_PLL_REG2, MAN_LF, 4, 3)
    FIELD(ME_PLL_REG2, ANALOG_MISC, 0, 4)
REG32(ME_PLL_REG3, 0x11c)
    FIELD(ME_PLL_REG3, REGLPF_RES_SHORT, 24, 1)
    FIELD(ME_PLL_REG3, CP_RES_L, 22, 2)
    FIELD(ME_PLL_REG3, CP_RES_H, 20, 2)
    FIELD(ME_PLL_REG3, CP_OPAMP_BN, 19, 1)
    FIELD(ME_PLL_REG3, CP_BIAS_TRIP_SET, 18, 1)
    FIELD(ME_PLL_REG3, HVLF_CNT_TEST_EN, 17, 1)
    FIELD(ME_PLL_REG3, HVLF_CNT_TEST, 11, 6)
    FIELD(ME_PLL_REG3, AVDD_COMP_SET, 8, 3)
    FIELD(ME_PLL_REG3, AVDD_VBG_PD, 5, 3)
    FIELD(ME_PLL_REG3, AVDD_VBG_SEL, 0, 5)
REG32(ME_PLL_REG4, 0x120)
    FIELD(ME_PLL_REG4, FORCE_SENSE_SHORT, 29, 1)
    FIELD(ME_PLL_REG4, FRACT_TEST_SEL, 26, 3)
    FIELD(ME_PLL_REG4, FRACT_TEST_EN, 25, 1)
    FIELD(ME_PLL_REG4, TC_GEN_MODE, 24, 1)
    FIELD(ME_PLL_REG4, TESTOUT3_MUX_SEL, 18, 6)
    FIELD(ME_PLL_REG4, TESTOUT2_MUX_SEL, 12, 6)
    FIELD(ME_PLL_REG4, TESTOUT1_MUX_SEL, 6, 6)
    FIELD(ME_PLL_REG4, TESTOUT0_MUX_SEL, 0, 6)
REG32(ME_PLL_REG5, 0x124)
    FIELD(ME_PLL_REG5, VLF_SWITCH_SEL, 9, 3)
    FIELD(ME_PLL_REG5, VCO_SINGLE_BAND_DEFAULT, 8, 1)
    FIELD(ME_PLL_REG5, VCO_BAND_MODE, 7, 1)
    FIELD(ME_PLL_REG5, VLF_HIGH_EN, 6, 1)
    FIELD(ME_PLL_REG5, VLF_SWITCH_PWDN, 5, 1)
    FIELD(ME_PLL_REG5, TEST_PLL_CK_SEL, 2, 3)
    FIELD(ME_PLL_REG5, FRACT_TEST_CK_SEL, 1, 1)
    FIELD(ME_PLL_REG5, FBCLK_PREDIV2, 0, 1)
REG32(ME_PLL_REG6, 0x128)
    FIELD(ME_PLL_REG6, CONTROL_1, 16, 16)
    FIELD(ME_PLL_REG6, CONTROL_0, 0, 16)
REG32(ME_PLL_REG7, 0x12c)
    FIELD(ME_PLL_REG7, CONTROL_3, 16, 16)
    FIELD(ME_PLL_REG7, CONTROL_2, 0, 16)
REG32(ME_PLL_REG8, 0x130)
    FIELD(ME_PLL_REG8, CONTROL_5, 16, 16)
    FIELD(ME_PLL_REG8, CONTROL_4, 0, 16)
REG32(ME_PLL_REG9, 0x134)
    FIELD(ME_PLL_REG9, CONTROL_7, 16, 16)
    FIELD(ME_PLL_REG9, CONTROL_6, 0, 16)
REG32(ME_CORE_REF_CTRL, 0x138)
    FIELD(ME_CORE_REF_CTRL, CLKACT, 25, 1)
    FIELD(ME_CORE_REF_CTRL, DIVISOR0, 8, 10)
REG32(ME_LSBUS_REF_CTRL, 0x13c)
    FIELD(ME_LSBUS_REF_CTRL, CLKACT, 25, 1)
    FIELD(ME_LSBUS_REF_CTRL, DIVISOR0, 8, 10)
REG32(ME_DBG_REF_CTRL, 0x140)
    FIELD(ME_DBG_REF_CTRL, CLKACT, 25, 1)
    FIELD(ME_DBG_REF_CTRL, DIVISOR0, 8, 10)
REG32(ME_VERSION, 0x144)
    FIELD(ME_VERSION, ME_RTL_REV, 12, 8)
    FIELD(ME_VERSION, ME_CORE_REV, 4, 8)
    FIELD(ME_VERSION, ME_SILICON_REV, 0, 4)
REG32(ME_TOP_ROW, 0x148)
    FIELD(ME_TOP_ROW, ROW_OFFSET, 5, 5)
    FIELD(ME_TOP_ROW, ME_TOP_ROW, 0, 5)
REG32(RAM_EMA_CTRL, 0x160)
    FIELD(RAM_EMA_CTRL, RAM_RF2PHS_STOV, 14, 1)
    FIELD(RAM_EMA_CTRL, RAM_RF2PHS_EMASA, 13, 1)
    FIELD(RAM_EMA_CTRL, RAM_RF2PHS_EMAB, 10, 3)
    FIELD(RAM_EMA_CTRL, RAM_RF2PHS_EMAA, 7, 3)
    FIELD(RAM_EMA_CTRL, RAM_RFSPHD_STOV, 6, 1)
    FIELD(RAM_EMA_CTRL, RAM_RFSPHD_EMAS, 5, 1)
    FIELD(RAM_EMA_CTRL, RAM_RFSPHD_EMAW, 3, 2)
    FIELD(RAM_EMA_CTRL, RAM_RFSPHD_EMA, 0, 3)
REG32(ME_SPARE_CTRL, 0x200)
    FIELD(ME_SPARE_CTRL, ME_SPARE, 1, 31)
    FIELD(ME_SPARE_CTRL, PROTECTED_REG_EN, 0, 1)
REG32(ME_SMID_REG, 0x204)
    FIELD(ME_SMID_REG, ME_SMID, 0, 5)
REG32(ME_SECURE_REG, 0x208)
    FIELD(ME_SECURE_REG, ME_SECURE, 0, 1)
REG32(XPD_PRE_LOAD, 0x260)
REG32(XPD_EXPECTED, 0x264)
REG32(XPD_CTRL0, 0x268)
    FIELD(XPD_CTRL0, DELAY_SPARE, 25, 5)
    FIELD(XPD_CTRL0, CMP_SEL, 24, 1)
    FIELD(XPD_CTRL0, DELAY_CELL_TYPE, 19, 5)
    FIELD(XPD_CTRL0, DELAY_VT_TYPE, 17, 2)
    FIELD(XPD_CTRL0, DELAY_VALUE, 6, 11)
    FIELD(XPD_CTRL0, PATH_SEL, 0, 6)
REG32(XPD_CTRL1, 0x26c)
    FIELD(XPD_CTRL1, CLK_SPARE, 12, 4)
    FIELD(XPD_CTRL1, CLK_PHASE_SEL, 10, 2)
    FIELD(XPD_CTRL1, CLK_VT_TYPE, 8, 2)
    FIELD(XPD_CTRL1, CLK_CELL_TYPE, 6, 2)
    FIELD(XPD_CTRL1, CLK_INSERT_DLY, 2, 4)
    FIELD(XPD_CTRL1, CLK_SEL, 0, 2)
REG32(XPD_CTRL2, 0x270)
    FIELD(XPD_CTRL2, CTRL_SPARE, 1, 2)
    FIELD(XPD_CTRL2, ENABLE, 0, 1)
REG32(XPD_CTRL3, 0x274)
    FIELD(XPD_CTRL3, DCYCLE_HIGH_LOW, 2, 1)
    FIELD(XPD_CTRL3, DCYCLE_CNT_CLR, 1, 1)
    FIELD(XPD_CTRL3, DCYCLE_START, 0, 1)
REG32(XPD_SOFT_RST, 0x278)
    FIELD(XPD_SOFT_RST, CLK0, 0, 1)
REG32(XPD_STAT, 0x27c)
    FIELD(XPD_STAT, DCYCLE_CNT_VALUE, 2, 12)
    FIELD(XPD_STAT, CMP_RESULT, 1, 1)
    FIELD(XPD_STAT, CMP_DONE, 0, 1)

#define ME_NPI_R_MAX (R_XPD_STAT + 1)

#define LOCK_VAL 0xF9E8D7C6

typedef struct ME_NPI {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[ME_NPI_R_MAX];
    RegisterInfo regs_info[ME_NPI_R_MAX];
} ME_NPI;

static void me_npi_lock_postw(RegisterInfo *reg, uint64_t val64)
{
    ME_NPI *s = XILINX_ME_NPI(reg->opaque);
    bool locked = val64 != LOCK_VAL;

    ARRAY_FIELD_DP32(s->regs, REG_PCSR_STATUS, PCSRLOCK, locked);
}

static const RegisterAccessInfo me_npi_regs_info[] = {
    {   .name = "REG_PCSR_MASK",  .addr = A_REG_PCSR_MASK,
        .rsvd = 0xf0000000,
    },{ .name = "REG_PCSR_CONTROL",  .addr = A_REG_PCSR_CONTROL,
        .reset = 0x50381fe,
        .rsvd = 0xf0000000,
    },{ .name = "REG_PCSR_STATUS",  .addr = A_REG_PCSR_STATUS,
        .reset = R_REG_PCSR_STATUS_PCSRLOCK_MASK \
                 | R_REG_PCSR_STATUS_MEM_CLEAR_PASS_MASK \
                 | R_REG_PCSR_STATUS_MEM_CLEAR_DONE_MASK \
                 | R_REG_PCSR_STATUS_CALDONE_MASK \
                 | R_REG_PCSR_STATUS_SCAN_CLEAR_PASS_MASK \
                 | R_REG_PCSR_STATUS_SCAN_CLEAR_DONE_MASK \
                 | R_REG_PCSR_STATUS_ME_PWR_SUPPLY_MASK,
        .rsvd = 0xffff0000,
        .ro = 0xffff,
    },{ .name = "REG_PCSR_LOCK",  .addr = A_REG_PCSR_LOCK,
        .reset = 0x1,
        .post_write = me_npi_lock_postw
    },{ .name = "ME_ISR",  .addr = A_ME_ISR,
        .rsvd = 0xfffffff0,
        .w1c = 0xf,
    },{ .name = "ME_ITR",  .addr = A_ME_ITR,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IMR0",  .addr = A_ME_IMR0,
        .reset = 0xe,
        .rsvd = 0xfffffff0,
        .ro = 0xf,
    },{ .name = "ME_IER0",  .addr = A_ME_IER0,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IDR0",  .addr = A_ME_IDR0,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IMR1",  .addr = A_ME_IMR1,
        .reset = 0xf,
        .rsvd = 0xfffffff0,
        .ro = 0xf,
    },{ .name = "ME_IER1",  .addr = A_ME_IER1,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IDR1",  .addr = A_ME_IDR1,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IMR2",  .addr = A_ME_IMR2,
        .reset = 0xf,
        .rsvd = 0xfffffff0,
        .ro = 0xf,
    },{ .name = "ME_IER2",  .addr = A_ME_IER2,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IDR2",  .addr = A_ME_IDR2,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IMR3",  .addr = A_ME_IMR3,
        .reset = 0xf,
        .rsvd = 0xfffffff0,
        .ro = 0xf,
    },{ .name = "ME_IER3",  .addr = A_ME_IER3,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IDR3",  .addr = A_ME_IDR3,
        .rsvd = 0xfffffff0,
    },{ .name = "ME_IOR",  .addr = A_ME_IOR,
        .reset = 0x4,
        .rsvd = 0xffffffe0,
    },{ .name = "MPLL_CTRL",  .addr = A_MPLL_CTRL,
        .reset = 0x14809,
        .rsvd = 0xf88c00f6,
        .ro = 0x8c00f6,
    },{ .name = "MPLL_CFG",  .addr = A_MPLL_CFG,
        .reset = 0x2000000,
        .rsvd = 0x1801210,
        .ro = 0x1801210,
    },{ .name = "MPLL_FRAC_CFG",  .addr = A_MPLL_FRAC_CFG,
        .rsvd = 0x7e330000,
        .ro = 0x7e330000,
    },{ .name = "ME_PLL_STATUS",  .addr = A_ME_PLL_STATUS,
        .reset = R_ME_PLL_STATUS_MPLL_LOCK_MASK |
                 R_ME_PLL_STATUS_MPLL_STABLE_MASK,
        .rsvd = 0xfa,
        .ro = 0xff,
    },{ .name = "ME_PLL_REG0",  .addr = A_ME_PLL_REG0,
        .reset = 0xfe9004,
        .rsvd = 0xfe000003,
        .ro = 0x3,
    },{ .name = "ME_PLL_REG1",  .addr = A_ME_PLL_REG1,
        .reset = 0x2b1020,
        .rsvd = 0xc4000000,
        .ro = 0x4000000,
    },{ .name = "ME_PLL_REG2",  .addr = A_ME_PLL_REG2,
        .reset = 0x9aa00,
        .rsvd = 0xe0f00080,
        .ro = 0xf00080,
    },{ .name = "ME_PLL_REG3",  .addr = A_ME_PLL_REG3,
        .reset = 0x803c9,
        .rsvd = 0xfe000000,
    },{ .name = "ME_PLL_REG4",  .addr = A_ME_PLL_REG4,
        .rsvd = 0xc0000000,
    },{ .name = "ME_PLL_REG5",  .addr = A_ME_PLL_REG5,
        .reset = 0x841,
        .rsvd = 0xfffff000,
    },{ .name = "ME_PLL_REG6",  .addr = A_ME_PLL_REG6,
        .reset = 0x7c4df37c,
    },{ .name = "ME_PLL_REG7",  .addr = A_ME_PLL_REG7,
        .reset = 0xebd8d042,
    },{ .name = "ME_PLL_REG8",  .addr = A_ME_PLL_REG8,
        .reset = 0xedfbec5f,
    },{ .name = "ME_PLL_REG9",  .addr = A_ME_PLL_REG9,
        .reset = 0x4428aacd,
    },{ .name = "ME_CORE_REF_CTRL",  .addr = A_ME_CORE_REF_CTRL,
        .reset = 0x2000200,
        .rsvd = 0xfdfc00ff,
        .ro = 0x1fc00ff,
    },{ .name = "ME_LSBUS_REF_CTRL",  .addr = A_ME_LSBUS_REF_CTRL,
        .reset = 0x2002500,
        .rsvd = 0xfdfc00ff,
        .ro = 0x1fc00ff,
    },{ .name = "ME_DBG_REF_CTRL",  .addr = A_ME_DBG_REF_CTRL,
        .reset = 0x2002500,
        .rsvd = 0xfdfc00ff,
        .ro = 0x1fc00ff,
    },{ .name = "ME_VERSION",  .addr = A_ME_VERSION,
        .reset = 0xffff1,
        .rsvd = 0xfff00000,
        .ro = 0xfffff,
    },{ .name = "ME_TOP_ROW",  .addr = A_ME_TOP_ROW,
        .reset = 0x1f,
        .rsvd = 0xfffffc00,
    },{ .name = "RAM_EMA_CTRL",  .addr = A_RAM_EMA_CTRL,
        .reset = 0xd8b,
        .rsvd = 0xffff8000,
    },{ .name = "ME_SPARE_CTRL",  .addr = A_ME_SPARE_CTRL,
    },{ .name = "ME_SMID_REG",  .addr = A_ME_SMID_REG,
        .rsvd = 0xffffffe0,
    },{ .name = "ME_SECURE_REG",  .addr = A_ME_SECURE_REG,
        .reset = 0x1,
        .rsvd = 0xfffffffe,
    },{ .name = "XPD_PRE_LOAD",  .addr = A_XPD_PRE_LOAD,
    },{ .name = "XPD_EXPECTED",  .addr = A_XPD_EXPECTED,
    },{ .name = "XPD_CTRL0",  .addr = A_XPD_CTRL0,
        .rsvd = 0xc0000000,
    },{ .name = "XPD_CTRL1",  .addr = A_XPD_CTRL1,
        .rsvd = 0xffff0000,
    },{ .name = "XPD_CTRL2",  .addr = A_XPD_CTRL2,
        .rsvd = 0xfffffff8,
    },{ .name = "XPD_CTRL3",  .addr = A_XPD_CTRL3,
        .rsvd = 0xfffffff8,
    },{ .name = "XPD_SOFT_RST",  .addr = A_XPD_SOFT_RST,
        .rsvd = 0xfffffffe,
    },{ .name = "XPD_STAT",  .addr = A_XPD_STAT,
        .rsvd = 0xffffc000,
        .ro = 0x3fff,
    }
};

static void me_npi_reset_enter(Object *obj, ResetType type)
{
    ME_NPI *s = XILINX_ME_NPI(obj);
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(s->regs_info); ++i) {
        register_reset(&s->regs_info[i]);
    }
}

static const MemoryRegionOps me_npi_ops = {
    .read = register_read_memory,
    .write_with_attrs = register_write_memory_with_attrs,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void me_npi_realize(DeviceState *dev, Error **errp)
{
    /* Delete this if you don't need it */
}

static void me_npi_init(Object *obj)
{
    ME_NPI *s = XILINX_ME_NPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    RegisterInfoArray *reg_array;

    memory_region_init(&s->iomem, obj, TYPE_XILINX_ME_NPI, ME_NPI_R_MAX * 4);
    reg_array =
        register_init_block32(DEVICE(obj), me_npi_regs_info,
                              ARRAY_SIZE(me_npi_regs_info),
                              s->regs_info, s->regs,
                              &me_npi_ops,
                              XILINX_ME_NPI_ERR_DEBUG,
                              ME_NPI_R_MAX * 4);
    memory_region_add_subregion(&s->iomem,
                                0x0,
                                &reg_array->mem);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_me_npi = {
    .name = TYPE_XILINX_ME_NPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ME_NPI, ME_NPI_R_MAX),
        VMSTATE_END_OF_LIST(),
    }
};

static void me_npi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = me_npi_realize;
    dc->vmsd = &vmstate_me_npi;
    rc->phases.enter = me_npi_reset_enter;
}

static const TypeInfo me_npi_info = {
    .name          = TYPE_XILINX_ME_NPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ME_NPI),
    .class_init    = me_npi_class_init,
    .instance_init = me_npi_init,
};

static void me_npi_register_types(void)
{
    type_register_static(&me_npi_info);
}

type_init(me_npi_register_types)