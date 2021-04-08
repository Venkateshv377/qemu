/*
 * SD Association Host Standard Specification v2.0 controller emulation
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Mitsyanko Igor <i.mitsyanko@samsung.com>
 * Peter A.G. Crosthwaite <peter.crosthwaite@petalogix.com>
 *
 * Based on MMC controller for Samsung S5PC1xx-based board emulation
 * by Alexey Merkulov and Vladimir Monakhov.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "sysemu/dma.h"
#include "qemu/timer.h"
#include "qemu/bitops.h"
#include "hw/sd/sdhci.h"
#include "migration/vmstate.h"
#include "sdhci-internal.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

#define DEBUG_LOG   1

#define TYPE_SDHCI_BUS "sdhci-bus"
#define SDHCI_BUS(obj) OBJECT_CHECK(SDBus, (obj), TYPE_SDHCI_BUS)

#define MASKED_WRITE(reg, mask, val)  (reg = (reg & (mask)) | (val))

/* Default SD/MMC host controller features information, which will be
 * presented in CAPABILITIES register of generic SD host controller at reset.
 *
 * support:
 * - 3.3v and 1.8v voltages
 * - SDMA/ADMA1/ADMA2
 * - high-speed
 * max host controller R/W buffers size: 512B
 * max clock frequency for SDclock: 52 MHz
 * timeout clock frequency: 52 MHz
 *
 * does not support:
 * - 3.0v voltage
 * - 64-bit system bus
 * - suspend/resume
 */
#define SDHC_CAPAB_REG_DEFAULT 0x057834b4

uint32_t cqhci_caps[] = {ITCFMUL_0h, ITCFMUL_1h, ITCFMUL_2h, ITCFMUL_3h, ITCFMUL_4h};
static struct CQHCIState *cqhci;

SDHCIState *variable; // Workaround fix for Error "Card stuck at wrong state

static inline unsigned int sdhci_get_fifolen(SDHCIState *s)
{
    return 1 << (9 + FIELD_EX32(s->capareg, SDHC_CAPAB, MAXBLOCKLENGTH));
}

/* return true on error */
static bool sdhci_check_capab_freq_range(SDHCIState *s, const char *desc,
        uint8_t freq, Error **errp)
{
    if (s->sd_spec_version >= 3) {
        return false;
    }
    switch (freq) {
    case 0:
    case 10 ... 63:
        break;
    default:
        error_setg(errp, "SD %s clock frequency can have value"
                "in range 0-63 only", desc);
        return true;
    }
    return false;
}

static void sdhci_check_capareg(SDHCIState *s, Error **errp)
{
    uint64_t msk = s->capareg;
    uint32_t val;
    bool y;

    switch (s->sd_spec_version) {
    case 4:
        val = FIELD_EX64(s->capareg, SDHC_CAPAB, BUS64BIT_V4);
        trace_sdhci_capareg("64-bit system bus (v4)", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, BUS64BIT_V4, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, UHS_II);
        trace_sdhci_capareg("UHS-II", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, UHS_II, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, ADMA3);
        trace_sdhci_capareg("ADMA3", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, ADMA3, 0);

        /* fallthrough */
    case 3:
        val = FIELD_EX64(s->capareg, SDHC_CAPAB, ASYNC_INT);
        trace_sdhci_capareg("async interrupt", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, ASYNC_INT, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, SLOT_TYPE);
        if (val) {
            error_setg(errp, "slot-type not supported");
            return;
        }
        trace_sdhci_capareg("slot type", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, SLOT_TYPE, 0);

        if (val != 2) {
            val = FIELD_EX64(s->capareg, SDHC_CAPAB, EMBEDDED_8BIT);
            trace_sdhci_capareg("8-bit bus", val);
        }
        msk = FIELD_DP64(msk, SDHC_CAPAB, EMBEDDED_8BIT, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, BUS_SPEED);
        trace_sdhci_capareg("bus speed mask", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, BUS_SPEED, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, DRIVER_STRENGTH);
        trace_sdhci_capareg("driver strength mask", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, DRIVER_STRENGTH, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, TIMER_RETUNING);
        trace_sdhci_capareg("timer re-tuning", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, TIMER_RETUNING, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, SDR50_TUNING);
        trace_sdhci_capareg("use SDR50 tuning", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, SDR50_TUNING, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, RETUNING_MODE);
        trace_sdhci_capareg("re-tuning mode", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, RETUNING_MODE, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, CLOCK_MULT);
        trace_sdhci_capareg("clock multiplier", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, CLOCK_MULT, 0);

        /* fallthrough */
    case 2: /* default version */
        val = FIELD_EX64(s->capareg, SDHC_CAPAB, ADMA2);
        trace_sdhci_capareg("ADMA2", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, ADMA2, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, ADMA1);
        trace_sdhci_capareg("ADMA1", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, ADMA1, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, BUS64BIT);
        trace_sdhci_capareg("64-bit system bus (v3)", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, BUS64BIT, 0);

        /* fallthrough */
    case 1:
        y = FIELD_EX64(s->capareg, SDHC_CAPAB, TOUNIT);
        msk = FIELD_DP64(msk, SDHC_CAPAB, TOUNIT, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, TOCLKFREQ);
        trace_sdhci_capareg(y ? "timeout (MHz)" : "Timeout (KHz)", val);
        if (sdhci_check_capab_freq_range(s, "timeout", val, errp)) {
            return;
        }
        msk = FIELD_DP64(msk, SDHC_CAPAB, TOCLKFREQ, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, BASECLKFREQ);
        trace_sdhci_capareg(y ? "base (MHz)" : "Base (KHz)", val);
        if (sdhci_check_capab_freq_range(s, "base", val, errp)) {
            return;
        }
        msk = FIELD_DP64(msk, SDHC_CAPAB, BASECLKFREQ, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, MAXBLOCKLENGTH);
        if (val >= 3) {
            error_setg(errp, "block size can be 512, 1024 or 2048 only");
            return;
        }
        trace_sdhci_capareg("max block length", sdhci_get_fifolen(s));
        msk = FIELD_DP64(msk, SDHC_CAPAB, MAXBLOCKLENGTH, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, HIGHSPEED);
        trace_sdhci_capareg("high speed", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, HIGHSPEED, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, SDMA);
        trace_sdhci_capareg("SDMA", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, SDMA, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, SUSPRESUME);
        trace_sdhci_capareg("suspend/resume", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, SUSPRESUME, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, V33);
        trace_sdhci_capareg("3.3v", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, V33, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, V30);
        trace_sdhci_capareg("3.0v", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, V30, 0);

        val = FIELD_EX64(s->capareg, SDHC_CAPAB, V18);
        trace_sdhci_capareg("1.8v", val);
        msk = FIELD_DP64(msk, SDHC_CAPAB, V18, 0);
        break;

    default:
            error_setg(errp, "Unsupported spec version: %u", s->sd_spec_version);
    }
    if (msk) {
        qemu_log_mask(LOG_UNIMP,
                "SDHCI: unknown CAPAB mask: 0x%016" PRIx64 "\n", msk);
    }
}

static uint8_t sdhci_slotint(SDHCIState *s)
{
    return (s->norintsts & s->norintsigen) || (s->errintsts & s->errintsigen) ||
        ((s->norintsts & SDHC_NIS_INSERT) && (s->wakcon & SDHC_WKUP_ON_INS)) ||
        ((s->norintsts & SDHC_NIS_REMOVE) && (s->wakcon & SDHC_WKUP_ON_RMV));
}

static inline void sdhci_update_irq(SDHCIState *s)
{
    fprintf(stdout, "%s: %d: Updating IRQ\n", __func__, __LINE__);
    qemu_set_irq(s->irq, sdhci_slotint(s));
}

static void sdhci_raise_insertion_irq(void *opaque)
{
    SDHCIState *s = (SDHCIState *)opaque;

    if (s->norintsts & SDHC_NIS_REMOVE) {
        timer_mod(s->insert_timer,
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SDHC_INSERTION_DELAY);
    } else {
        s->prnsts = 0x1ff0000;
        if (s->norintstsen & SDHC_NISEN_INSERT) {
            s->norintsts |= SDHC_NIS_INSERT;
        }
        sdhci_update_irq(s);
    }
}

static void sdhci_set_inserted(DeviceState *dev, bool level)
{
    SDHCIState *s = (SDHCIState *)dev;

    trace_sdhci_set_inserted(level ? "insert" : "eject");
    if ((s->norintsts & SDHC_NIS_REMOVE) && level) {
        /* Give target some time to notice card ejection */
        timer_mod(s->insert_timer,
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SDHC_INSERTION_DELAY);
    } else {
        if (level) {
            s->prnsts = 0x1ff0000;
            if (s->norintstsen & SDHC_NISEN_INSERT) {
                s->norintsts |= SDHC_NIS_INSERT;
            }
        } else {
            s->prnsts = 0x1fa0000;
            s->pwrcon &= ~SDHC_POWER_ON;
            s->clkcon &= ~SDHC_CLOCK_SDCLK_EN;
            if (s->norintstsen & SDHC_NISEN_REMOVE) {
                s->norintsts |= SDHC_NIS_REMOVE;
            }
        }
        sdhci_update_irq(s);
    }
}

static void sdhci_set_readonly(DeviceState *dev, bool level)
{
    SDHCIState *s = (SDHCIState *)dev;

    if (level) {
        s->prnsts &= ~SDHC_WRITE_PROTECT;
    } else {
        /* Write enabled */
        s->prnsts |= SDHC_WRITE_PROTECT;
    }
}

static void sdhci_reset(SDHCIState *s)
{
    DeviceState *dev = DEVICE(s);

    timer_del(s->insert_timer);
    timer_del(s->transfer_timer);

    /* Set all registers to 0. Capabilities/Version registers are not cleared
     * and assumed to always preserve their value, given to them during
     * initialization */
    memset(&s->sdmasysad, 0, (uintptr_t)&s->capareg - (uintptr_t)&s->sdmasysad);

    /* Reset other state based on current card insertion/readonly status */
    sdhci_set_inserted(dev, sdbus_get_inserted(&s->sdbus));
    sdhci_set_readonly(dev, sdbus_get_readonly(&s->sdbus));

    s->data_count = 0;
    s->stopped_state = sdhc_not_stopped;
    s->pending_insert_state = false;
}

static void sdhci_poweron_reset(DeviceState *dev)
{
    /* QOM (ie power-on) reset. This is identical to reset
     * commanded via device register apart from handling of the
     * 'pending insert on powerup' quirk.
     */
    SDHCIState *s = (SDHCIState *)dev;

    sdhci_reset(s);

    if (s->pending_insert_quirk) {
        s->pending_insert_state = true;
    }
}

static void sdhci_data_transfer(void *opaque);

static void sdhci_send_command(SDHCIState *s)
{
    SDRequest request;
    uint8_t response[16];
    int rlen;

    s->errintsts = 0;
    s->acmd12errsts = 0;
    request.cmd = s->cmdreg >> 8;
    request.arg = s->argument;

    trace_sdhci_send_command(request.cmd, request.arg);
    rlen = sdbus_do_command(&s->sdbus, &request, response);
    fprintf(stdout, "%s: request.cmd: %d\trequest.arg: 0x%lx\ts->cmdreg: 0x%x\ts->blksize: 0x%x\n", __func__, request.cmd, request.arg, s->cmdreg, s->blksize);

    if (s->cmdreg & SDHC_CMD_RESPONSE) {
        if (rlen == 4) {
            s->rspreg[0] = ldl_be_p(response);
            s->rspreg[1] = s->rspreg[2] = s->rspreg[3] = 0;
            trace_sdhci_response4(s->rspreg[0]);
        } else if (rlen == 16) {
            s->rspreg[0] = ldl_be_p(&response[11]);
            s->rspreg[1] = ldl_be_p(&response[7]);
            s->rspreg[2] = ldl_be_p(&response[3]);
            s->rspreg[3] = (response[0] << 16) | (response[1] << 8) |
                response[2];
            trace_sdhci_response16(s->rspreg[3], s->rspreg[2],
                    s->rspreg[1], s->rspreg[0]);
        } else {
            trace_sdhci_error("timeout waiting for command response");
            if (s->errintstsen & SDHC_EISEN_CMDTIMEOUT) {
                s->errintsts |= SDHC_EIS_CMDTIMEOUT;
                s->norintsts |= SDHC_NIS_ERR;
            }
        }

        if (!(s->quirks & SDHCI_QUIRK_NO_BUSY_IRQ) &&
                (s->norintstsen & SDHC_NISEN_TRSCMP) &&
                (s->cmdreg & SDHC_CMD_RESPONSE) == SDHC_CMD_RSP_WITH_BUSY) {
            s->norintsts |= SDHC_NIS_TRSCMP;
        }
    }

    if (s->norintstsen & SDHC_NISEN_CMDCMP) {
        s->norintsts |= SDHC_NIS_CMDCMP;
    }

    sdhci_update_irq(s);

    if (s->blksize && (s->cmdreg & SDHC_CMD_DATA_PRESENT)) {
        s->data_count = 0;
        sdhci_data_transfer(s);
    }
}

static void sdhci_end_transfer(SDHCIState *s)
{
    /* Automatically send CMD12 to stop transfer if AutoCMD12 enabled */
    fprintf(stdout, "%s: %d: Issue ACMD12\n", __func__, __LINE__);
    if ((s->trnmod & SDHC_TRNS_ACMD12) != 0) {
        SDRequest request;
        uint8_t response[16];

        request.cmd = 0x0C;
        request.arg = 0;
        trace_sdhci_end_transfer(request.cmd, request.arg);
        sdbus_do_command(&s->sdbus, &request, response);
        /* Auto CMD12 response goes to the upper Response register */
        s->rspreg[3] = ldl_be_p(response);
    }

    s->prnsts &= ~(SDHC_DOING_READ | SDHC_DOING_WRITE |
            SDHC_DAT_LINE_ACTIVE | SDHC_DATA_INHIBIT |
            SDHC_SPACE_AVAILABLE | SDHC_DATA_AVAILABLE);

    if (s->norintstsen & SDHC_NISEN_TRSCMP) {
        s->norintsts |= SDHC_NIS_TRSCMP;
    }

    sdhci_update_irq(s);
}

/*
 * Programmed i/o data transfer
 */
#define BLOCK_SIZE_MASK (4 * KiB - 1)

/* Fill host controller's read buffer with BLKSIZE bytes of data from card */
static void sdhci_read_block_from_card(SDHCIState *s)
{
    fprintf(stdout, "%s: %d: CARD to FIFO\n", __func__, __LINE__);
    int index = 0;
    uint8_t data;
    const uint16_t blk_size = s->blksize & BLOCK_SIZE_MASK;

    if ((s->trnmod & SDHC_TRNS_MULTI) &&
            (s->trnmod & SDHC_TRNS_BLK_CNT_EN) && (s->blkcnt == 0)) {
        return;
    }

    for (index = 0; index < blk_size; index++) {
        data = sdbus_read_data(&s->sdbus);
        if (!FIELD_EX32(s->hostctl2, SDHC_HOSTCTL2, EXECUTE_TUNING)) {
            /* Device is not in tuning */
            s->fifo_buffer[index] = data;
        }
    }

    /* New data now available for READ through Buffer Port Register */
    s->prnsts |= SDHC_DATA_AVAILABLE;
    if (s->norintstsen & SDHC_NISEN_RBUFRDY) {
        s->norintsts |= SDHC_NIS_RBUFRDY;
    }

    if (FIELD_EX32(s->hostctl2, SDHC_HOSTCTL2, EXECUTE_TUNING)) {
        /* Device is in tuning */
        s->hostctl2 &= ~R_SDHC_HOSTCTL2_EXECUTE_TUNING_MASK;
        s->hostctl2 |= R_SDHC_HOSTCTL2_SAMPLING_CLKSEL_MASK;
        s->prnsts &= ~(SDHC_DAT_LINE_ACTIVE | SDHC_DOING_READ |
                SDHC_DATA_INHIBIT);
        goto read_done;
    }

    /* Clear DAT line active status if that was the last block */
    if ((s->trnmod & SDHC_TRNS_MULTI) == 0 ||
            ((s->trnmod & SDHC_TRNS_MULTI) && s->blkcnt == 1)) {
        s->prnsts &= ~SDHC_DAT_LINE_ACTIVE;
    }

    /* If stop at block gap request was set and it's not the last block of
     * data - generate Block Event interrupt */
    if (s->stopped_state == sdhc_gap_read && (s->trnmod & SDHC_TRNS_MULTI) &&
            s->blkcnt != 1)    {
        s->prnsts &= ~SDHC_DAT_LINE_ACTIVE;
        if (s->norintstsen & SDHC_EISEN_BLKGAP) {
            s->norintsts |= SDHC_EIS_BLKGAP;
        }
    }

read_done:
    sdhci_update_irq(s);
}

/* Read @size byte of data from host controller @s BUFFER DATA PORT register */
static uint32_t sdhci_read_dataport(SDHCIState *s, unsigned size)
{
    fprintf(stdout, "%s: %d: READ from DATA PORT reg\n", __func__, __LINE__);
    uint32_t value = 0;
    int i;

    /* first check that a valid data exists in host controller input buffer */
    if ((s->prnsts & SDHC_DATA_AVAILABLE) == 0) {
        trace_sdhci_error("read from empty buffer");
        return 0;
    }

    for (i = 0; i < size; i++) {
        value |= s->fifo_buffer[s->data_count] << i * 8;
        s->data_count++;
        /* check if we've read all valid data (blksize bytes) from buffer */
        if ((s->data_count) >= (s->blksize & BLOCK_SIZE_MASK)) {
            trace_sdhci_read_dataport(s->data_count);
            s->prnsts &= ~SDHC_DATA_AVAILABLE; /* no more data in a buffer */
            s->data_count = 0;  /* next buff read must start at position [0] */

            if (s->trnmod & SDHC_TRNS_BLK_CNT_EN) {
                s->blkcnt--;
            }

            /* if that was the last block of data */
            if ((s->trnmod & SDHC_TRNS_MULTI) == 0 ||
                    ((s->trnmod & SDHC_TRNS_BLK_CNT_EN) && (s->blkcnt == 0)) ||
                    /* stop at gap request */
                    (s->stopped_state == sdhc_gap_read &&
                     !(s->prnsts & SDHC_DAT_LINE_ACTIVE))) {
                sdhci_end_transfer(s);
            } else { /* if there are more data, read next block from card */
                sdhci_read_block_from_card(s);
            }
            break;
        }
    }

    return value;
}

/* Write data from host controller FIFO to card */
static void sdhci_write_block_to_card(SDHCIState *s)
{
    int index = 0;
    fprintf(stdout, "%s: %d: FIFO to CARD\n", __func__, __LINE__);

    if (s->prnsts & SDHC_SPACE_AVAILABLE) {
        if (s->norintstsen & SDHC_NISEN_WBUFRDY) {
            s->norintsts |= SDHC_NIS_WBUFRDY;
        }
        sdhci_update_irq(s);
        return;
    }

    if (s->trnmod & SDHC_TRNS_BLK_CNT_EN) {
        if (s->blkcnt == 0) {
            return;
        } else {
            s->blkcnt--;
        }
    }

    for (index = 0; index < (s->blksize & BLOCK_SIZE_MASK); index++) {
        sdbus_write_data(&s->sdbus, s->fifo_buffer[index]);
    }

    /* Next data can be written through BUFFER DATORT register */
    s->prnsts |= SDHC_SPACE_AVAILABLE;

    /* Finish transfer if that was the last block of data */
    if ((s->trnmod & SDHC_TRNS_MULTI) == 0 ||
            ((s->trnmod & SDHC_TRNS_MULTI) &&
             (s->trnmod & SDHC_TRNS_BLK_CNT_EN) && (s->blkcnt == 0))) {
        sdhci_end_transfer(s);
    } else if (s->norintstsen & SDHC_NISEN_WBUFRDY) {
        s->norintsts |= SDHC_NIS_WBUFRDY;
    }

    /* Generate Block Gap Event if requested and if not the last block */
    if (s->stopped_state == sdhc_gap_write && (s->trnmod & SDHC_TRNS_MULTI) &&
            s->blkcnt > 0) {
        s->prnsts &= ~SDHC_DOING_WRITE;
        if (s->norintstsen & SDHC_EISEN_BLKGAP) {
            s->norintsts |= SDHC_EIS_BLKGAP;
        }
        sdhci_end_transfer(s);
    }

    sdhci_update_irq(s);
}

/* Write @size bytes of @value data to host controller @s Buffer Data Port
 * register */
static void sdhci_write_dataport(SDHCIState *s, uint32_t value, unsigned size)
{
    unsigned i;

    /* Check that there is free space left in a buffer */
    if (!(s->prnsts & SDHC_SPACE_AVAILABLE)) {
        trace_sdhci_error("Can't write to data buffer: buffer full");
        return;
    }

    for (i = 0; i < size; i++) {
        s->fifo_buffer[s->data_count] = value & 0xFF;
        s->data_count++;
        value >>= 8;
        if (s->data_count >= (s->blksize & BLOCK_SIZE_MASK)) {
            trace_sdhci_write_dataport(s->data_count);
            s->data_count = 0;
            s->prnsts &= ~SDHC_SPACE_AVAILABLE;
            if (s->prnsts & SDHC_DOING_WRITE) {
                sdhci_write_block_to_card(s);
            }
        }
    }
}

/*
 * Single DMA data transfer
 */

/* Multi block SDMA transfer */
static void sdhci_sdma_transfer_multi_blocks(SDHCIState *s)
{
    fprintf(stdout, "%s: %d: MULTI_BLOCK_TRANSFER (SDMA)\n", __func__, __LINE__);
    bool page_aligned = false;
    unsigned int n, begin;
    const uint16_t block_size = s->blksize & BLOCK_SIZE_MASK;
    uint32_t boundary_chk = 1 << (((s->blksize & ~BLOCK_SIZE_MASK) >> 12) + 12);
    uint32_t boundary_count = boundary_chk - (s->sdmasysad % boundary_chk);

    if (!(s->trnmod & SDHC_TRNS_BLK_CNT_EN) || !s->blkcnt) {
        qemu_log_mask(LOG_UNIMP, "infinite transfer is not supported\n");
        return;
    }

    /* XXX: Some sd/mmc drivers (for example, u-boot-slp) do not account for
     * possible stop at page boundary if initial address is not page aligned,
     * allow them to work properly */
    if ((s->sdmasysad % boundary_chk) == 0) {
        page_aligned = true;
    }

    if (s->trnmod & SDHC_TRNS_READ) {
        s->prnsts |= SDHC_DOING_READ | SDHC_DATA_INHIBIT |
            SDHC_DAT_LINE_ACTIVE;
        while (s->blkcnt) {
            if (s->data_count == 0) {
                for (n = 0; n < block_size; n++) {
                    s->fifo_buffer[n] = sdbus_read_data(&s->sdbus);
                }
            }
            begin = s->data_count;
            if (((boundary_count + begin) < block_size) && page_aligned) {
                s->data_count = boundary_count + begin;
                boundary_count = 0;
            } else {
                s->data_count = block_size;
                boundary_count -= block_size - begin;
                if (s->trnmod & SDHC_TRNS_BLK_CNT_EN) {
                    s->blkcnt--;
                }
            }
            dma_memory_rw_attr(s->dma_as, s->sdmasysad,
                    &s->fifo_buffer[begin], s->data_count - begin,
                    DMA_DIRECTION_FROM_DEVICE, *s->memattr);
            s->sdmasysad += s->data_count - begin;
            if (s->data_count == block_size) {
                s->data_count = 0;
            }
            if (page_aligned && boundary_count == 0) {
                break;
            }
        }
    } else {
        s->prnsts |= SDHC_DOING_WRITE | SDHC_DATA_INHIBIT |
            SDHC_DAT_LINE_ACTIVE;
        while (s->blkcnt) {
            begin = s->data_count;
            if (((boundary_count + begin) < block_size) && page_aligned) {
                s->data_count = boundary_count + begin;
                boundary_count = 0;
            } else {
                s->data_count = block_size;
                boundary_count -= block_size - begin;
            }
            dma_memory_rw_attr(s->dma_as, s->sdmasysad,
                    &s->fifo_buffer[begin], s->data_count - begin,
                    DMA_DIRECTION_TO_DEVICE, *s->memattr);
            s->sdmasysad += s->data_count - begin;
            if (s->data_count == block_size) {
                for (n = 0; n < block_size; n++) {
                    sdbus_write_data(&s->sdbus, s->fifo_buffer[n]);
                }
                s->data_count = 0;
                if (s->trnmod & SDHC_TRNS_BLK_CNT_EN) {
                    s->blkcnt--;
                }
            }
            if (page_aligned && boundary_count == 0) {
                break;
            }
        }
    }

    if (s->blkcnt == 0) {
        sdhci_end_transfer(s);
    } else {
        if (s->norintstsen & SDHC_NISEN_DMA) {
            s->norintsts |= SDHC_NIS_DMA;
        }
        sdhci_update_irq(s);
    }
}

/* single block SDMA transfer */
static void sdhci_sdma_transfer_single_block(SDHCIState *s)
{
    fprintf(stdout, "%s: %d: SINGLE BLOCK TRANSFER (SDMA)\n", __func__, __LINE__);
    int n;
    uint32_t datacnt = s->blksize & BLOCK_SIZE_MASK;

    if (s->trnmod & SDHC_TRNS_READ) {
        for (n = 0; n < datacnt; n++) {
            s->fifo_buffer[n] = sdbus_read_data(&s->sdbus);
        }
        dma_memory_rw_attr(s->dma_as, s->sdmasysad, s->fifo_buffer, datacnt,
                DMA_DIRECTION_FROM_DEVICE, *s->memattr);
    } else {
        dma_memory_rw_attr(s->dma_as, s->sdmasysad, s->fifo_buffer, datacnt,
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        for (n = 0; n < datacnt; n++) {
            sdbus_write_data(&s->sdbus, s->fifo_buffer[n]);
        }
    }
    s->blkcnt--;

    sdhci_end_transfer(s);
}

typedef struct ADMADescr {
    hwaddr addr;
    uint16_t length;
    uint8_t attr;
    uint8_t incr;
} ADMADescr;

static void get_adma_description(SDHCIState *s, ADMADescr *dscr)
{
    uint32_t adma1 = 0;
    uint64_t adma2 = 0;
    fprintf(stdout, "%s: SDHCI: s->admasysaddr: 0x%lx\n", __func__, s->admasysaddr);
    hwaddr entry_addr = (hwaddr)s->admasysaddr;
    switch (SDHC_DMA_TYPE(s->hostctl1)) {
    case SDHC_CTRL_ADMA2_32:
        dma_memory_rw_attr(s->dma_as, entry_addr, (uint8_t *)&adma2,
                sizeof(adma2),
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        adma2 = le64_to_cpu(adma2);
        fprintf(stdout, "SDHCI: adma2: 0x%lx\n", adma2);
        /* The spec does not specify endianness of descriptor table.
         * We currently assume that it is LE.
         */
        dscr->addr = (hwaddr)extract64(adma2, 32, 32) & ~0x3ull;
        dscr->length = (uint16_t)extract64(adma2, 16, 16);
        dscr->attr = (uint8_t)extract64(adma2, 0, 7);
        dscr->incr = 8;
        break;
    case SDHC_CTRL_ADMA1_32:
        dma_memory_rw_attr(s->dma_as, entry_addr, (uint8_t *)&adma1,
                sizeof(adma1),
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        adma1 = le32_to_cpu(adma1);
        fprintf(stdout, "SDHCI: adma1: 0x%x\n", adma1);
        dscr->addr = (hwaddr)(adma1 & 0xFFFFF000);
        dscr->attr = (uint8_t)extract32(adma1, 0, 7);
        dscr->incr = 4;
        if ((dscr->attr & SDHC_ADMA_ATTR_ACT_MASK) == SDHC_ADMA_ATTR_SET_LEN) {
            dscr->length = (uint16_t)extract32(adma1, 12, 16);
        } else {
            dscr->length = 4 * KiB;
        }
        break;
    case SDHC_CTRL_ADMA2_64:
        fprintf(stdout, "SDHCI: adma2_64: 0x%x\n", adma1);
        dma_memory_rw_attr(s->dma_as, entry_addr,
                (uint8_t *)(&dscr->attr), 1,
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        dma_memory_rw_attr(s->dma_as, entry_addr + 2,
                (uint8_t *)(&dscr->length), 2,
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        dscr->length = le16_to_cpu(dscr->length);
        dma_memory_rw_attr(s->dma_as, entry_addr + 4,
                (uint8_t *)(&dscr->addr), 8,
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        dscr->addr = le64_to_cpu(dscr->addr);
        dscr->attr &= (uint8_t) ~0xC0;
        fprintf(stdout, "descr->attr: 0x%x\n", dscr->attr);
        dscr->incr = 12;
        break;
    }
}

static void get_cqhci_desc(SDHCIState *s, ADMADescr *desc)
{
    uint8_t task_id = s->argument >> 16;
    desc->addr = cqhci->cmdq[task_id].data_buf_addr[0];
    desc->length = cqhci->cmdq[task_id].block_cnt * 512;
    desc->incr = 0;
    desc->attr = cqhci->cmdq[task_id].attr;
}

/* Advanced DMA data transfer */
static void sdhci_do_adma(SDHCIState *s)
{
    unsigned int n, begin, length;
    const uint16_t block_size = s->blksize & BLOCK_SIZE_MASK;
    ADMADescr dscr = {};
    int i;

    for (i = 0; i < SDHC_ADMA_DESCS_PER_DELAY; ++i) {
        s->admaerr &= ~SDHC_ADMAERR_LENGTH_MISMATCH;
        if (((s->cmdreg >> 8) == 46) || ((s->cmdreg >> 8) == 47))
            get_cqhci_desc(s, &dscr);
        else
            get_adma_description(s, &dscr);

        trace_sdhci_adma_loop(dscr.addr, dscr.length, dscr.attr);

        if ((dscr.attr & SDHC_ADMA_ATTR_VALID) == 0) {
            /* Indicate that error occurred in ST_FDS state */
            s->admaerr &= ~SDHC_ADMAERR_STATE_MASK;
            s->admaerr |= SDHC_ADMAERR_STATE_ST_FDS;

            /* Generate ADMA error interrupt */
            if (s->errintstsen & SDHC_EISEN_ADMAERR) {
                s->errintsts |= SDHC_EIS_ADMAERR;
                s->norintsts |= SDHC_NIS_ERR;
            }

            sdhci_update_irq(s);
            return;
        }

        length = dscr.length ? dscr.length : 64 * KiB;
        fprintf(stdout, "length: %d\tdata_count: %d\n", length, s->data_count);

        switch (dscr.attr & SDHC_ADMA_ATTR_ACT_MASK) {
        case SDHC_ADMA_ATTR_ACT_TRAN:  /* data transfer */

            fprintf(stdout, "%s: %d: case SDHC_ADMA_ATTR_ACT_TRAN: s->trnmod: 0x%x\ts->blksize: %d\n", __func__, __LINE__, s->trnmod, s->blksize);
            fprintf(stdout, "length: %d\ts->data_count: %d\tblock_size: %d\n", length, s->data_count, block_size);
            if (s->trnmod & SDHC_TRNS_READ) {
                while (length) {
                    if (s->data_count == 0) {
                        for (n = 0; n < block_size; n++) {
            //                fprintf(stdout, "loop: %d\t", n);
                            s->fifo_buffer[n] = sdbus_read_data(&s->sdbus);
                        }
                    }
                    begin = s->data_count;
                    if ((length + begin) < block_size) {
                        s->data_count = length + begin;
                        length = 0;
                    } else {
                        s->data_count = block_size;
                        length -= block_size - begin;
                    }
                    fprintf(stdout, "length: %d\n", length);
                    dma_memory_rw_attr(s->dma_as, dscr.addr,
                            &s->fifo_buffer[begin],
                            s->data_count - begin,
                            DMA_DIRECTION_FROM_DEVICE, *s->memattr);
                    dscr.addr += s->data_count - begin;
                    if (s->data_count == block_size) {
                        s->data_count = 0;
                        if (s->trnmod & SDHC_TRNS_BLK_CNT_EN) {
                            s->blkcnt--;
                            if (s->blkcnt == 0) {
                                fprintf(stdout, "s->blkcnt: %d\n", s->blkcnt);
                                break;
                            }
                            fprintf(stdout, "%d\n", __LINE__);
                        }
                        fprintf(stdout, "%d\n", __LINE__);
                    }
                    fprintf(stdout, "%d\n", __LINE__);
                }
                fprintf(stdout, "%d\n", __LINE__);
            } else {
                while (length) {
                    begin = s->data_count;
                    if ((length + begin) < block_size) {
                        s->data_count = length + begin;
                        length = 0;
                    } else {
                        s->data_count = block_size;
                        length -= block_size - begin;
                    }
                    dma_memory_rw_attr(s->dma_as, dscr.addr,
                            &s->fifo_buffer[begin],
                            s->data_count - begin,
                            DMA_DIRECTION_TO_DEVICE, *s->memattr);
                    dscr.addr += s->data_count - begin;
                    if (s->data_count == block_size) {
                        for (n = 0; n < block_size; n++) {
                            sdbus_write_data(&s->sdbus, s->fifo_buffer[n]);
                        }
                        s->data_count = 0;
                        if (s->trnmod & SDHC_TRNS_BLK_CNT_EN) {
                            s->blkcnt--;
                            if (s->blkcnt == 0) {
                                break;
                            }
                        }
                    }
                }
            }
            s->admasysaddr += dscr.incr;
            fprintf(stdout, "%s: SDHCI: s->admasysaddr: 0x%lx\n", __func__, s->admasysaddr);
            break;
        case SDHC_ADMA_ATTR_ACT_LINK:   /* link to next descriptor table */
            s->admasysaddr = dscr.addr;
            trace_sdhci_adma("link", s->admasysaddr);
            break;
        default:
            s->admasysaddr += dscr.incr;
            fprintf(stdout, "%s: SDHCI: s->admasysaddr: 0x%lx\n", __func__, s->admasysaddr);
            break;
        }

        if (dscr.attr & SDHC_ADMA_ATTR_INT) {
            trace_sdhci_adma("interrupt", s->admasysaddr);
            if (s->norintstsen & SDHC_NISEN_DMA) {
                s->norintsts |= SDHC_NIS_DMA;
            }

            sdhci_update_irq(s);
        }

        /* ADMA transfer terminates if blkcnt == 0 or by END attribute */
        if (((s->trnmod & SDHC_TRNS_BLK_CNT_EN) &&
                    (s->blkcnt == 0)) || (dscr.attr & SDHC_ADMA_ATTR_END)) {
            trace_sdhci_adma_transfer_completed();
            if (length || ((dscr.attr & SDHC_ADMA_ATTR_END) &&
                        (s->trnmod & SDHC_TRNS_BLK_CNT_EN) &&
                        s->blkcnt != 0)) {
                trace_sdhci_error("SD/MMC host ADMA length mismatch");
                s->admaerr |= SDHC_ADMAERR_LENGTH_MISMATCH |
                    SDHC_ADMAERR_STATE_ST_TFR;
                if (s->errintstsen & SDHC_EISEN_ADMAERR) {
                    trace_sdhci_error("Set ADMA error flag");
                    s->errintsts |= SDHC_EIS_ADMAERR;
                    s->norintsts |= SDHC_NIS_ERR;
                }
                fprintf(stdout, "Calling 'sdhci_end_transfer'\n");
                sdhci_update_irq(s);
            }
            sdhci_end_transfer(s);
            return;
        }

    }

    /* we have unfinished business - reschedule to continue ADMA */
    timer_mod(s->transfer_timer,
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SDHC_TRANSFER_DELAY);
}

/* Perform data transfer according to controller configuration */

static void sdhci_data_transfer(void *opaque)
{
    SDHCIState *s = (SDHCIState *)opaque;
    fprintf(stdout, "%s: %d: s->hostctll: 0x%x\n", __func__, __LINE__, s->hostctl1);
    if (s->trnmod & SDHC_TRNS_DMA) {
        switch (SDHC_DMA_TYPE(s->hostctl1)) {
        case SDHC_CTRL_SDMA:
            if ((s->blkcnt == 1) || !(s->trnmod & SDHC_TRNS_MULTI)) {
                sdhci_sdma_transfer_single_block(s);
            } else {
                sdhci_sdma_transfer_multi_blocks(s);
            }

            break;
        case SDHC_CTRL_ADMA1_32:
            if (!(s->capareg & R_SDHC_CAPAB_ADMA1_MASK)) {
                trace_sdhci_error("ADMA1 not supported");
                break;
            }

            sdhci_do_adma(s);
            break;
        case SDHC_CTRL_ADMA2_32:
            if (!(s->capareg & R_SDHC_CAPAB_ADMA2_MASK)) {
                trace_sdhci_error("ADMA2 not supported");
                break;
            }

            sdhci_do_adma(s);
            break;
        case SDHC_CTRL_ADMA2_64:
            if (!(s->capareg & R_SDHC_CAPAB_ADMA2_MASK) ||
                    !(s->capareg & R_SDHC_CAPAB_BUS64BIT_MASK)) {
                trace_sdhci_error("64 bit ADMA not supported");
                break;
            }

            sdhci_do_adma(s);
            break;
        default:
            trace_sdhci_error("Unsupported DMA type");
            break;
        }
    } else {
        if ((s->trnmod & SDHC_TRNS_READ) && sdbus_data_ready(&s->sdbus)) {
            s->prnsts |= SDHC_DOING_READ | SDHC_DATA_INHIBIT |
                SDHC_DAT_LINE_ACTIVE;
            sdhci_read_block_from_card(s);
        } else {
            s->prnsts |= SDHC_DOING_WRITE | SDHC_DAT_LINE_ACTIVE |
                SDHC_SPACE_AVAILABLE | SDHC_DATA_INHIBIT;
            sdhci_write_block_to_card(s);
        }
    }
}

static bool sdhci_can_issue_command(SDHCIState *s)
{
    if (!SDHC_CLOCK_IS_ON(s->clkcon) ||
            (((s->prnsts & SDHC_DATA_INHIBIT) || s->stopped_state) &&
             ((s->cmdreg & SDHC_CMD_DATA_PRESENT) ||
              ((s->cmdreg & SDHC_CMD_RESPONSE) == SDHC_CMD_RSP_WITH_BUSY &&
               !(SDHC_COMMAND_TYPE(s->cmdreg) == SDHC_CMD_ABORT))))) {
        return false;
    }

    return true;
}

/* The Buffer Data Port register must be accessed in sequential and
 * continuous manner */
    static inline bool
sdhci_buff_access_is_sequential(SDHCIState *s, unsigned byte_num)
{
    if ((s->data_count & 0x3) != byte_num) {
        trace_sdhci_error("Non-sequential access to Buffer Data Port register"
                "is prohibited\n");
        return false;
    }
    return true;
}

static uint64_t sdhci_read(void *opaque, hwaddr offset, unsigned size)
{
    SDHCIState *s = (SDHCIState *)opaque;
    uint32_t ret = 0;

    switch (offset & ~0x3) {
    case SDHC_SYSAD:
        ret = s->sdmasysad;
        break;
    case SDHC_BLKSIZE:
        ret = s->blksize | (s->blkcnt << 16);
        break;
    case SDHC_ARGUMENT:
        ret = s->argument;
        break;
    case SDHC_TRNMOD:
        ret = s->trnmod | (s->cmdreg << 16);
        break;
    case SDHC_RSPREG0 ... SDHC_RSPREG3:
        ret = s->rspreg[((offset & ~0x3) - SDHC_RSPREG0) >> 2];
        break;
    case  SDHC_BDATA:
        if (sdhci_buff_access_is_sequential(s, offset - SDHC_BDATA)) {
            ret = sdhci_read_dataport(s, size);
            trace_sdhci_access("rd", size << 3, offset, "->", ret, ret);
            return ret;
        }
        break;
    case SDHC_PRNSTS:
        ret = s->prnsts;
        ret = FIELD_DP32(ret, SDHC_PRNSTS, DAT_LVL,
                sdbus_get_dat_lines(&s->sdbus));
        ret = FIELD_DP32(ret, SDHC_PRNSTS, CMD_LVL,
                sdbus_get_cmd_line(&s->sdbus));
        break;
    case SDHC_HOSTCTL:
        ret = s->hostctl1 | (s->pwrcon << 8) | (s->blkgap << 16) |
            (s->wakcon << 24);
        break;
    case SDHC_CLKCON:
        ret = s->clkcon | (s->timeoutcon << 16);
        break;
    case SDHC_NORINTSTS:
        ret = s->norintsts | (s->errintsts << 16);
        break;
    case SDHC_NORINTSTSEN:
        ret = s->norintstsen | (s->errintstsen << 16);
        break;
    case SDHC_NORINTSIGEN:
        ret = s->norintsigen | (s->errintsigen << 16);
        break;
    case SDHC_ACMD12ERRSTS:
        ret = s->acmd12errsts | (s->hostctl2 << 16);
        break;
    case SDHC_CAPAB:
        ret = (uint32_t)s->capareg;
        break;
    case SDHC_CAPAB + 4:
        ret = (uint32_t)(s->capareg >> 32);
        break;
    case SDHC_MAXCURR:
        ret = (uint32_t)s->maxcurr;
        break;
    case SDHC_MAXCURR + 4:
        ret = (uint32_t)(s->maxcurr >> 32);
        break;
    case SDHC_ADMAERR:
        ret =  s->admaerr;
        break;
    case SDHC_ADMASYSADDR:
        ret = (uint32_t)s->admasysaddr;
        break;
    case SDHC_ADMASYSADDR + 4:
        ret = (uint32_t)(s->admasysaddr >> 32);
        break;
    case SDHC_SLOT_INT_STATUS:
        ret = (s->version << 16) | sdhci_slotint(s);
        break;

    case CQHCI_VER:
        ret = (uint32_t)s->cq_version;
        break;
    case CQHCI_CAP:
        ret = (uint32_t)s->cq_cap;
        fprintf(stdout, "CQE_CAP: 0x%x\n", s->cq_cap);
        break;
    case CQHCI_CFG:
        ret = (uint32_t)s->cq_config;
        break;
    case CQHCI_CTL:
        ret = (uint32_t)s->cq_ctrl;
        break;
    case CQHCI_IS:
        ret = (uint32_t)s->cq_int_status;
        break;
    case CQHCI_ISTE:
        ret = (uint32_t)s->cq_int_status_en;
        break;
    case CQHCI_ISGE:
        ret = (uint32_t)s->cq_int_sig_en;
        break;
    case CQHCI_IC:
        ret = (uint32_t)s->cq_int_coalescing;
        break;
    case CQHCI_TDLBA:
        ret = (uint32_t)s->cq_tdlba;
        break;
    case CQHCI_TDLBAU:
        ret = (uint32_t)s->cq_tdlbau;
        break;
    case CQHCI_TDBR:
        ret = (uint32_t)s->cq_task_doorbell;
        break;
    case CQHCI_TCN:
        ret = (uint32_t)s->cq_task_comp_notif;
        break;
    case CQHCI_DQS:
        ret = (uint32_t)s->cq_dev_queue_status;
        break;
    case CQHCI_DPT:
        ret = (uint32_t)s->cq_dev_pending_tasks;
        break;
    case CQHCI_TCLR:
        ret = (uint32_t)s->cq_task_clear;
        break;
    case CQHCI_SSC1:
        fprintf(stdout, "SSC1: Configured to read QSR\n");
        ret = (uint32_t)s->cq_send_status_config1;
        break;
    case CQHCI_SSC2:
        fprintf(stdout, "SSC2: Configured to read QSR\n");
        ret = (uint32_t)s->cq_send_status_config2;
        break;
    case CQHCI_CRDCT:
        ret = (uint32_t)s->cq_cmd_resp_dcmd_task;
        break;
    case CQHCI_RMEM:
        ret = (uint32_t)s->cq_resp_mode_err_mask;
        break;
    case CQHCI_TERRI:
        ret = (uint32_t)s->cq_task_err_info;
        break;
    case CQHCI_CRI:
        ret = (uint32_t)s->cq_cmd_resp_index;
        break;
    case CQHCI_CRA:
        ret = (uint32_t)s->cq_cmd_resp_arg;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "SDHC rd_%ub @0x%02" HWADDR_PRIx " "
                "not implemented\n", size, offset);
        break;
    }

    ret >>= (offset & 0x3) * 8;
    ret &= (1ULL << (size * 8)) - 1;
    trace_sdhci_access("rd", size << 3, offset, "->", ret, ret);
    return ret;
}

static inline void sdhci_blkgap_write(SDHCIState *s, uint8_t value)
{
    if ((value & SDHC_STOP_AT_GAP_REQ) && (s->blkgap & SDHC_STOP_AT_GAP_REQ)) {
        return;
    }
    s->blkgap = value & SDHC_STOP_AT_GAP_REQ;

    if ((value & SDHC_CONTINUE_REQ) && s->stopped_state &&
            (s->blkgap & SDHC_STOP_AT_GAP_REQ) == 0) {
        if (s->stopped_state == sdhc_gap_read) {
            s->prnsts |= SDHC_DAT_LINE_ACTIVE | SDHC_DOING_READ;
            sdhci_read_block_from_card(s);
        } else {
            s->prnsts |= SDHC_DAT_LINE_ACTIVE | SDHC_DOING_WRITE;
            sdhci_write_block_to_card(s);
        }
        s->stopped_state = sdhc_not_stopped;
    } else if (!s->stopped_state && (value & SDHC_STOP_AT_GAP_REQ)) {
        if (s->prnsts & SDHC_DOING_READ) {
            s->stopped_state = sdhc_gap_read;
        } else if (s->prnsts & SDHC_DOING_WRITE) {
            s->stopped_state = sdhc_gap_write;
        }
    }
}

static inline void sdhci_reset_write(SDHCIState *s, uint8_t value)
{
    switch (value) {
    case SDHC_RESET_ALL:
        sdhci_reset(s);
        break;
    case SDHC_RESET_CMD:
        s->prnsts &= ~SDHC_CMD_INHIBIT;
        s->norintsts &= ~SDHC_NIS_CMDCMP;
        break;
    case SDHC_RESET_DATA:
        s->data_count = 0;
        s->prnsts &= ~(SDHC_SPACE_AVAILABLE | SDHC_DATA_AVAILABLE |
                SDHC_DOING_READ | SDHC_DOING_WRITE |
                SDHC_DATA_INHIBIT | SDHC_DAT_LINE_ACTIVE);
        s->blkgap &= ~(SDHC_STOP_AT_GAP_REQ | SDHC_CONTINUE_REQ);
        s->stopped_state = sdhc_not_stopped;
        s->norintsts &= ~(SDHC_NIS_WBUFRDY | SDHC_NIS_RBUFRDY |
                SDHC_NIS_DMA | SDHC_NIS_TRSCMP | SDHC_NIS_BLKGAP);
        break;
    }
}

static inline void process_cqtdlba_descriptor(SDHCIState *s, struct CQHCIState *cqhci)
{
    /* Processing the Task Descriptor */
    hwaddr entry_addr;
    fprintf(stdout, "Updating Doorbell register: 0x%x\n", s->cq_task_doorbell);
    uint8_t task_id = get_task_id(s->cq_task_doorbell);
    fprintf(stdout, "Found TaskID#: %d\n", task_id);

    uint8_t *desc_dma_base = (uint8_t*)(((uint64_t)(s->cq_tdlbau << 16) << 16) | (uint32_t)(s->cq_tdlba));
    fprintf(stdout, "%s: %d: desc_dma_base: %p\n", __func__, __LINE__, desc_dma_base);

    cqhci->trans_desc_len = 16;
    cqhci->task_slot_size = 32;
    cqhci->num_slots = 32;
    cqhci->max_seg = 128;
    /* Formula for Calculating the Task address
     * ADDR = TDLBA + task_id * (Sizeof(TaskDesc) + Sizeof(TransDesc)
     * ADDR = task_desc_dma + task_id * (16bytes + 16bytes))
     * ADDR = task_desc_dma + task_id * 32
     */

    //  for (int i=0; i<32; i++){
    entry_addr = (hwaddr)(desc_dma_base + task_id * cqhci->task_slot_size);
    //  entry_addr = (hwaddr)(desc_dma_base + i * task_slot_size);

    fprintf(stdout, "desc_dma_base: %p\tentry_addr: 0x%lx\n", desc_dma_base, entry_addr);
    uint64_t *task_desc_dma = malloc(32);
    /* Mapping Task and Transfer descriptor */
    dma_memory_rw_attr(s->dma_as, entry_addr, (uint8_t*)task_desc_dma, 32,
            DMA_DIRECTION_TO_DEVICE, *s->memattr);
    *task_desc_dma = le64_to_cpu(*task_desc_dma);
    cqhci->task_desc = (struct task_descriptor *)(task_desc_dma);
#ifdef DEBUG_LOG
    fprintf(stdout, "task_desc_dma: %p\t*task_desc_dma: 0x%lx\n", task_desc_dma, *task_desc_dma);
    fprintf(stdout, "task_desc->valid: 0x%x\n", cqhci->task_desc->valid);
    fprintf(stdout, "task_desc->end: 0x%x\n", cqhci->task_desc->end);
    fprintf(stdout, "task_desc->interrupt: 0x%x\n", cqhci->task_desc->interrupt);
    fprintf(stdout, "task_desc->act: 0x%x\n", cqhci->task_desc->act);
    fprintf(stdout, "task_desc->data_dir: 0x%x\n", cqhci->task_desc->data_dir);
    fprintf(stdout, "task_desc->block_cnt: %d\n", cqhci->task_desc->block_cnt);
    fprintf(stdout, "task_desc->block_addr: 0x%x\n", cqhci->task_desc->block_addr);
#endif

    cqhci->trans_desc = (struct trans_descriptor *)(task_desc_dma + 2);
#ifdef DEBUG_LOG
    fprintf(stdout, "task_desc_dma+2: %p\t*(task_desc_dma+2): 0x%lx\n", task_desc_dma+2, *(task_desc_dma+2));
    fprintf(stdout, "trans_desc->valid: 0x%x\n",cqhci->trans_desc->valid);
    fprintf(stdout, "trans_desc->end: 0x%x\n", cqhci->trans_desc->end);
    fprintf(stdout, "trans_desc->interrupt: 0x%x\n", cqhci->trans_desc->interrupt);
    fprintf(stdout, "trans_desc->act: 0x%x\n", cqhci->trans_desc->act);
    fprintf(stdout, "trans_desc->length: 0x%x\n", cqhci->trans_desc->length);
    fprintf(stdout, "trans_desc->address: 0x%lx\n\n", cqhci->trans_desc->address);
#endif
    //  }
    /* If found link descriptor */
    if (!cqhci->trans_desc->end && cqhci->trans_desc->act == 0x6) {
        uint32_t size_of_sglist = cqhci->max_seg * cqhci->trans_desc_len;
        uint64_t *link_desc_dma = malloc(size_of_sglist);
        uint64_t *temp_link_desc = NULL;
        uint8_t sg_count = 0;

        /* Mapping Link Descriptor */
        dma_memory_rw_attr(s->dma_as, cqhci->trans_desc->address, (uint8_t*)link_desc_dma, size_of_sglist,
                DMA_DIRECTION_TO_DEVICE, *s->memattr);
        *link_desc_dma = le64_to_cpu(*link_desc_dma);

        cqhci->trans_desc = (struct trans_descriptor *)(link_desc_dma);
        while(cqhci->trans_desc->valid) {
#ifdef DEBUG_LOG
            fprintf(stdout, "link_desc_dma: %p\t*link_desc_dma: 0x%lx\n", link_desc_dma, *(link_desc_dma));
            fprintf(stdout, "trans_desc->valid: 0x%x\n",cqhci->trans_desc->valid);
            fprintf(stdout, "trans_desc->end: 0x%x\n", cqhci->trans_desc->end);
            fprintf(stdout, "trans_desc->interrupt: 0x%x\n", cqhci->trans_desc->interrupt);
            fprintf(stdout, "trans_desc->act: 0x%x\n", cqhci->trans_desc->act);
            fprintf(stdout, "trans_desc->length: 0x%x\n", cqhci->trans_desc->length);
            fprintf(stdout, "trans_desc->address: 0x%lx\n\n", cqhci->trans_desc->address);
#endif
            temp_link_desc = (uint64_t *)cqhci->trans_desc;
            if (cqhci->trans_desc->address)
                cqhci->cmdq[task_id].data_buf_addr[sg_count++] = cqhci->trans_desc->address;
            cqhci->trans_desc = (struct trans_descriptor *)(link_desc_dma+2);
        }
        cqhci->trans_desc = (struct trans_descriptor *)temp_link_desc;
#ifdef DEBUG_LOG
        fprintf(stdout, "trans_desc->valid: 0x%x\n",cqhci->trans_desc->valid);
        fprintf(stdout, "trans_desc->end: 0x%x\n", cqhci->trans_desc->end);
        fprintf(stdout, "trans_desc->interrupt: 0x%x\n", cqhci->trans_desc->interrupt);
        fprintf(stdout, "trans_desc->act: 0x%x\n", cqhci->trans_desc->act);
        fprintf(stdout, "trans_desc->length: 0x%x\n", cqhci->trans_desc->length);
        fprintf(stdout, "trans_desc->address: 0x%lx\n\n", cqhci->trans_desc->address);
#endif
        cqhci->cmdq[task_id].task_id = task_id;
        cqhci->cmdq[task_id].reliable_write_req = cqhci->task_desc->reliable_write;
        cqhci->cmdq[task_id].read_write = cqhci->task_desc->data_dir;
        cqhci->cmdq[task_id].priority = cqhci->task_desc->priority;
        cqhci->cmdq[task_id].force_prog = cqhci->task_desc->force_prog;
        cqhci->cmdq[task_id].block_cnt = cqhci->task_desc->block_cnt;
        cqhci->cmdq[task_id].block_addr = cqhci->task_desc->block_addr;
        cqhci->cmdq[task_id].attr = cqhci->trans_desc->valid |
                                    cqhci->trans_desc->end << 1 |
                                    cqhci->trans_desc->interrupt << 2 |
                                    cqhci->trans_desc->act << 3;

    }
}
void add_task_to_queue(struct SDHCIState *s, struct CQHCIState *cqhci)
{
    SDRequest request;
    uint8_t response[16], rlen;
    uint8_t task_pos = get_task_id(s->cq_task_doorbell);

    /* Preparing and Issuing the CMD44 */
    request.cmd = 44;
    request.arg = (cqhci->task_desc->block_cnt | task_pos << 16 | 
            cqhci->task_desc->priority << 23 | 
            cqhci->task_desc->force_prog << 24 |
            cqhci->task_desc->data_dir << 30 |
            cqhci->task_desc->reliable_write << 31);

    trace_sdhci_send_command(request.cmd, request.arg);
    rlen = sdbus_do_command(&s->sdbus, &request, (uint8_t*)&response);
    fprintf(stdout, "CMD44: request.arg: 0x%lx\trlen: %d\n", request.arg, rlen);
    /* Preparing and Issuing the CMD45 */
    request.cmd = 45;
    request.arg = cqhci->task_desc->block_addr;
    trace_sdhci_send_command(request.cmd, request.arg);
    rlen = sdbus_do_command(&s->sdbus, &request, (uint8_t*)&response);
    fprintf(stdout, "CMD45: request.arg: 0x%lx\tblock_addr: %d\trlen: %d\n", request.arg, cqhci->task_desc->block_addr, rlen);

}

#if 1
static void delay(float time)
{
    float rettime = clock();
    fprintf(stdout, "rettime: %.12f\ttime: %.12f\n", rettime, time);
    rettime += time;
    while(clock() < rettime);
}
#endif
static void cqhci_get_qsr(struct SDHCIState *s, struct CQHCIState *cqhci)
{
    SDRequest request;
    uint32_t response[4], rlen;
    fprintf(stdout, "s->cq_cap: 0x%x\ts->cq_cap: 0x%x\ts->cq_cap: %x\n", s->cq_cap, (s->cq_cap & 0x3FF), ((s->cq_cap >> 12) & 0xF));
    float clock_freq = ((s->cq_cap & 0x3FF) * (cqhci_caps[(s->cq_cap >> 12) & 0xF])); 
    float pollingperiod, timeperiod = (float)(1 / clock_freq);

    pollingperiod = (float)((s->cq_send_status_config1 & 0xFFFF) * timeperiod);
    fprintf(stdout, "clock_freq: %.12f\ttimeperiod: %.12f\tpollingperiod: %.12f\n", clock_freq, timeperiod, pollingperiod);

    request.cmd = 13;
    request.arg = ((s->cq_send_status_config2 << 16) | (1 << 15));

    trace_sdhci_send_command(request.cmd, request.arg);
    delay(pollingperiod * SCALE_MS);

    rlen = sdbus_do_command(&s->sdbus, &request, (uint8_t*)&response);
    fprintf(stdout, "CMD13: request.arg: 0x%lx\trlen: %d\tStatus: %x\n", request.arg, rlen, response[0]);
    s->cq_dev_queue_status |= response[0];
}

static void clear_task_from_queue(SDHCIState *s, uint32_t tasks)
{
    /* Prepare and Send the CMD48 to discard the task */
    SDRequest request;
    uint32_t response[4], rlen;

    request.cmd = 48;
    request.arg = (0x2 | get_task_id(tasks) << 16);

    trace_sdhci_send_command(request.cmd, request.arg);
    rlen = sdbus_do_command(&s->sdbus, &request, (uint8_t*)&response);
    fprintf(stdout, "CMD48: request.arg: 0x%lx\trlen: %d\tStatus: %x\n", request.arg, rlen, response[0]);

}

static void
sdhci_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    SDHCIState *s = (SDHCIState *)opaque;
//    struct CQHCIState *cqhci;
    variable = s;      // Workaround fix for Error "Card stuck at wrong state
    unsigned shift =  8 * (offset & 0x3);
    uint32_t mask = ~(((1ULL << (size * 8)) - 1) << shift);
    uint32_t value = val;
    value <<= shift;

    switch (offset & ~0x3) {
    case SDHC_SYSAD:
        s->sdmasysad = (s->sdmasysad & mask) | value;
        MASKED_WRITE(s->sdmasysad, mask, value);
        /* Writing to last byte of sdmasysad might trigger transfer */
        if (!(mask & 0xFF000000) && TRANSFERRING_DATA(s->prnsts) && s->blkcnt &&
                s->blksize && SDHC_DMA_TYPE(s->hostctl1) == SDHC_CTRL_SDMA) {
            if (s->trnmod & SDHC_TRNS_MULTI) {
                sdhci_sdma_transfer_multi_blocks(s);
            } else {
                sdhci_sdma_transfer_single_block(s);
            }
        }
        break;
    case SDHC_BLKSIZE:
        if (!TRANSFERRING_DATA(s->prnsts)) {
            MASKED_WRITE(s->blksize, mask, value);
            MASKED_WRITE(s->blkcnt, mask >> 16, value >> 16);
        }

        /* Limit block size to the maximum buffer size */
        if (extract32(s->blksize, 0, 12) > s->buf_maxsz) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Size 0x%x is larger than "
                    "the maximum buffer 0x%x", __func__, s->blksize,
                    s->buf_maxsz);

            s->blksize = deposit32(s->blksize, 0, 12, s->buf_maxsz);
        }

        break;
    case SDHC_ARGUMENT:
        MASKED_WRITE(s->argument, mask, value);
        break;
    case SDHC_TRNMOD:
        /* DMA can be enabled only if it is supported as indicated by
         * capabilities register */
        if (!(s->capareg & R_SDHC_CAPAB_SDMA_MASK)) {
            value &= ~SDHC_TRNS_DMA;
        }
        MASKED_WRITE(s->trnmod, mask, value & SDHC_TRNMOD_MASK);
        MASKED_WRITE(s->cmdreg, mask >> 16, value >> 16);

        /* Writing to the upper byte of CMDREG triggers SD command generation */
        if ((mask & 0xFF000000) || !sdhci_can_issue_command(s)) {
            break;
        }

        sdhci_send_command(s);
        break;
    case  SDHC_BDATA:
        if (sdhci_buff_access_is_sequential(s, offset - SDHC_BDATA)) {
            sdhci_write_dataport(s, value >> shift, size);
        }
        break;
    case SDHC_HOSTCTL:
        if (!(mask & 0xFF0000)) {
            sdhci_blkgap_write(s, value >> 16);
        }
        MASKED_WRITE(s->hostctl1, mask, value);
        MASKED_WRITE(s->pwrcon, mask >> 8, value >> 8);
        MASKED_WRITE(s->wakcon, mask >> 24, value >> 24);
        if (!(s->prnsts & SDHC_CARD_PRESENT) || ((s->pwrcon >> 1) & 0x7) < 5 ||
                !(s->capareg & (1 << (31 - ((s->pwrcon >> 1) & 0x7))))) {
            s->pwrcon &= ~SDHC_POWER_ON;
        }
        break;
    case SDHC_CLKCON:
        if (!(mask & 0xFF000000)) {
            sdhci_reset_write(s, value >> 24);
        }
        MASKED_WRITE(s->clkcon, mask, value);
        MASKED_WRITE(s->timeoutcon, mask >> 16, value >> 16);
        if (s->clkcon & SDHC_CLOCK_INT_EN) {
            s->clkcon |= SDHC_CLOCK_INT_STABLE;
        } else {
            s->clkcon &= ~SDHC_CLOCK_INT_STABLE;
        }
        break;
    case SDHC_NORINTSTS:
        if (s->norintstsen & SDHC_NISEN_CARDINT) {
            value &= ~SDHC_NIS_CARDINT;
        }
        s->norintsts &= mask | ~value;
        s->errintsts &= (mask >> 16) | ~(value >> 16);
        if (s->errintsts) {
            s->norintsts |= SDHC_NIS_ERR;
        } else {
            s->norintsts &= ~SDHC_NIS_ERR;
        }
        sdhci_update_irq(s);
        break;
    case SDHC_NORINTSTSEN:
        MASKED_WRITE(s->norintstsen, mask, value);
        MASKED_WRITE(s->errintstsen, mask >> 16, value >> 16);
        s->norintsts &= s->norintstsen;
        s->errintsts &= s->errintstsen;
        if (s->errintsts) {
            s->norintsts |= SDHC_NIS_ERR;
        } else {
            s->norintsts &= ~SDHC_NIS_ERR;
        }
        /* Quirk for Raspberry Pi: pending card insert interrupt
         * appears when first enabled after power on */
        if ((s->norintstsen & SDHC_NISEN_INSERT) && s->pending_insert_state) {
            assert(s->pending_insert_quirk);
            s->norintsts |= SDHC_NIS_INSERT;
            s->pending_insert_state = false;
        }
        sdhci_update_irq(s);
        break;
    case SDHC_NORINTSIGEN:
        MASKED_WRITE(s->norintsigen, mask, value);
        MASKED_WRITE(s->errintsigen, mask >> 16, value >> 16);
        sdhci_update_irq(s);
        break;
    case SDHC_ADMAERR:
        MASKED_WRITE(s->admaerr, mask, value);
        break;
    case SDHC_ADMASYSADDR:
        s->admasysaddr = (s->admasysaddr & (0xFFFFFFFF00000000ULL |
                    (uint64_t)mask)) | (uint64_t)value;
        break;
    case SDHC_ADMASYSADDR + 4:
        s->admasysaddr = (s->admasysaddr & (0x00000000FFFFFFFFULL |
                    ((uint64_t)mask << 32))) | ((uint64_t)value << 32);
        break;
    case SDHC_FEAER:
        s->acmd12errsts |= value;
        s->errintsts |= (value >> 16) & s->errintstsen;
        if (s->acmd12errsts) {
            s->errintsts |= SDHC_EIS_CMD12ERR;
        }
        if (s->errintsts) {
            s->norintsts |= SDHC_NIS_ERR;
        }
        sdhci_update_irq(s);
        break;
    case SDHC_ACMD12ERRSTS:
        MASKED_WRITE(s->acmd12errsts, mask, value & UINT16_MAX);
        if (s->uhs_mode >= UHS_I) {
            MASKED_WRITE(s->hostctl2, mask >> 16, value >> 16);

            if (FIELD_EX32(s->hostctl2, SDHC_HOSTCTL2, V18_ENA)) {
                sdbus_set_voltage(&s->sdbus, SD_VOLTAGE_1_8V);
            } else {
                sdbus_set_voltage(&s->sdbus, SD_VOLTAGE_3_3V);
            }
        }
        break;

    case SDHC_CAPAB:
    case SDHC_CAPAB + 4:
    case SDHC_MAXCURR:
    case SDHC_MAXCURR + 4:
        qemu_log_mask(LOG_GUEST_ERROR, "SDHC wr_%ub @0x%02" HWADDR_PRIx
                " <- 0x%08x read-only\n", size, offset, value >> shift);
        break;

        /* Adding support for Command Queue Host Controller */  
    case CQHCI_VER:
        s->cq_version |= value;
        fprintf(stdout, "CQE_VERSION: 0x%x\n", s->cq_version);
        break;
    case CQHCI_CAP:
        /* Encoded for 19.2MHz hence ITCFMUL set to 02h */
        s->cq_cap = ((ITCFVAL) | (0x2 << 12));
        fprintf(stdout, "CQE_CAP: 0x%x\n", s->cq_cap);
        break;
    case CQHCI_CFG:
        s->cq_config |= value;
        fprintf(stdout, "CQE_CONFIG: 0x%x\n", s->cq_config);
        break;
    case CQHCI_CTL:
        s->cq_ctrl |= value;
        fprintf(stdout, "CQE_CTL: 0x%x\n", s->cq_ctrl);
        break;
    case CQHCI_IS:
        s->cq_int_status |= value;
        break;
    case CQHCI_ISTE:
        s->cq_int_status_en |= value;
        break;
    case CQHCI_ISGE:
        s->cq_int_sig_en |= value;
        break;
    case CQHCI_IC:
        s->cq_int_coalescing |= value;
        break;
    case CQHCI_TDLBA:
        s->cq_tdlba |= value;
        break;
    case CQHCI_TDLBAU:
        s->cq_tdlbau |= value;
        break;
    case CQHCI_TDBR:
        /* Performing Task Issuance Sequence */
        s->cq_task_doorbell |= value;

        process_cqtdlba_descriptor(s, cqhci);
        if (cqhci->task_desc->interrupt)
            s->cq_int_status_en |= (cqhci->task_desc->interrupt << 1);

        add_task_to_queue(s, cqhci);
        cqhci_get_qsr(s, cqhci);
        /* Requesting to perform Task Completion Sequence */
        sdhci_write(opaque, CQHCI_DPT, s->cq_dev_queue_status, size);
        break;
    case CQHCI_TCN:
        s->cq_task_comp_notif &= ~(value);
        break;
    case CQHCI_DQS:
        s->cq_dev_queue_status |= value;
        break;
    case CQHCI_DPT:
        /* Performing Task Completion Sequence */
        s->cq_dev_pending_tasks |= value;
        uint8_t pending_task_id, temp_pending_tasks;
        temp_pending_tasks = s->cq_dev_pending_tasks;
        while (temp_pending_tasks){
            fprintf(stdout, "Device_Pending Tasks: 0x%x\n", temp_pending_tasks);
            pending_task_id = get_task_id(temp_pending_tasks);

            fprintf(stdout, "Issue CMD46/CMD47 for Queued Task#: %d\n", pending_task_id);
            fprintf(stdout, "block_addr: 0x%lx\tdirection: %d\ts->cmdreg: 0x%x\n", 
                    cqhci->cmdq[pending_task_id].block_addr, cqhci->cmdq[pending_task_id].read_write, s->cmdreg);
            if (cqhci->cmdq[pending_task_id].block_cnt)
            {
                if (cqhci->cmdq[pending_task_id].read_write){
                    s->cmdreg &= 0xFFFF00FF;
                    s->cmdreg |= (46 << 8);
                    s->trnmod |= SDHC_TRNS_READ;
                }else{
                    s->cmdreg &= 0xFFFF00FF;
                    s->cmdreg |= (47 << 8);
                }
                s->argument = pending_task_id << 16;
                s->blkcnt = cqhci->cmdq[pending_task_id].block_cnt;
            }
            s->cmdreg |= SDHC_CMD_DATA_PRESENT;
            fprintf(stdout, "s->cmdreg: 0x%x\tcmd: %d\n",s->cmdreg, (s->cmdreg>>8) & 0xFF);
            sdhci_send_command(s);
            /* Set task complete interrupt bit if this bit in status enable reg is set */
            if (s->cq_int_status_en == 0x2)
                s->cq_int_status |= (1 << 1);

            s->cq_task_comp_notif |= 1 << pending_task_id;
            fprintf(stdout, "Device_Pending Tasks: 0x%x\n", s->cq_dev_pending_tasks);
            s->cq_dev_pending_tasks &= ~(1 << pending_task_id);
            fprintf(stdout, "Cleared task from Device_Pending Tasks: 0x%x\n", s->cq_dev_pending_tasks);
            temp_pending_tasks >>= pending_task_id;
        }

        break;
    case CQHCI_TCLR:
        /* Clearing the Task from the list only if it's in Halt state*/
        if (s->cq_ctrl == 0x1 && value)
        {
            s->cq_task_clear |= value;
            clear_task_from_queue(s, value);
            s->cq_task_clear &= ~value;
            s->cq_task_doorbell &= ~value;
        }
        break;
    case CQHCI_SSC1:
        /* Configuring default reset values */
        s->cq_send_status_config1 = 0x11000;
        break;
    case CQHCI_SSC2:
        s->cq_send_status_config2 = 1;
        break;
    case CQHCI_CRDCT:
        s->cq_cmd_resp_dcmd_task |= value;
        break;
    case CQHCI_RMEM:
        s->cq_resp_mode_err_mask |= value;
        break;
    case CQHCI_TERRI:
        s->cq_task_err_info |= value;
        break;
    case CQHCI_CRI:
        s->cq_cmd_resp_index |= value;
        break;
    case CQHCI_CRA:
        s->cq_cmd_resp_arg |= value;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "SDHC wr_%ub @0x%02" HWADDR_PRIx " <- 0x%08x "
                "not implemented\n", size, offset, value >> shift);
        break;
    }
    trace_sdhci_access("wr", size << 3, offset, "<-",
            value >> shift, value >> shift);
}

static const MemoryRegionOps sdhci_mmio_ops = {
    .read = sdhci_read,
    .write = sdhci_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = false
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void sdhci_init_readonly_registers(SDHCIState *s, Error **errp)
{
    ERRP_GUARD();

    switch (s->sd_spec_version) {
    case 2 ... 3:
        break;
    default:
        error_setg(errp, "Only Spec v2/v3 are supported");
        return;
    }
    s->version = (SDHC_HCVER_VENDOR << 8) | (s->sd_spec_version - 1);
    s->cq_cap = (0x2 << 12) | (ITCFVAL);
    s->cq_version = 0x1;

    fprintf(stderr, "%s, %s, %d: s->version:0x%08x \tspec_verson:%08x \n", __FILE__, __func__, __LINE__, s->version, s->sd_spec_version);
    fprintf(stderr, "%s, %s, %d: capareg:%lu \n", __FILE__, __func__, __LINE__, s->capareg);
    sdhci_check_capareg(s, errp);
    if (*errp) {
        return;
    }
}

static void cqhci_init_registers(SDHCIState *s)
{
    s->cq_version = 0x1;
    s->cq_cap = (0x2 << 12) | (ITCFVAL);
    s->cq_send_status_config1 = 0x11000;
    s->cq_send_status_config2 = 0x1;
    cqhci = malloc(sizeof(struct CQHCIState));
}

/* --- qdev common --- */

void sdhci_initfn(SDHCIState *s)
{
    qbus_create_inplace(&s->sdbus, sizeof(s->sdbus),
            TYPE_SDHCI_BUS, DEVICE(s), "sd-bus");

    s->insert_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sdhci_raise_insertion_irq, s);
    s->transfer_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sdhci_data_transfer, s);

    s->io_ops = &sdhci_mmio_ops;

    object_property_add_link(OBJECT(s), "memattr", TYPE_MEMORY_TRANSACTION_ATTR,
            (Object **)&s->memattr,
            qdev_prop_allow_set_link_before_realize,
            OBJ_PROP_LINK_STRONG);

    fprintf(stderr, "%s: %s: %d \n", __FILE__, __func__, __LINE__);
}

void sdhci_uninitfn(SDHCIState *s)
{
    timer_del(s->insert_timer);
    timer_free(s->insert_timer);
    timer_del(s->transfer_timer);
    timer_free(s->transfer_timer);

    g_free(s->fifo_buffer);
    s->fifo_buffer = NULL;
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
}

void sdhci_common_realize(SDHCIState *s, Error **errp)
{
    ERRP_GUARD();

    sdhci_init_readonly_registers(s, errp);
    if (*errp) {
        return;
    }
    cqhci_init_registers(s);
    s->buf_maxsz = sdhci_get_fifolen(s);
    s->fifo_buffer = g_malloc0(s->buf_maxsz);

    memory_region_init_io(&s->iomem, OBJECT(s), s->io_ops, s, "sdhci",
            SDHC_REGISTERS_MAP_SIZE);
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
}

void sdhci_common_unrealize(SDHCIState *s)
{
    /* This function is expected to be called only once for each class:
     * - SysBus:    via DeviceClass->unrealize(),
     * - PCI:       via PCIDeviceClass->exit().
     * However to avoid double-free and/or use-after-free we still nullify
     * this variable (better safe than sorry!). */
    g_free(s->fifo_buffer);
    s->fifo_buffer = NULL;
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
}

static bool sdhci_pending_insert_vmstate_needed(void *opaque)
{
    SDHCIState *s = opaque;

    return s->pending_insert_state;
}

static const VMStateDescription sdhci_pending_insert_vmstate = {
    .name = "sdhci/pending-insert",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = sdhci_pending_insert_vmstate_needed,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(pending_insert_state, SDHCIState),
        VMSTATE_END_OF_LIST()
    },
};

const VMStateDescription sdhci_vmstate = {
    .name = "sdhci",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(sdmasysad, SDHCIState),
        VMSTATE_UINT16(blksize, SDHCIState),
        VMSTATE_UINT16(blkcnt, SDHCIState),
        VMSTATE_UINT32(argument, SDHCIState),
        VMSTATE_UINT16(trnmod, SDHCIState),
        VMSTATE_UINT16(cmdreg, SDHCIState),
        VMSTATE_UINT32_ARRAY(rspreg, SDHCIState, 4),
        VMSTATE_UINT32(prnsts, SDHCIState),
        VMSTATE_UINT8(hostctl1, SDHCIState),
        VMSTATE_UINT8(pwrcon, SDHCIState),
        VMSTATE_UINT8(blkgap, SDHCIState),
        VMSTATE_UINT8(wakcon, SDHCIState),
        VMSTATE_UINT16(clkcon, SDHCIState),
        VMSTATE_UINT8(timeoutcon, SDHCIState),
        VMSTATE_UINT8(admaerr, SDHCIState),
        VMSTATE_UINT16(norintsts, SDHCIState),
        VMSTATE_UINT16(errintsts, SDHCIState),
        VMSTATE_UINT16(norintstsen, SDHCIState),
        VMSTATE_UINT16(errintstsen, SDHCIState),
        VMSTATE_UINT16(norintsigen, SDHCIState),
        VMSTATE_UINT16(errintsigen, SDHCIState),
        VMSTATE_UINT16(acmd12errsts, SDHCIState),
        VMSTATE_UINT16(data_count, SDHCIState),
        VMSTATE_UINT64(admasysaddr, SDHCIState),
        VMSTATE_UINT8(stopped_state, SDHCIState),
        VMSTATE_VBUFFER_UINT32(fifo_buffer, SDHCIState, 1, NULL, buf_maxsz),
        VMSTATE_TIMER_PTR(insert_timer, SDHCIState),
        VMSTATE_TIMER_PTR(transfer_timer, SDHCIState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {
        &sdhci_pending_insert_vmstate,
        NULL
    },
};

void sdhci_common_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->vmsd = &sdhci_vmstate;
    dc->reset = sdhci_poweron_reset;
    fprintf(stderr, "%s, %s, %d\n", __FILE__, __func__, __LINE__);
}

/* --- qdev SysBus --- */

static Property sdhci_sysbus_properties[] = {
    DEFINE_SDHCI_COMMON_PROPERTIES(SDHCIState),
    DEFINE_PROP_BOOL("pending-insert-quirk", SDHCIState, pending_insert_quirk,
            false),
    DEFINE_PROP_LINK("dma", SDHCIState,
            dma_mr, TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST(),
};

static void sdhci_sysbus_init(Object *obj)
{
    SDHCIState *s = SYSBUS_SDHCI(obj);

    sdhci_initfn(s);
    fprintf(stderr, "%s: %s: %d \n", __FILE__, __func__, __LINE__);
}

static void sdhci_sysbus_finalize(Object *obj)
{
    SDHCIState *s = SYSBUS_SDHCI(obj);

    if (s->dma_mr) {
        object_unparent(OBJECT(s->dma_mr));
    }

    sdhci_uninitfn(s);
}

static void sdhci_sysbus_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    SDHCIState *s = SYSBUS_SDHCI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sdhci_common_realize(s, errp);
    if (*errp) {
        return;
    }

    if (s->dma_mr) {
        s->dma_as = &s->sysbus_dma_as;
        address_space_init(s->dma_as, s->dma_mr, "sdhci-dma");
    } else {
        /* use system_memory() if property "dma" not set */
        s->dma_as = &address_space_memory;
    }

    if (!s->memattr) {
        s->memattr = g_malloc0(sizeof(MemTxAttrs));
        *s->memattr =  MEMTXATTRS_UNSPECIFIED;
    }

    sysbus_init_irq(sbd, &s->irq);

    sysbus_init_mmio(sbd, &s->iomem);
    fprintf(stderr, "%s: %s: %d \n", __FILE__, __func__, __LINE__);
}

static void sdhci_sysbus_unrealize(DeviceState *dev)
{
    SDHCIState *s = SYSBUS_SDHCI(dev);

    sdhci_common_unrealize(s);

    if (s->dma_mr) {
        address_space_destroy(s->dma_as);
    }
}

static void sdhci_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, sdhci_sysbus_properties);
    dc->realize = sdhci_sysbus_realize;
    dc->unrealize = sdhci_sysbus_unrealize;

    sdhci_common_class_init(klass, data);
}

static const TypeInfo sdhci_sysbus_info = {
    .name = TYPE_SYSBUS_SDHCI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SDHCIState),
    .instance_init = sdhci_sysbus_init,
    .instance_finalize = sdhci_sysbus_finalize,
    .class_init = sdhci_sysbus_class_init,
};

/* --- qdev bus master --- */

static void sdhci_bus_class_init(ObjectClass *klass, void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_inserted = sdhci_set_inserted;
    sbc->set_readonly = sdhci_set_readonly;
}

static const TypeInfo sdhci_bus_info = {
    .name = TYPE_SDHCI_BUS,
    .parent = TYPE_SD_BUS,
    .instance_size = sizeof(SDBus),
    .class_init = sdhci_bus_class_init,
};

/* --- qdev i.MX eSDHC --- */

static uint64_t usdhc_read(void *opaque, hwaddr offset, unsigned size)
{
    SDHCIState *s = SYSBUS_SDHCI(opaque);
    uint32_t ret;
    uint16_t hostctl1;

    switch (offset) {
    default:
        return sdhci_read(opaque, offset, size);

    case SDHC_HOSTCTL:
        /*
         * For a detailed explanation on the following bit
         * manipulation code see comments in a similar part of
         * usdhc_write()
         */
        hostctl1 = SDHC_DMA_TYPE(s->hostctl1) << (8 - 3);

        if (s->hostctl1 & SDHC_CTRL_8BITBUS) {
            hostctl1 |= ESDHC_CTRL_8BITBUS;
        }

        if (s->hostctl1 & SDHC_CTRL_4BITBUS) {
            hostctl1 |= ESDHC_CTRL_4BITBUS;
        }

        ret  = hostctl1;
        ret |= (uint32_t)s->blkgap << 16;
        ret |= (uint32_t)s->wakcon << 24;

        break;

    case SDHC_PRNSTS:
        /* Add SDSTB (SD Clock Stable) bit to PRNSTS */
        ret = sdhci_read(opaque, offset, size) & ~ESDHC_PRNSTS_SDSTB;
        if (s->clkcon & SDHC_CLOCK_INT_STABLE) {
            ret |= ESDHC_PRNSTS_SDSTB;
        }
        break;

    case ESDHC_VENDOR_SPEC:
        ret = s->vendor_spec;
        break;
    case ESDHC_DLL_CTRL:
    case ESDHC_TUNE_CTRL_STATUS:
    case ESDHC_UNDOCUMENTED_REG27:
    case ESDHC_TUNING_CTRL:
    case ESDHC_MIX_CTRL:
    case ESDHC_WTMK_LVL:
        ret = 0;
        break;
    }

    return ret;
}

    static void
usdhc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    SDHCIState *s = SYSBUS_SDHCI(opaque);
    uint8_t hostctl1;
    uint32_t value = (uint32_t)val;

    switch (offset) {
    case ESDHC_DLL_CTRL:
    case ESDHC_TUNE_CTRL_STATUS:
    case ESDHC_UNDOCUMENTED_REG27:
    case ESDHC_TUNING_CTRL:
    case ESDHC_WTMK_LVL:
        break;

    case ESDHC_VENDOR_SPEC:
        s->vendor_spec = value;
        switch (s->vendor) {
            case SDHCI_VENDOR_IMX:
                if (value & ESDHC_IMX_FRC_SDCLK_ON) {
                    s->prnsts &= ~SDHC_IMX_CLOCK_GATE_OFF;
                } else {
                    s->prnsts |= SDHC_IMX_CLOCK_GATE_OFF;
                }
                break;
            default:
                break;
        }
        break;

    case SDHC_HOSTCTL:
        /*
         * Here's What ESDHCI has at offset 0x28 (SDHC_HOSTCTL)
         *
         *       7         6     5      4      3      2        1      0
         * |-----------+--------+--------+-----------+----------+---------|
         * | Card      | Card   | Endian | DATA3     | Data     | Led     |
         * | Detect    | Detect | Mode   | as Card   | Transfer | Control |
         * | Signal    | Test   |        | Detection | Width    |         |
         * | Selection | Level  |        | Pin       |          |         |
         * |-----------+--------+--------+-----------+----------+---------|
         *
         * and 0x29
         *
         *  15      10 9    8
         * |----------+------|
         * | Reserved | DMA  |
         * |          | Sel. |
         * |          |      |
         * |----------+------|
         *
         * and here's what SDCHI spec expects those offsets to be:
         *
         * 0x28 (Host Control Register)
         *
         *     7        6         5       4  3      2         1        0
         * |--------+--------+----------+------+--------+----------+---------|
         * | Card   | Card   | Extended | DMA  | High   | Data     | LED     |
         * | Detect | Detect | Data     | Sel. | Speed  | Transfer | Control |
         * | Signal | Test   | Transfer |      | Enable | Width    |         |
         * | Sel.   | Level  | Width    |      |        |          |         |
         * |--------+--------+----------+------+--------+----------+---------|
         *
         * and 0x29 (Power Control Register)
         *
         * |----------------------------------|
         * | Power Control Register           |
         * |                                  |
         * | Description omitted,             |
         * | since it has no analog in ESDHCI |
         * |                                  |
         * |----------------------------------|
         *
         * Since offsets 0x2A and 0x2B should be compatible between
         * both IP specs we only need to reconcile least 16-bit of the
         * word we've been given.
         */

        /*
         * First, save bits 7 6 and 0 since they are identical
         */
        hostctl1 = value & (SDHC_CTRL_LED |
                SDHC_CTRL_CDTEST_INS |
                SDHC_CTRL_CDTEST_EN);
        /*
         * Second, split "Data Transfer Width" from bits 2 and 1 in to
         * bits 5 and 1
         */
        if (value & ESDHC_CTRL_8BITBUS) {
            hostctl1 |= SDHC_CTRL_8BITBUS;
        }

        if (value & ESDHC_CTRL_4BITBUS) {
            hostctl1 |= ESDHC_CTRL_4BITBUS;
        }

        /*
         * Third, move DMA select from bits 9 and 8 to bits 4 and 3
         */
        hostctl1 |= SDHC_DMA_TYPE(value >> (8 - 3));

        /*
         * Now place the corrected value into low 16-bit of the value
         * we are going to give standard SDHCI write function
         *
         * NOTE: This transformation should be the inverse of what can
         * be found in drivers/mmc/host/sdhci-esdhc-imx.c in Linux
         * kernel
         */
        value &= ~UINT16_MAX;
        value |= hostctl1;
        value |= (uint16_t)s->pwrcon << 8;

        sdhci_write(opaque, offset, value, size);
        break;

    case ESDHC_MIX_CTRL:
        /*
         * So, when SD/MMC stack in Linux tries to write to "Transfer
         * Mode Register", ESDHC i.MX quirk code will translate it
         * into a write to ESDHC_MIX_CTRL, so we do the opposite in
         * order to get where we started
         *
         * Note that Auto CMD23 Enable bit is located in a wrong place
         * on i.MX, but since it is not used by QEMU we do not care.
         *
         * We don't want to call sdhci_write(.., SDHC_TRNMOD, ...)
         * here becuase it will result in a call to
         * sdhci_send_command(s) which we don't want.
         *
         */
        s->trnmod = value & UINT16_MAX;
        break;
    case SDHC_TRNMOD:
        /*
         * Similar to above, but this time a write to "Command
         * Register" will be translated into a 4-byte write to
         * "Transfer Mode register" where lower 16-bit of value would
         * be set to zero. So what we do is fill those bits with
         * cached value from s->trnmod and let the SDHCI
         * infrastructure handle the rest
         */
        sdhci_write(opaque, offset, val | s->trnmod, size);
        break;
    case SDHC_BLKSIZE:
        /*
         * ESDHCI does not implement "Host SDMA Buffer Boundary", and
         * Linux driver will try to zero this field out which will
         * break the rest of SDHCI emulation.
         *
         * Linux defaults to maximum possible setting (512K boundary)
         * and it seems to be the only option that i.MX IP implements,
         * so we artificially set it to that value.
         */
        val |= 0x7 << 12;
        /* FALLTHROUGH */
    default:
        sdhci_write(opaque, offset, val, size);
        break;
    }
}

static const MemoryRegionOps usdhc_mmio_ops = {
    .read = usdhc_read,
    .write = usdhc_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = false
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void imx_usdhc_init(Object *obj)
{
    SDHCIState *s = SYSBUS_SDHCI(obj);

    s->io_ops = &usdhc_mmio_ops;
    s->quirks = SDHCI_QUIRK_NO_BUSY_IRQ;
}

static const TypeInfo imx_usdhc_info = {
    .name = TYPE_IMX_USDHC,
    .parent = TYPE_SYSBUS_SDHCI,
    .instance_init = imx_usdhc_init,
};

/* --- qdev Samsung s3c --- */

#define S3C_SDHCI_CONTROL2      0x80
#define S3C_SDHCI_CONTROL3      0x84
#define S3C_SDHCI_CONTROL4      0x8c

static uint64_t sdhci_s3c_read(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t ret;

    switch (offset) {
    case S3C_SDHCI_CONTROL2:
    case S3C_SDHCI_CONTROL3:
    case S3C_SDHCI_CONTROL4:
        /* ignore */
        ret = 0;
        break;
    default:
        ret = sdhci_read(opaque, offset, size);
        break;
    }

    return ret;
}

static void sdhci_s3c_write(void *opaque, hwaddr offset, uint64_t val,
        unsigned size)
{
    switch (offset) {
    case S3C_SDHCI_CONTROL2:
    case S3C_SDHCI_CONTROL3:
    case S3C_SDHCI_CONTROL4:
        /* ignore */
        break;
    default:
        sdhci_write(opaque, offset, val, size);
        break;
    }
}

static const MemoryRegionOps sdhci_s3c_mmio_ops = {
    .read = sdhci_s3c_read,
    .write = sdhci_s3c_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = false
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void sdhci_s3c_init(Object *obj)
{
    SDHCIState *s = SYSBUS_SDHCI(obj);

    s->io_ops = &sdhci_s3c_mmio_ops;
}

static const TypeInfo sdhci_s3c_info = {
    .name = TYPE_S3C_SDHCI  ,
    .parent = TYPE_SYSBUS_SDHCI,
    .instance_init = sdhci_s3c_init,
};

static void sdhci_register_types(void)
{
    type_register_static(&sdhci_sysbus_info);
    type_register_static(&sdhci_bus_info);
    type_register_static(&imx_usdhc_info);
    type_register_static(&sdhci_s3c_info);
}

type_init(sdhci_register_types)
