/******************************************************************************* ******************************************************************************
MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any derivatives created by
any person or entity by or on your behalf, exclusively with Microchip?s products.  Microchip and its licensors
 retain all ownership and intellectual property rights in the accompanying software and in all derivatives hereto.
This software and any accompanying information is for suggestion only.  It does not modify Microchip?s standard warranty for its products.
You agree that you are solely responsible for testing the software and determining its suitability.  Microchip has no obligation to modify, test, certify,
or support the software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,
BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN
ANY APPLICATION.

IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY),
STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS,
DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON
ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP
FOR THIS SOFTWARE.
MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.

*************************************************************************************************************************************************************/
#include <xc.h>
#include <libpic30.h>

#include "p33Fxxxx.h"
#include "define.h"
#include "dsp.h"

volatile unsigned int timerInterruptCount;

int16_t e_n, e_n1, e_n2, e_n3;
int32_t y_n1, y_n2;
int32_t y_n;

int32_t n1;
int32_t n2;
int32_t n3;
int16_t d2;
int16_t d3;

/* --- ADC inputs --- */
u16  adc_an0;    /* AN0 raw sample (Vout path A)   */
u16  adc_an2;    /* AN2 raw sample (Vout path B)   */
u16  adc_an0_prev;    /* AN0 raw sample (Vout path A)   */
u16  adc_an2_prev;    /* AN2 raw sample (Vout path B)   */
s16  vfb_sum2ch;      /* (AN0 + AN2) * 2                */
s16  voutSetpoint;          /* output voltage set-point       */

/* --- Frequency control word (after clamping) --- */
s16  u_exec;        /* clamped output -> PTPER calc   */

/* --- Gain scheduling (Kff) --- */
s16  kff_vout;      /* Vout-proportional Kff base     */
s16  kff_gain;      /* dynamic gain factor G(n)
                     * 1024 = steady state  (0x400)
                     * 1500 = transient boost (0x5DC) x1.465      */

/* --- Computed PWM values (written to registers each cycle) --- */
s16  ptper;         /* switching period register      */
s16  pdc1;     /* PTPER - 8  (fine adjustment)   */
s16  pdc2;     /* PTPER - 8  (fine adjustment)   */
s16  pdc3;          /* SR duty cycle                  */
s16  dtr;           /* dead-time  (symmetric)         */


extern  int16_t adc_4pt_sum;       // DAT_ram_1d9e, 4-point moving average
extern void updateCurrentMeasurementPipeline(void);
extern void updateCurrentOcpFastAvg8Pt(void);
extern void ocpVrefFoldbackUpdate(void);
extern void llc_voltage_cal_ovp(void);
extern void ocpShutdownCheck(void);
extern void tempFanHandler(void);
extern void PWMStart(void);
extern void pwm_force_off(void);
extern int spi_flash_save_config(void);
extern int spi_flash_verify_firmware_boot(void);
extern int spi_flash_verify_firmware_slot(uint16_t slot);
extern int spi_flash_save_firmware_metadata(uint16_t slot, uint16_t expected_crc);
extern int spi_flash_save_current_firmware_crc(uint16_t slot);
extern uint16_t spi_flash_config_status;
extern uint16_t spi_flash_fw_status;
extern uint16_t spi_flash_fw_actual_crc;
extern uint16_t spi_flash_fw_expected_crc;
extern uint16_t spi_flash_fw_active_slot;
extern uint16_t spi_flash_fw_checked_page;
extern int16_t vout_cal;
extern int16_t Imeas_scaled;
extern s16 voutSetpoint;
extern volatile uint16_t systemState;
extern volatile uint16_t protectionStatus;
extern volatile uint16_t controlStatus;
extern volatile uint16_t pwmRunRequest;
extern volatile uint16_t pwmRunning;
extern volatile uint16_t pmbusAlertFlags;
extern volatile uint16_t adcLiveA;
extern volatile uint16_t tempAdcValue;
extern uint16_t ioutAdcRaw;
extern uint16_t ocp_latch_threshold;
extern uint16_t ocp_hard_threshold;
extern uint16_t ocp_latch_delay;
extern uint16_t ocp_foldback_delay;
extern uint16_t ovp_threshold_normal;
extern uint16_t ovp_threshold_mode4;
extern uint16_t ovp_freq_ctrl_min;

/* Small PMBus subset used by src: telemetry, on/off, margins,
 * runtime protection limits, fan temperature input and flash save. */
#define PMBUS_OPERATION          0x01u
#define PMBUS_CLEAR_FAULTS       0x03u
#define PMBUS_VOUT_COMMAND      0x21u
#define PMBUS_VOUT_MARGIN_HIGH  0x25u
#define PMBUS_VOUT_MARGIN_LOW   0x26u
#define PMBUS_STATUS_BYTE       0x78u
#define PMBUS_STATUS_WORD       0x79u
#define PMBUS_READ_VOUT         0x8Bu
#define PMBUS_READ_IOUT         0x8Cu
#define PMBUS_READ_TEMPERATURE  0x8Du
#define PMBUS_MFR_SAVE_CONFIG   0xD0u
#define PMBUS_MFR_FAN_TEMP      0xD1u
#define PMBUS_MFR_OCP_LATCH     0xD2u
#define PMBUS_MFR_OCP_HARD      0xD3u
#define PMBUS_MFR_OCP_DELAY     0xD4u
#define PMBUS_MFR_OCP_FOLDBACK  0xD5u
#define PMBUS_MFR_OVP_NORMAL    0xD6u
#define PMBUS_MFR_OVP_MODE4     0xD7u
#define PMBUS_MFR_OVP_FREQ_MIN  0xD8u
#define PMBUS_MFR_IOUT_LIMIT    0xD9u
#define PMBUS_MFR_FW_SLOT       0xDAu
#define PMBUS_MFR_FW_STATUS     0xDBu
#define PMBUS_MFR_FW_ACTUAL_CRC 0xDCu
#define PMBUS_MFR_FW_EXPECT_CRC 0xDDu
#define PMBUS_MFR_FW_PAGE       0xDEu
#define PMBUS_MFR_FW_ACTION     0xDFu

static uint8_t pmbus_cmd;
static uint8_t pmbus_rx_count;
static uint8_t pmbus_rx_buf[2];
static uint8_t pmbus_tx_buf[2];
static uint8_t pmbus_tx_len;
static uint8_t pmbus_tx_idx;
static uint16_t pmbus_margin_high;
static uint16_t pmbus_margin_low;
static uint8_t pmbus_operation;
static uint16_t pmbus_fw_slot;

static void pmbus_put_word(uint16_t value)
{
        pmbus_tx_buf[0] = (uint8_t)value;
        pmbus_tx_buf[1] = (uint8_t)(value >> 8);
        pmbus_tx_len = 2;
        pmbus_tx_idx = 0;
}

static void pmbus_put_byte(uint8_t value)
{
        pmbus_tx_buf[0] = value;
        pmbus_tx_len = 1;
        pmbus_tx_idx = 0;
}

static uint16_t pmbus_status_word(void)
{
        uint16_t word = controlStatus;

        if (protectionStatus != 0)
                word |= 0x0008u;
        if (pmbusAlertFlags & 0x0001u)
                word |= 0x0080u;
        if (systemState == 3u)
                word |= 0x0400u;

        return word;
}

static void pmbus_remote_on(void)
{
        pmbus_operation = 0x80u;
        systemState = 2;
        pwmRunRequest |= (1u << 0);
        pwmRunning |= (1u << 0);
        PWMStart();
}

static void pmbus_remote_off(void)
{
        pmbus_operation = 0;
        pwmRunRequest &= (uint16_t)~(1u << 0);
        pwmRunning &= (uint16_t)~(1u << 0);
        systemState = 0;
        pwm_force_off();
}

static void pmbus_apply_operation(uint8_t value)
{
        if (value == 0xA8u) {
                if (pmbus_margin_high != 0)
                        voutSetpoint = (s16)pmbus_margin_high;
                pmbus_remote_on();
        } else if (value == 0x98u) {
                if (pmbus_margin_low != 0)
                        voutSetpoint = (s16)pmbus_margin_low;
                pmbus_remote_on();
        } else if (value & 0x80u) {
                pmbus_remote_on();
        } else {
                pmbus_remote_off();
        }
}

static void pmbus_write_word(uint8_t cmd, uint16_t value)
{
        switch (cmd) {
            case PMBUS_VOUT_COMMAND:     voutSetpoint = (s16)value; break;
            case PMBUS_VOUT_MARGIN_HIGH: pmbus_margin_high = value; break;
            case PMBUS_VOUT_MARGIN_LOW:  pmbus_margin_low = value; break;
            case PMBUS_MFR_FAN_TEMP:
                tempAdcValue = value;
                adcLiveA = value << 6;
                break;
            case PMBUS_MFR_OCP_LATCH:    ocp_latch_threshold = value; break;
            case PMBUS_MFR_OCP_HARD:     ocp_hard_threshold = value; break;
            case PMBUS_MFR_OCP_DELAY:    ocp_latch_delay = value; break;
            case PMBUS_MFR_OCP_FOLDBACK: ocp_foldback_delay = value; break;
            case PMBUS_MFR_OVP_NORMAL:   ovp_threshold_normal = value; break;
            case PMBUS_MFR_OVP_MODE4:    ovp_threshold_mode4 = value; break;
            case PMBUS_MFR_OVP_FREQ_MIN: ovp_freq_ctrl_min = value; break;
            case PMBUS_MFR_IOUT_LIMIT:   ioutAdcRaw = value; break;
            case PMBUS_MFR_FW_SLOT:      pmbus_fw_slot = value & 1u; break;
            case PMBUS_MFR_FW_EXPECT_CRC:
                spi_flash_fw_expected_crc = value;
                break;
            default: break;
        }
}

static void pmbus_write_byte(uint8_t cmd, uint8_t value)
{
        if (cmd == PMBUS_OPERATION) {
                pmbus_apply_operation(value);
        } else if ((cmd == PMBUS_MFR_SAVE_CONFIG) && (value == 0xA5u)) {
                (void)spi_flash_save_config();
        } else if (cmd == PMBUS_MFR_FW_ACTION) {
                /* Firmware image actions: 0xA5 verify slot, 0x5A boot fallback,
                 * 0x3C save supplied CRC, 0xC3 learn CRC from selected slot. */
                if (value == 0xA5u) {
                        (void)spi_flash_verify_firmware_slot(pmbus_fw_slot);
                } else if (value == 0x5Au) {
                        (void)spi_flash_verify_firmware_boot();
                } else if (value == 0x3Cu) {
                        (void)spi_flash_save_firmware_metadata(pmbus_fw_slot,
                                                               spi_flash_fw_expected_crc);
                } else if (value == 0xC3u) {
                        (void)spi_flash_save_current_firmware_crc(pmbus_fw_slot);
                }
        }
}

static void pmbus_prepare_response(void)
{
        switch (pmbus_cmd) {
            case PMBUS_OPERATION:         pmbus_put_byte(pmbus_operation); break;
            case PMBUS_STATUS_BYTE:       pmbus_put_byte((uint8_t)pmbus_status_word()); break;
            case PMBUS_STATUS_WORD:       pmbus_put_word(pmbus_status_word()); break;
            case PMBUS_VOUT_COMMAND:      pmbus_put_word((uint16_t)voutSetpoint); break;
            case PMBUS_VOUT_MARGIN_HIGH:  pmbus_put_word(pmbus_margin_high); break;
            case PMBUS_VOUT_MARGIN_LOW:   pmbus_put_word(pmbus_margin_low); break;
            case PMBUS_READ_VOUT:         pmbus_put_word((uint16_t)vout_cal); break;
            case PMBUS_READ_IOUT:         pmbus_put_word((uint16_t)Imeas_scaled); break;
            case PMBUS_READ_TEMPERATURE:  pmbus_put_word(tempAdcValue); break;
            case PMBUS_MFR_FAN_TEMP:      pmbus_put_word(tempAdcValue); break;
            case PMBUS_MFR_SAVE_CONFIG:   pmbus_put_word(spi_flash_config_status); break;
            case PMBUS_MFR_OCP_LATCH:     pmbus_put_word(ocp_latch_threshold); break;
            case PMBUS_MFR_OCP_HARD:      pmbus_put_word(ocp_hard_threshold); break;
            case PMBUS_MFR_OCP_DELAY:     pmbus_put_word(ocp_latch_delay); break;
            case PMBUS_MFR_OCP_FOLDBACK:  pmbus_put_word(ocp_foldback_delay); break;
            case PMBUS_MFR_OVP_NORMAL:    pmbus_put_word(ovp_threshold_normal); break;
            case PMBUS_MFR_OVP_MODE4:     pmbus_put_word(ovp_threshold_mode4); break;
            case PMBUS_MFR_OVP_FREQ_MIN:  pmbus_put_word(ovp_freq_ctrl_min); break;
            case PMBUS_MFR_IOUT_LIMIT:    pmbus_put_word(ioutAdcRaw); break;
            case PMBUS_MFR_FW_SLOT:        pmbus_put_word(pmbus_fw_slot); break;
            case PMBUS_MFR_FW_STATUS:      pmbus_put_word(spi_flash_fw_status); break;
            case PMBUS_MFR_FW_ACTUAL_CRC:  pmbus_put_word(spi_flash_fw_actual_crc); break;
            case PMBUS_MFR_FW_EXPECT_CRC:  pmbus_put_word(spi_flash_fw_expected_crc); break;
            case PMBUS_MFR_FW_PAGE:        pmbus_put_word(spi_flash_fw_checked_page); break;
            case PMBUS_MFR_FW_ACTION:      pmbus_put_word(spi_flash_fw_active_slot); break;
            default:                      pmbus_put_byte(0xFFu); break;
        }
}

static void pmbus_accept_byte(uint8_t value)
{
        if (pmbus_rx_count == 0) {
                pmbus_cmd = value;
                pmbus_rx_count = 1;
                if (pmbus_cmd == PMBUS_CLEAR_FAULTS) {
                        protectionStatus = 0;
                        controlStatus &= (uint16_t)~0x0001u;
                        pmbusAlertFlags &= (uint16_t)~0x0001u;
                }
                pmbus_prepare_response();
                return;
        }

        if (pmbus_rx_count > 2)
                return;

        pmbus_rx_buf[pmbus_rx_count - 1u] = value;
        pmbus_rx_count++;

        if (pmbus_rx_count == 2)
                pmbus_write_byte(pmbus_cmd, pmbus_rx_buf[0]);

        if (pmbus_rx_count == 3) {
                uint16_t word = (uint16_t)pmbus_rx_buf[0] |
                                ((uint16_t)pmbus_rx_buf[1] << 8);
                pmbus_write_word(pmbus_cmd, word);
                pmbus_prepare_response();
        }
}


static __attribute__((always_inline)) int16_t util_divsd(int32_t dividend, int16_t divisor)
{
        register int32_t result asm("w0") = dividend;

        __asm__ volatile (
            "push w1\n\t"
            "repeat #17\n\t"
            "div.sd %0, %1\n\t"
            "pop w1\n\t"
            : "+r"(result)
            : "r"(divisor)
            : "cc"
        );
        return (int16_t)result;
}


void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt()
{
        // timerInterruptCount ++; 	/* Increment interrupt counter */
        updateCurrentMeasurementPipeline();
        //llc_droop_trim_calc();
        ocpVrefFoldbackUpdate();
        ocpShutdownCheck();
        tempFanHandler();
        IFS0bits.T1IF = 0; 		/* Clear Interrupt Flag */
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt()
{
        // timerInterruptCount ++; 	/* Increment interrupt counter */

        s16 delta;

        s32 tmp;
        s16 kff_vout;
        s16 kff;

        /* Stage 1: u_exec -> kff_vout */
        tmp      = (u_exec * KFF_COEFF) >> 15;
        kff_vout = tmp - KFF_OFFSET;

        if      (kff_vout > (s16)KFF_MAX) kff_vout = (s16)KFF_MAX;
        else if (kff_vout < (s16)KFF_MIN) kff_vout = (s16)KFF_MIN;

        /* Stage 2: apply transient gain factor
         *   steady-state:  kff_gain=1024, result = kff_vout (unity scale)
         *   transient:     kff_gain=1500, result = kff_vout * 1.465       */
        kff_gain = 1024;

        adc_an0 = ADCBUF0;
        adc_an2 = ADCBUF2;
        vfb_sum2ch = adc_an0 * 2 + adc_an2 * 2;
        adc_4pt_sum = adc_an0 + adc_an0_prev + adc_an2 + adc_an2_prev;
        adc_an0_prev = adc_an0;
        adc_an2_prev = adc_an2;

        e_n = voutSetpoint -  vfb_sum2ch;	    /* Find error */

        delta = e_n - e_n1;
        if (delta > 100) {                      /* rate limit up */
                e_n = e_n1 + 50;
        } else if (delta < -100) {             /* rate limit down */
                e_n = e_n1 - 50;
        }

        if (e_n > 100) {
                e_n = 100;
        } else if (e_n < -100) {
                e_n = -100;
        }

        kff = ((s32)kff_vout * (s32)kff_gain) >> 10;

        /* ---- A terms: d2*u(n-1) + d3*u(n-2) ---- */
        int32_t A =  __mulsi3((int32_t)d2, (int32_t)y_n1)
                        + __mulsi3((int32_t)d3, (int32_t)y_n2);

        /* ---- B terms: n1*e(n) + n2*e(n-1) + n3*e(n-2) ---- */
        int32_t B = __mulsi3((int32_t)n1, (int32_t)e_n)
                        + __mulsi3((int32_t)n2, (int32_t)e_n1)
                        + __mulsi3((int32_t)n3, (int32_t)e_n2);

        /* ---- Final: [(B * Kff) >> 7 + A] >> 13 ---- */
        s32 BxKff = __mulsi3(B, (s32)kff) >> 7;
        y_n    = (BxKff + A) >> 13;

        if(y_n > 25000) {
                y_n = 25000;
        } else if(y_n < 2600) {
                y_n = 2600;
        }

        u_exec = (s16)y_n;

        /* Protect divider */
        if (u_exec < U_EXEC_MIN) u_exec = U_EXEC_MIN;
        if (u_exec > U_EXEC_MAX) u_exec = U_EXEC_MAX;


        /* Switching period */
        int16_t period     = ((s16)util_divsd((s32)PWM_CLK_80, (s16)u_exec)) >> 1;
        // int16_t period     = ((s16)__builtin_divsd((s32)PWM_CLK_80, (s16)u_exec)) >> 1;
        ptper = period - 8;
        pdc1 = period;
        pdc2 = period;
        pdc3 = SR_FIXED_PDC3;

        e_n1 = e_n;		/* Update previous voltage error */
        e_n2 = e_n1;
        e_n3 = e_n2;

        /* Upadation of previous Compensator outputs */
        y_n1 = y_n;
        y_n2 = y_n1;

        updateCurrentOcpFastAvg8Pt();
        llc_voltage_cal_ovp();
        
        IFS0bits.T2IF = 0; 		/* Clear Interrupt Flag */
        IFS3bits.PSEMIF         = 0;
        IEC3bits.PSEMIE         = 1;
        PTCONbits.SEIEN         = 1;
}

void __attribute__((__interrupt__, no_auto_psv)) _SI2C2Interrupt()
{
        if (I2C2STATbits.R_W) {
                if (!I2C2STATbits.D_A) {
                        pmbus_tx_idx = 0;
                        pmbus_prepare_response();
                }

                if (pmbus_tx_idx < pmbus_tx_len)
                        I2C2TRN = pmbus_tx_buf[pmbus_tx_idx++];
                else
                        I2C2TRN = 0xFFu;
        } else {
                uint8_t value = (uint8_t)I2C2RCV;

                if (!I2C2STATbits.D_A)
                        pmbus_rx_count = 0;
                else
                        pmbus_accept_byte(value);
        }

        I2C2CONbits.SCLREL = 1;
        IFS3bits.SI2C2IF = 0; 		/* Clear Interrupt Flag */
}

void __attribute__((__interrupt__, no_auto_psv)) _PWMSpEventMatchInterrupt()
{
        // timerInterruptCount ++;    /* Increment interrupt counter */

        PTPER   = ptper;           /* period register (PDC-8)   */
        PDC1    = pdc1;            /* primary high-side: 100% DC   */
        PDC2    = pdc2;            /* primary low-side:  100% DC   */
        PDC3    = pdc3;            /* SR duty cycle               */
        PHASE3  = 0;               /* SR phase offset             */
        DTR3    = 0x2F;            /* SR dead-time rising edge    */
        ALTDTR3 = 0x2F;            /* SR dead-time falling edge   */

        PTCONbits.SEIEN         = 0;   /* 0x0400 bit11 - disable Special Event interrupt enable */
        IEC3bits.PSEMIE         = 0;   /* 0x009A bit10 - disable PSEM interrupt                */
        IFS3bits.PSEMIF         = 0;   /* 0x008A bit10 - clear PSEM interrupt flag             */
}
