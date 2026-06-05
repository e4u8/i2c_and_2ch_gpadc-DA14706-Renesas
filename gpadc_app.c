/**
 ****************************************************************************************
 *
 * @file gpadc_app.c
 *
 * @brief GPADC application logic — DA14706
 *        Interleaved single-sample acquisition (CH0=current, CH1=voltage).
 *        DMA-assisted reads (CPU yields during conversion).
 *
 *        • DWT cycle counter  — sub-microsecond timestamps, no OS overhead.
 *        • Startup skew print — exact CH0→CH1 inter-sample delay.
 *        • *fs_acq / *us_pair — true ADC throughput from DWT timestamps.
 *        • *Vrms / *Irms      — 1-second windowed AC RMS.
 *        • *P / *S / *Q / *PF — active, apparent, reactive power and power factor.
 *        • *Temp / *Hum       — last AHT20 reading (updated every ~2 s).
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include "osal.h"
#include "ad_gpadc.h"
#include "platform_devices.h"
#include "include/gpadc_app.h"

/* ── Shared AHT20 data (written by aht20_task every ~2 s) ───────────────── */
extern volatile float   g_last_temp_c;
extern volatile uint8_t g_last_hum_percent;

/* ── Channel aliases ─────────────────────────────────────────────────────── */
#define CHAN0_DEVICE    ADC_CH0_DEVICE   /* current channel */
#define CHAN1_DEVICE    ADC_CH1_DEVICE   /* voltage channel */

/* ── Acquisition ─────────────────────────────────────────────────────────── */
#define BATCH_SIZE      64

/* ── Calibration ─────────────────────────────────────────────────────────── */
#define OFFSET_MV_CH0   0.0f
#define GAIN_CH0        1.0f
#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f

/* ── RMS scaling ─────────────────────────────────────────────────────────── *
 * K_V  : mains Volts per ADC millivolt   [V/V]  — transformer ratio +       *
 *         signal conditioning attenuation.                                    *
 * K_I  : Hall sensor mV per ADC millivolt [mV/mV] — signal conditioning.    *
 * HALL_SENSITIVITY_MV_PER_A : Hall sensor output sensitivity [mV/A].        *
 * ─────────────────────────────────────────────────────────────────────────  */
#define K_V                       (289.269f)
#define K_I                       (1.298f)
#define HALL_SENSITIVITY_MV_PER_A (80.0f)
#define P_SIGN                    (-1.0f)   /* -1: signal conditioning inverts one channel */

/* ── Output scaling ──────────────────────────────────────────────────────── */
#define V_RMS_SCALE   100   /* centivolts      — 230.45 V  → 23045 */
#define I_RMS_SCALE  1000   /* milliamps       —   1.500 A →  1500 */
#define P_W_SCALE     100   /* centiwatts      — 1234.56 W → 123456 */
#define S_VA_SCALE    100   /* centi volt-amps */
#define Q_VAR_SCALE   100   /* centi volt-amps reactive */
#define PF_SCALE     1000   /* milli power factor — 0.985 → 985 */

/* ── Active power scale factor [W/mV²] ──────────────────────────────────── */
#define P_SCALE  (K_V * K_I / (1000.0f * HALL_SENSITIVITY_MV_PER_A))

/* ── CPU clock ───────────────────────────────────────────────────────────── */
#define CPU_CLOCK_HZ    32000000UL

/* ─────────────────────────────────────────────────────────────────────────── */

void gpadc_app_init(void) {}

static float correct_mv(uint32_t mv_raw, float offset, float gain)
{
        if ((float)mv_raw < offset) return 0.0f;
        return ((float)mv_raw - offset) / gain;
}

typedef struct {
        float rms_mv_v;  /* CH1 voltage AC RMS [mV] */
        float rms_mv_i;  /* CH0 current AC RMS [mV] */
        float p_mvsq;    /* mean( (v-mean_v)*(i-mean_i) ) [mV²] */
} batch_metrics_t;

/*
 * Two-pass batch computation of AC RMS and mean instantaneous power.
 *
 * Pass 1: convert both channels to mV, compute DC offsets (mean_v, mean_i).
 * Pass 2: squared residuals for RMS, cross-product for P.
 *
 * Mean-subtracted residuals avoid the spurious V_dc*I_dc term in P.
 * Stack: two float[BATCH_SIZE] local arrays = 512 bytes.
 */
static batch_metrics_t compute_batch_metrics(
        const uint16_t *raw_v, const uint16_t *raw_i, int n,
        const ad_gpadc_driver_conf_t *drv_v,
        const ad_gpadc_driver_conf_t *drv_i,
        float offset_v, float gain_v,
        float offset_i, float gain_i)
{
        float mv_v[BATCH_SIZE];
        float mv_i[BATCH_SIZE];

        float sum_v = 0.0f, sum_i = 0.0f;
        for (int k = 0; k < n; k++) {
                mv_v[k] = correct_mv(
                        (uint32_t)ad_gpadc_conv_to_mvolt(drv_v, raw_v[k]),
                        offset_v, gain_v);
                mv_i[k] = correct_mv(
                        (uint32_t)ad_gpadc_conv_to_mvolt(drv_i, raw_i[k]),
                        offset_i, gain_i);
                sum_v += mv_v[k];
                sum_i += mv_i[k];
        }
        const float mean_v = sum_v / (float)n;
        const float mean_i = sum_i / (float)n;

        float sum_sq_v = 0.0f, sum_sq_i = 0.0f, sum_p = 0.0f;
        for (int k = 0; k < n; k++) {
                const float rv = mv_v[k] - mean_v;
                const float ri = mv_i[k] - mean_i;
                sum_sq_v += rv * rv;
                sum_sq_i += ri * ri;
                sum_p    += rv * ri;
        }

        const float inv_n = 1.0f / (float)n;
        batch_metrics_t res;
        res.rms_mv_v = sqrtf(sum_sq_v * inv_n);
        res.rms_mv_i = sqrtf(sum_sq_i * inv_n);
        res.p_mvsq   = sum_p * inv_n;
        return res;
}

/* ─────────────────────────────────────────────────────────────────────────── */

void gpadc_app_task(void *pvParameters)
{
        static uint16_t   raw0[BATCH_SIZE];
        static uint16_t   raw1[BATCH_SIZE];
        static uint32_t   acq_cycles_accum  = 0;
        static uint32_t   batches_in_window = 0;
        static TickType_t t_last            = 0;
        static float      rms2_accum_ch0    = 0.0f;
        static float      rms2_accum_ch1    = 0.0f;
        static float      p_accum           = 0.0f;

        /* Enable DWT cycle counter */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT       = 0;
        DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

        /* Warmup dummy read + inter-channel skew measurement */
        {
                uint16_t dummy      = 0;
                uint32_t t_ch0_done = 0;
                uint32_t t_ch1_done = 0;
                bool     both_ok    = false;

                ad_gpadc_handle_t h = ad_gpadc_open(CHAN0_DEVICE);
                if (h) {
                        ad_gpadc_read_nof_conv(h, 1, &dummy);
                        t_ch0_done = DWT->CYCCNT;
                        ad_gpadc_close(h, false);

                        h = ad_gpadc_open(CHAN1_DEVICE);
                        if (h) {
                                ad_gpadc_read_nof_conv(h, 1, &dummy);
                                t_ch1_done = DWT->CYCCNT;
                                ad_gpadc_close(h, false);
                                both_ok = true;
                        }
                }

                if (both_ok) {
                        uint32_t skew_cy = t_ch1_done - t_ch0_done;
                        uint32_t skew_us = skew_cy / (CPU_CLOCK_HZ / 1000000UL);
                        printf("*skew=%u cycles (~%u us)\n",
                               (unsigned)skew_cy, (unsigned)skew_us);
                }
        }

        for (;;) {

                /* ── 1. Interleaved batch acquisition ────────────────────── */
                uint32_t t_acq_start = DWT->CYCCNT;

                for (int i = 0; i < BATCH_SIZE; i++) {
                        ad_gpadc_handle_t h0 = ad_gpadc_open(CHAN0_DEVICE);
                        if (h0) {
                                ad_gpadc_read_nof_conv(h0, 1, &raw0[i]);
                                ad_gpadc_close(h0, false);
                        } else {
                                raw0[i] = 0;
                        }
                        ad_gpadc_handle_t h1 = ad_gpadc_open(CHAN1_DEVICE);
                        if (h1) {
                                ad_gpadc_read_nof_conv(h1, 1, &raw1[i]);
                                ad_gpadc_close(h1, false);
                        } else {
                                raw1[i] = 0;
                        }
                }

                acq_cycles_accum += DWT->CYCCNT - t_acq_start;
                batches_in_window++;

                /* ── 2. Per-batch metrics accumulation ───────────────────── */
                {
                        batch_metrics_t m = compute_batch_metrics(
                                raw1, raw0, BATCH_SIZE,
                                CHAN1_DEVICE->drv, CHAN0_DEVICE->drv,
                                OFFSET_MV_CH1, GAIN_CH1,
                                OFFSET_MV_CH0, GAIN_CH0);
                        rms2_accum_ch1 += m.rms_mv_v * m.rms_mv_v;
                        rms2_accum_ch0 += m.rms_mv_i * m.rms_mv_i;
                        p_accum        += m.p_mvsq;
                }

                /* ── 3. Per-sample CSV print (disabled; enable for plotter) ─ */
#if 0
                for (int i = 0; i < BATCH_SIZE; i++) {
                        int mv0_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(CHAN0_DEVICE->drv, raw0[i]),
                                OFFSET_MV_CH0, GAIN_CH0);
                        int mv1_val = (int)correct_mv(
                                (uint32_t)ad_gpadc_conv_to_mvolt(CHAN1_DEVICE->drv, raw1[i]),
                                OFFSET_MV_CH1, GAIN_CH1);
                        printf("%d,%d\n", mv0_val, mv1_val);
                }
#endif

                /* ── 4. Once-per-second diagnostics ──────────────────────── */
                TickType_t now = xTaskGetTickCount();
                if ((now - t_last) >= pdMS_TO_TICKS(1000)) {
                        if (acq_cycles_accum > 0 && batches_in_window > 0) {
                                uint32_t total_pairs = batches_in_window * (uint32_t)BATCH_SIZE;

                                uint32_t fs_acq = (uint32_t)(
                                        (uint64_t)total_pairs * 2UL * CPU_CLOCK_HZ
                                        / acq_cycles_accum);
                                uint32_t us_pair = acq_cycles_accum
                                        / (total_pairs * (CPU_CLOCK_HZ / 1000000UL));
                                printf("*fs_acq=%u  *us_pair=%u\n",
                                       (unsigned)fs_acq, (unsigned)us_pair);

                                float rms_mv0_w = sqrtf(rms2_accum_ch0 / (float)batches_in_window);
                                float rms_mv1_w = sqrtf(rms2_accum_ch1 / (float)batches_in_window);

                                float v_rms = K_V * rms_mv1_w / 1000.0f;
                                float i_rms = (K_I * rms_mv0_w) / HALL_SENSITIVITY_MV_PER_A;

                                int32_t v_rms_cV = (int32_t)(v_rms * V_RMS_SCALE);
                                int32_t i_rms_mA = (int32_t)(i_rms * I_RMS_SCALE);

                                printf("*Vrms=%"PRId32".%02"PRId32" V  *Irms=%"PRId32".%03"PRId32" A\n",
                                       v_rms_cV / 100,  v_rms_cV % 100,
                                       i_rms_mA / 1000, i_rms_mA % 1000);

                                float p_w   = P_SIGN * P_SCALE * (p_accum / (float)batches_in_window);
                                float s_va  = v_rms * i_rms;
                                float q_var = sqrtf(fabsf(s_va * s_va - p_w * p_w));
                                float pf    = (s_va > 0.0f) ? (p_w / s_va) : 0.0f;

                                int32_t p_cW   = (int32_t)(p_w   * P_W_SCALE);
                                int32_t s_cVA  = (int32_t)(s_va  * S_VA_SCALE);
                                int32_t q_cVAr = (int32_t)(q_var * Q_VAR_SCALE);
                                int32_t pf_m   = (int32_t)(fabsf(pf) * PF_SCALE);

                                printf("*P=%"PRId32".%02"PRId32" W\n",  p_cW   / 100, p_cW   % 100);
                                printf("*S=%"PRId32".%02"PRId32" VA\n", s_cVA  / 100, s_cVA  % 100);
                                printf("*Q=%"PRId32".%02"PRId32" VAr\n",q_cVAr / 100, q_cVAr % 100);
                                printf("*PF=%s%"PRId32".%03"PRId32"\n",
                                       pf < 0.0f ? "-" : "", pf_m / 1000, pf_m % 1000);

                                /* AHT20 snapshot (single volatile read — atomic on CM33) */
                                float   temp_c = g_last_temp_c;
                                uint8_t hum    = g_last_hum_percent;
                                int32_t t_x100 = (int32_t)(temp_c * 100.0f);
                                int32_t t_int  = t_x100 / 100;
                                int32_t t_frac = t_x100 % 100;
                                if (t_frac < 0) t_frac = -t_frac;
                                printf("*Temp=%"PRId32".%02"PRId32" C  *Hum=%u%%\n",
                                       t_int, t_frac, (unsigned)hum);
                        }

                        acq_cycles_accum  = 0;
                        batches_in_window = 0;
                        rms2_accum_ch0    = 0.0f;
                        rms2_accum_ch1    = 0.0f;
                        p_accum           = 0.0f;
                        t_last            = now;
                }

        } /* end for(;;) */
}
