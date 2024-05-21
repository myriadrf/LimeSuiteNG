#ifndef LIME_LMS7002M_H
#define LIME_LMS7002M_H

#include "limesuiteng/embedded/result.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_context;

enum lms7002m_vco_type { LMS7002M_VCO_CGEN, LMS7002M_VCO_SXR, LMS7002M_VCO_SXT };
enum lms7002m_path_rfe {
    LMS7002M_PATH_RFE_NONE,
    LMS7002M_PATH_RFE_LNAH,
    LMS7002M_PATH_RFE_LNAL,
    LMS7002M_PATH_RFE_LNAW,
    LMS7002M_PATH_RFE_LB1,
    LMS7002M_PATH_RFE_LB2
};

enum lms7002m_channel {
    LMS7002M_CHANNEL_A = 1U,
    LMS7002M_CHANNEL_B = 2U,
    LMS7002M_CHANNEL_AB = 3U,
    LMS7002M_CHANNEL_SXR = 1U,
    LMS7002M_CHANNEL_SXT = 2U
};

typedef int (*lms7002m_spi16_transact_hook)(const uint32_t* mosi, uint32_t* miso, uint32_t count, void* userData);
typedef void (*lms7002m_log_hook)(int level, const char* message, void* userData);
typedef void (*lms7002m_on_cgen_frequency_changed_hook)(void* userData);

typedef struct lms7002m_hooks {
    lms7002m_log_hook log;
    void* log_userData;

    lms7002m_spi16_transact_hook spi16_transact;
    void* spi16_userData;

    lms7002m_on_cgen_frequency_changed_hook on_cgen_frequency_changed;
    void* on_cgen_frequency_changed_userData;
} lms7002m_hooks;

struct lms7002m_context* lms7002m_create(const lms7002m_hooks* hooks);
void lms7002m_destroy(struct lms7002m_context* context);

double lms7002m_get_reference_clock(struct lms7002m_context* context);
lime_Result lms7002m_set_reference_clock(struct lms7002m_context* context, double frequency_Hz);

lime_Result lms7002m_enable_channel(struct lms7002m_context* self, const bool isTx, const uint8_t channel, const bool enable);

uint8_t lms7002m_get_active_channel(struct lms7002m_context* self);
lime_Result lms7002m_set_active_channel(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_soft_reset(struct lms7002m_context* self);
lime_Result lms7002m_reset_logic_registers(struct lms7002m_context* self);

lime_Result lms7002m_tune_cgen_vco(struct lms7002m_context* self);

lime_Result lms7002m_set_frequency_cgen(struct lms7002m_context* context, double frequency_Hz);
double lms7002m_get_frequency_cgen(struct lms7002m_context* self);

lime_Result lms7002m_set_rbbpga_db(struct lms7002m_context* self, const double value, const uint8_t channel);
double lms7002m_get_rbbpga_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_rfelna_db(struct lms7002m_context* self, const double value, const uint8_t channel);
double lms7002m_get_rfelna_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_rfe_loopback_lna_db(struct lms7002m_context* self, const double gain, const uint8_t channel);
double lms7002m_get_rfe_loopback_lna_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_rfetia_db(struct lms7002m_context* self, const double value, const uint8_t channel);
double lms7002m_get_rfetia_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_trfpad_db(struct lms7002m_context* self, const double value, const uint8_t channel);
double lms7002m_get_trfpad_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_trf_loopback_pad_db(struct lms7002m_context* self, const double gain, const uint8_t channel);
double lms7002m_get_trf_loopback_pad_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_path_rfe(struct lms7002m_context* self, const uint8_t path);
uint8_t lms7002m_get_path_rfe(struct lms7002m_context* self);

lime_Result lms7002m_set_band_trf(struct lms7002m_context* self, const uint8_t band);
uint8_t lms7002m_get_band_trf(struct lms7002m_context* self);

lime_Result lms7002m_set_path(struct lms7002m_context* self, bool isTx, uint8_t channel, uint8_t path);

double lms7002m_get_reference_clock_tsp(struct lms7002m_context* self, bool isTx);
bool lms7002m_get_cgen_locked(struct lms7002m_context* self);
bool lms7002m_get_sx_locked(struct lms7002m_context* self, bool isTx);

lime_Result lms7002m_tune_vco(struct lms7002m_context* self, enum lms7002m_vco_type module);

double lms7002m_get_frequency_sx(struct lms7002m_context* self, bool isTx);

lime_Result lms7002m_set_nco_frequency(struct lms7002m_context* self, bool isTx, const uint8_t index, double freq_Hz);
double lms7002m_get_nco_frequency(struct lms7002m_context* self, bool isTx, const uint8_t index);

lime_Result lms7002m_set_nco_phase_offset(struct lms7002m_context* self, bool isTx, uint8_t index, double angle_deg);
lime_Result lms7002m_set_nco_phase_offset_for_mode_0(struct lms7002m_context* self, bool isTx, double angle_deg);
lime_Result lms7002m_set_nco_phases(
    struct lms7002m_context* self, bool isTx, const double* const angles_deg, uint8_t count, double frequencyOffset);

lime_Result lms7002m_set_nco_frequencies(
    struct lms7002m_context* self, bool isTx, const double* const freq_Hz, uint8_t count, double phaseOffset);
lime_Result lms7002m_get_nco_frequencies(
    struct lms7002m_context* self, bool isTx, double* const freq_Hz, uint8_t count, double* phaseOffset);

lime_Result lms7002m_set_gfir_coefficients(
    struct lms7002m_context* self, bool isTx, uint8_t gfirIndex, const double* const coef, uint8_t coefCount);
lime_Result lms7002m_get_gfir_coefficients(
    struct lms7002m_context* self, bool isTx, uint8_t gfirIndex, double* const coef, uint8_t coefCount);

lime_Result lms7002m_set_interface_frequency(
    struct lms7002m_context* self, double cgen_freq_Hz, const uint8_t hbi, const uint8_t hbd);

lime_Result lms7002m_enable_sxtdd(struct lms7002m_context* self, bool tdd);

lime_Result lms7002m_set_dc_offset(struct lms7002m_context* self, bool isTx, const double I, const double Q);
lime_Result lms7002m_get_dc_offset(struct lms7002m_context* self, bool isTx, double* const I, double* const Q);

lime_Result lms7002m_set_i_q_balance(
    struct lms7002m_context* self, bool isTx, const double phase, const double gainI, const double gainQ);
lime_Result lms7002m_get_i_q_balance(
    struct lms7002m_context* self, bool isTx, double* const phase, double* const gainI, double* const gainQ);

lime_Result lms7002m_calibrate_internal_adc(struct lms7002m_context* self, int clkDiv);
lime_Result lms7002m_calibrate_rp_bias(struct lms7002m_context* self);

#ifdef __cplusplus
}
#endif

#endif // LIME_LMS7002M_H
