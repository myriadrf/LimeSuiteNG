#ifndef LIME_LMS7002M_H
#define LIME_LMS7002M_H

#include "limesuiteng/embedded/result.h"
#include "limesuiteng/embedded/types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_context;

enum lms7002m_vco_type { LMS7002M_VCO_CGEN, LMS7002M_VCO_SXR, LMS7002M_VCO_SXT };

/// @brief The IDs of the clocks on the chip.
enum lms7002m_clock_id {
    LMS7002M_CLK_REFERENCE = 0, ///< Reference clock
    LMS7002M_CLK_SXR = 1, ///< RX LO clock
    LMS7002M_CLK_SXT = 2, ///< TX LO clock
    LMS7002M_CLK_CGEN = 3, ///< Clock generator clock
    LMS7002M_CLK_RXTSP = 4, ///< RXTSP reference clock (read-only)
    LMS7002M_CLK_TXTSP = 5 ///< TXTSP reference clock (read-only)
};

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
typedef lime_Result (*lms7002m_on_cgen_frequency_changed_hook)(void* userData);

typedef struct lms7002m_hooks {
    lms7002m_log_hook log;
    void* log_userData;

    lms7002m_spi16_transact_hook spi16_transact;
    void* spi16_userData;

    lms7002m_on_cgen_frequency_changed_hook on_cgen_frequency_changed;
    void* on_cgen_frequency_changed_userData;
} lms7002m_hooks;

struct lms7002m_decibel {
    int32_t data;
};

struct lms7002m_context* lms7002m_create(const lms7002m_hooks* hooks);
void lms7002m_destroy(struct lms7002m_context* context);

uint32_t lms7002m_get_reference_clock(struct lms7002m_context* context);
lime_Result lms7002m_set_reference_clock(struct lms7002m_context* context, uint32_t frequency_Hz);

lime_Result lms7002m_enable_channel(
    struct lms7002m_context* self, const bool isTx, const enum lms7002m_channel channel, const bool enable);

enum lms7002m_channel lms7002m_get_active_channel(struct lms7002m_context* self);
lime_Result lms7002m_set_active_channel(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_soft_reset(struct lms7002m_context* self);
lime_Result lms7002m_reset_logic_registers(struct lms7002m_context* self);

lime_Result lms7002m_tune_cgen_vco(struct lms7002m_context* self);

lime_Result lms7002m_set_frequency_cgen(struct lms7002m_context* context, uint32_t frequency_Hz);
uint32_t lms7002m_get_frequency_cgen(struct lms7002m_context* self);

lime_Result lms7002m_set_rbbpga_db(
    struct lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel);
struct lms7002m_decibel lms7002m_get_rbbpga_db(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_set_rfelna_db(
    struct lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel);
struct lms7002m_decibel lms7002m_get_rfelna_db(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_set_rfe_loopback_lna_db(
    struct lms7002m_context* self, const struct lms7002m_decibel gain, const enum lms7002m_channel channel);
struct lms7002m_decibel lms7002m_get_rfe_loopback_lna_db(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_set_rfetia_db(
    struct lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel);
struct lms7002m_decibel lms7002m_get_rfetia_db(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_set_trfpad_db(
    struct lms7002m_context* self, const struct lms7002m_decibel value, const enum lms7002m_channel channel);
struct lms7002m_decibel lms7002m_get_trfpad_db(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_set_trf_loopback_pad_db(
    struct lms7002m_context* self, const struct lms7002m_decibel gain, const enum lms7002m_channel channel);
struct lms7002m_decibel lms7002m_get_trf_loopback_pad_db(struct lms7002m_context* self, const enum lms7002m_channel channel);

lime_Result lms7002m_set_path_rfe(struct lms7002m_context* self, const enum lms7002m_path_rfe path);
uint8_t lms7002m_get_path_rfe(struct lms7002m_context* self);

lime_Result lms7002m_set_band_trf(struct lms7002m_context* self, const uint8_t band);
uint8_t lms7002m_get_band_trf(struct lms7002m_context* self);

lime_Result lms7002m_set_path(struct lms7002m_context* self, bool isTx, enum lms7002m_channel channel, uint8_t path);

uint32_t lms7002m_get_reference_clock_tsp(struct lms7002m_context* self, bool isTx);
bool lms7002m_get_cgen_locked(struct lms7002m_context* self);
bool lms7002m_get_sx_locked(struct lms7002m_context* self, bool isTx);

lime_Result lms7002m_tune_vco(struct lms7002m_context* self, enum lms7002m_vco_type module);

lime_Result lms7002m_set_frequency_sx(struct lms7002m_context* self, bool isTx, uint64_t freq_Hz);
uint64_t lms7002m_get_frequency_sx(struct lms7002m_context* self, bool isTx);

lime_Result lms7002m_set_nco_frequency(struct lms7002m_context* self, bool isTx, const uint8_t index, uint32_t freq_Hz);
uint32_t lms7002m_get_nco_frequency(struct lms7002m_context* self, bool isTx, const uint8_t index);

lime_Result lms7002m_set_nco_phase_offset(struct lms7002m_context* self, bool isTx, uint8_t index, int16_t pho_calculated);

lime_Result lms7002m_set_nco_phase_offset_for_mode_0(struct lms7002m_context* self, bool isTx, int16_t pho_calculated);
lime_Result lms7002m_set_nco_phases(
    struct lms7002m_context* self, bool isTx, const int16_t* const angles_deg, uint8_t count, uint32_t frequencyOffset);
lime_Result lms7002m_get_nco_phases(
    struct lms7002m_context* self, bool isTx, int16_t* angles_deg, uint8_t count, uint32_t* frequencyOffset);

lime_Result lms7002m_set_nco_frequencies(
    struct lms7002m_context* self, bool isTx, const uint32_t* const freq_Hz, uint8_t count, int16_t phaseOffset);
lime_Result lms7002m_get_nco_frequencies(
    struct lms7002m_context* self, bool isTx, uint32_t* const freq_Hz, uint8_t count, int16_t* phaseOffset);

lime_Result lms7002m_set_gfir_coefficients(
    struct lms7002m_context* self, bool isTx, uint8_t gfirIndex, const int16_t* const coef, uint8_t coefCount);
lime_Result lms7002m_get_gfir_coefficients(
    struct lms7002m_context* self, bool isTx, uint8_t gfirIndex, int16_t* const coef, uint8_t coefCount);

lime_Result lms7002m_set_interface_frequency(
    struct lms7002m_context* self, uint32_t cgen_freq_Hz, const uint8_t hbi, const uint8_t hbd);

lime_Result lms7002m_enable_sxtdd(struct lms7002m_context* self, bool tdd);

lime_Result lms7002m_set_dc_offset(struct lms7002m_context* self, bool isTx, const uint8_t I, const uint8_t Q);
lime_Result lms7002m_get_dc_offset(struct lms7002m_context* self, bool isTx, uint8_t* const I, uint8_t* const Q);

lime_Result lms7002m_set_i_q_balance(
    struct lms7002m_context* self, bool isTx, const int16_t iqcorr, const uint16_t gcorri, const uint16_t gcorrq);
lime_Result lms7002m_get_i_q_balance(
    struct lms7002m_context* self, bool isTx, int16_t* const iqcorr, uint16_t* const gcorri, uint16_t* const gcorrq);

/// @param temperature_mC Chip temperature in millidegree Celsius
lime_Result lms7002m_get_temperature(struct lms7002m_context* self, int32_t* temperature_mC);

lime_Result lms7002m_set_clock_frequency(struct lms7002m_context* self, enum lms7002m_clock_id clk_id, uint32_t freq);
uint32_t lms7002m_get_clock_frequency(struct lms7002m_context* self, enum lms7002m_clock_id clk_id);

uint32_t lms7002m_get_sample_rate(struct lms7002m_context* self, bool isTx, enum lms7002m_channel ch);

lime_Result lms7002m_set_rx_lpf(struct lms7002m_context* self, uint32_t rfBandwidth_Hz);
lime_Result lms7002m_set_tx_lpf(struct lms7002m_context* self, uint32_t rfBandwidth_Hz);

int16_t lms7002m_read_analog_dc(struct lms7002m_context* self, const uint16_t addr);

uint32_t lms7002m_get_rssi(struct lms7002m_context* self);

lime_Result lms7002m_load_dc_reg_iq(struct lms7002m_context* self, bool isTx, int16_t I, int16_t Q);

// Calibrations

lime_Result lms7002m_calibrate_rx(struct lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback, bool dcOnly);
lime_Result lms7002m_calibrate_tx(struct lms7002m_context* self, uint32_t bandwidthRF, bool extLoopback);
lime_Result lms7002m_calibrate_internal_adc(struct lms7002m_context* self, int clkDiv);
lime_Result lms7002m_calibrate_rp_bias(struct lms7002m_context* self);
lime_Result lms7002m_calibrate_analog_rssi_dc_offset(struct lms7002m_context* self);

#ifdef __cplusplus
}
#endif

#endif // LIME_LMS7002M_H
