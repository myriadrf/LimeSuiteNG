#ifndef LIME_LMS7002M_H
#define LIME_LMS7002M_H

#include "limesuiteng/embedded/result.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct lms7002m_context;

enum lms7002m_vco_type { LMS7002M_VCO_CGEN, LMS7002M_VCO_SXR, LMS7002M_VCO_SXT };

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

float lms7002m_get_reference_clock(struct lms7002m_context* context);
lime_Result lms7002m_set_reference_clock(struct lms7002m_context* context, float frequency_Hz);

uint8_t lms7002m_get_active_channel(struct lms7002m_context* self);
lime_Result lms7002m_set_active_channel(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_frequency_cgen(struct lms7002m_context* context, float frequency_Hz);

lime_Result lms7002m_set_rbbpga_db(struct lms7002m_context* self, const float value, const uint8_t channel);
float lms7002m_get_rbbpga_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_rfelna_db(struct lms7002m_context* self, const float value, const uint8_t channel);
float lms7002m_get_rfelna_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_rfe_loopback_lna_db(struct lms7002m_context* self, const float gain, const uint8_t channel);
float lms7002m_get_rfe_loopback_lna_db(struct lms7002m_context* self, const uint8_t channel);

lime_Result lms7002m_set_rfetia_db(struct lms7002m_context* self, const float value, const uint8_t channel);
float lms7002m_get_rfetia_db(struct lms7002m_context* self, const uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif // LIME_LMS7002M_H
