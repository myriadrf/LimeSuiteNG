/*
 * LimeSuiteNG public C API.
 *
 * Stable C ABI over the C++ internal implementation. Opaque handles,
 * lime_ snake_case entry points, lime_OpStatus return codes.
 *
 * This header is the 1.0 public surface. The C++ headers under
 * limesuiteng/ are the internal implementation, exported only during the
 * transition window (see the 1.0 C-ABI freeze proposal).
 */
#ifndef LIMESUITENG_C_H
#define LIMESUITENG_C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined(_WIN32)
    #if defined(LIME_C_BUILD)
        #define LIME_C_API __declspec(dllexport)
    #else
        #define LIME_C_API __declspec(dllimport)
    #endif
#elif defined(__GNUC__)
    #define LIME_C_API __attribute__((visibility("default")))
#else
    #define LIME_C_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ *
 * Error codes (mirror lime::OpStatus). 0 == success.                 *
 * ------------------------------------------------------------------ */
typedef enum {
    lime_OpStatus_Success          =  0,
    lime_OpStatus_Error            = -1,
    lime_OpStatus_NotImplemented   = -2,
    lime_OpStatus_IOFailure        = -3,
    lime_OpStatus_InvalidValue     = -4,
    lime_OpStatus_FileNotFound     = -5,
    lime_OpStatus_OutOfRange       = -6,
    lime_OpStatus_NotSupported     = -7,
    lime_OpStatus_Timeout          = -8,
    lime_OpStatus_Busy             = -9,
    lime_OpStatus_Aborted          = -10,
    lime_OpStatus_PermissionDenied = -11,
    lime_OpStatus_NotConnected     = -12
} lime_OpStatus;

typedef enum { lime_TRXDir_Rx = 0, lime_TRXDir_Tx = 1 } lime_TRXDir;

/* ------------------------------------------------------------------ *
 * Opaque handles. The library owns them; never dereference or free   *
 * directly -- use the _open / _create and _close / _destroy pairs.   *
 * ------------------------------------------------------------------ */
typedef struct lime_device        lime_device;        /* generic: SDR / RFE / Utility */
typedef struct lime_SDRDevice     lime_SDRDevice;
typedef struct lime_GPIO          lime_GPIO;
typedef struct lime_Stream        lime_Stream;
typedef struct lime_SDRDescriptor lime_SDRDescriptor; /* opaque; read via lime_descriptor_* accessors */

/* ------------------------------------------------------------------ *
 * Enumeration & device tree.                                         *
 * ------------------------------------------------------------------ */
typedef struct { char str[256]; } lime_DeviceHandle;

/* Fills up to 'max' handles; returns the count found (>=0),
 * or a negative lime_OpStatus on failure. */
LIME_C_API int          lime_enumerate(lime_DeviceHandle* out, size_t max);
LIME_C_API lime_device* lime_device_open(const lime_DeviceHandle* handle);
LIME_C_API void         lime_device_close(lime_device* dev);

typedef enum { lime_Kind_SDR = 0, lime_Kind_RFE = 1, lime_Kind_Utility = 2 } lime_Kind;
LIME_C_API lime_Kind       lime_device_kind(lime_device* dev);
LIME_C_API size_t          lime_device_child_count(lime_device* dev);
LIME_C_API lime_device*    lime_device_child(lime_device* dev, size_t index);
LIME_C_API lime_SDRDevice* lime_device_as_sdr(lime_device* dev); /* NULL if not an SDR */

/* ------------------------------------------------------------------ *
 * SDRDevice. Channels are addressed by (module, dir, channel).       *
 * ------------------------------------------------------------------ */
LIME_C_API lime_OpStatus lime_sdrdevice_set_frequency(
    lime_SDRDevice* dev, uint8_t module, lime_TRXDir dir, uint8_t channel, double hz);
LIME_C_API lime_OpStatus lime_sdrdevice_enable_channel(
    lime_SDRDevice* dev, uint8_t module, lime_TRXDir dir, uint8_t channel, bool enable);
/* Antenna is selected by name (not index). */
LIME_C_API lime_OpStatus lime_sdrdevice_set_antenna(
    lime_SDRDevice* dev, uint8_t module, lime_TRXDir dir, uint8_t channel, const char* name);

LIME_C_API const lime_SDRDescriptor* lime_sdrdevice_get_descriptor(lime_SDRDevice* dev);

/* ------------------------------------------------------------------ *
 * Descriptor accessors. The descriptor is library-owned: pointers    *
 * and strings returned here stay valid for the device's lifetime.    *
 * Out-of-range arguments yield NULL / 0.                             *
 * ------------------------------------------------------------------ */
LIME_C_API const char* lime_descriptor_name(const lime_SDRDescriptor* d);
LIME_C_API uint64_t    lime_descriptor_serial(const lime_SDRDescriptor* d);
LIME_C_API size_t      lime_descriptor_rfsoc_count(const lime_SDRDescriptor* d);
LIME_C_API const char* lime_descriptor_rfsoc_name(const lime_SDRDescriptor* d, size_t soc);
LIME_C_API uint8_t     lime_descriptor_channel_count(const lime_SDRDescriptor* d, size_t soc);
/* Antenna names: the vocabulary for lime_sdrdevice_set_antenna(). */
LIME_C_API size_t      lime_descriptor_antenna_count(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dir);
LIME_C_API const char* lime_descriptor_antenna_name(
    const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dir, size_t index);

/* ------------------------------------------------------------------ *
 * Streaming (the hot path).                                          *
 * format is the application layout; link_format is the wire layout;  *
 * a conversion runs whenever they differ.                            *
 * ------------------------------------------------------------------ */
typedef enum { /* mirrors lime::DataFormat */
    lime_DataFormat_I16 = 0, /* 16-bit integers */
    lime_DataFormat_I12 = 1, /* 12-bit integers stored as int16_t, range [-2048;2047] */
    lime_DataFormat_F32 = 2 /* 32-bit floating-point */
} lime_DataFormat;

/* Transparent, caller-built, append-only. struct_size = sizeof(lime_StreamConfig)
 * lets the ABI grow: the library never reads past the size the caller declares. */
typedef struct {
    uint32_t struct_size; /* = sizeof(lime_StreamConfig) */
    uint8_t module; /* RF SoC index the stream attaches to */
    const uint8_t* rx_channels; /* channel indices to stream; may be NULL when the count is 0 */
    size_t rx_count;
    const uint8_t* tx_channels;
    size_t tx_count;
    lime_DataFormat format; /* samples layout for _recv / _send */
    lime_DataFormat link_format; /* wire layout Host<->FPGA */
    double hint_sample_rate_hz; /* 0 = decide internally */
} lime_StreamConfig;

/* Unified at the C boundary: flush is Tx-only and ignored on Rx. */
typedef struct {
    uint64_t timestamp; /* in sample ticks */
    bool has_timestamp;
    bool flush; /* Tx: submit a partially filled packet (end of burst) */
} lime_StreamMeta;

LIME_C_API lime_Stream*  lime_stream_create(lime_SDRDevice* dev, const lime_StreamConfig* cfg);
LIME_C_API lime_OpStatus lime_stream_start(lime_Stream* s);
LIME_C_API void          lime_stream_stop(lime_Stream* s);
/* dst/src: one buffer pointer per configured channel, laid out per cfg->format.
 * Return: samples transferred per channel (>=0), or a negative lime_OpStatus. */
LIME_C_API int lime_stream_recv(
    lime_Stream* s, void* const* dst, size_t count, lime_StreamMeta* meta, uint32_t timeout_ms);
LIME_C_API int lime_stream_send(
    lime_Stream* s, const void* const* src, size_t count, const lime_StreamMeta* meta, uint32_t timeout_ms);
LIME_C_API void lime_stream_destroy(lime_Stream* s);

/* ------------------------------------------------------------------ *
 * Subinterface providers. Return NULL if the capability is absent.   *
 * ------------------------------------------------------------------ */
LIME_C_API lime_GPIO*    lime_sdrdevice_get_gpio(lime_SDRDevice* dev);
LIME_C_API lime_OpStatus lime_gpio_set_value(lime_GPIO* gpio, uint32_t pin, bool value);
LIME_C_API lime_OpStatus lime_gpio_get_value(lime_GPIO* gpio, uint32_t pin, bool* value);

typedef struct lime_SPI lime_SPI;
LIME_C_API lime_SPI* lime_sdrdevice_get_spi(lime_SDRDevice* dev);
/* Full-duplex SPI transaction on the given internal bus. bus_address is the
 * chip-select id from the device descriptor. mosi or miso may be NULL for
 * read-only / write-only transfers. */
LIME_C_API lime_OpStatus lime_spi_transact(
    lime_SPI* spi, uint32_t bus_address, const uint32_t* mosi, uint32_t* miso, uint32_t count);

/* ------------------------------------------------------------------ *
 * Version.                                                           *
 * ------------------------------------------------------------------ */
LIME_C_API const char* lime_get_library_version(void);
LIME_C_API const char* lime_get_api_version(void);
LIME_C_API const char* lime_get_abi_version(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_C_H */
