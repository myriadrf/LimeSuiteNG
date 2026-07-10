/**
 * @file limesuiteng/rfstream_c.h
 * @author Lime Microsystems
 * @brief RF sample streaming: configuration, metadata, and data transfer.
 */
#ifndef LIMESUITENG_RFSTREAM_C_H
#define LIMESUITENG_RFSTREAM_C_H

#include "limesuiteng/types_c.h"
#include "limesuiteng/sdrdevice_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque RF stream handle. Created with lime_stream_create(). */
typedef struct lime_Stream lime_Stream;

/**
 * @brief Stream configuration. Caller-built and append-only: new fields are only
 * ever added at the end, and the library never reads past @ref struct_size.
 */
typedef struct {
    uint32_t struct_size; ///< Must be set to sizeof(lime_StreamConfig)
    uint32_t module_index; ///< The RF SoC index the stream attaches to
    const uint32_t* rx_channels; ///< Receive channel indices to stream; may be NULL when the count is 0
    size_t rx_count; ///< Number of entries in rx_channels
    const uint32_t* tx_channels; ///< Transmit channel indices to stream; may be NULL when the count is 0
    size_t tx_count; ///< Number of entries in tx_channels
    lime_DataFormat format; ///< Sample layout used by lime_stream_recv() / lime_stream_send()
    lime_DataFormat link_format; ///< Sample layout on the wire (host <-> FPGA)
    double hint_sample_rate_hz; ///< Expected sampling rate for transfer optimizations; 0 = decide internally
} lime_StreamConfig;

/** @brief Receive metadata, filled by lime_stream_recv(). */
typedef struct {
    uint64_t timestamp; ///< Time of the first sample in the buffer, in sample ticks
    bool has_timestamp; ///< Whether timestamp is valid
} lime_StreamRxMeta;

/** @brief Transmit metadata, read by lime_stream_send(). */
typedef struct {
    uint64_t timestamp; ///< When the first submitted sample should be transmitted, in sample ticks
    bool has_timestamp; ///< Wait for timestamp before transmitting
    bool flush; ///< Submit a partially filled packet (end of burst)
} lime_StreamTxMeta;

/**
 * @brief Creates an RF data stream on a device.
 * @param dev The device to stream from/to.
 * @param cfg The stream configuration; struct_size must be set.
 * @return The stream, or NULL on failure. Release with lime_stream_destroy().
 */
LIME_C_API lime_Stream* lime_stream_create(lime_SDRDevice* dev, const lime_StreamConfig* cfg);

/**
 * @brief Starts the stream.
 * @param s The stream to start.
 * @return The status of the operation.
 */
LIME_C_API lime_OpStatus lime_stream_start(lime_Stream* s);

/**
 * @brief Stops the stream and clears its internal buffers.
 * @param s The stream to stop; NULL is allowed and ignored.
 */
LIME_C_API void lime_stream_stop(lime_Stream* s);

/**
 * @brief Receives samples from all configured receive channels.
 * @param s The stream to receive from.
 * @param[out] dst One destination buffer per configured Rx channel, laid out per the configured format.
 * @param count The number of samples to receive per channel.
 * @param[out] meta Receive metadata, or NULL if not needed.
 * @param timeout_ms The maximum wait time for incoming samples (in milliseconds).
 * @return The number of samples received per channel (>= 0), or a negative lime_OpStatus on failure.
 */
LIME_C_API int lime_stream_recv(lime_Stream* s, void* const* dst, size_t count, lime_StreamRxMeta* meta, uint32_t timeout_ms);

/**
 * @brief Transmits samples on all configured transmit channels.
 * @param s The stream to transmit on.
 * @param src One source buffer per configured Tx channel, laid out per the configured format.
 * @param count The number of samples to transmit per channel.
 * @param meta Transmit metadata, or NULL for defaults.
 * @param timeout_ms The maximum wait time for buffer space (in milliseconds).
 * @return The number of samples submitted per channel (>= 0), or a negative lime_OpStatus on failure.
 */
LIME_C_API int lime_stream_send(
    lime_Stream* s, const void* const* src, size_t count, const lime_StreamTxMeta* meta, uint32_t timeout_ms);

/**
 * @brief Stops the stream if needed, frees its resources, and destroys the handle.
 * @param s The stream to destroy; NULL is allowed and ignored.
 */
LIME_C_API void lime_stream_destroy(lime_Stream* s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_RFSTREAM_C_H */
