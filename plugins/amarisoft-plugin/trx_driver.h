/*
 * Amarisoft Transceiver API version 2024-06-14
 * Copyright (C) 2013-2024 Amarisoft
 */
#ifndef TRX_DRIVER_H
#define TRX_DRIVER_H

#include <inttypes.h>
#include <stdarg.h>

#define TRX_API_VERSION 15


/* These 2 defines are only used in TRXDriverParams structure (trx_start function) */
/* they must be kept for backward compatibility of TRX API */
#define TRX_MAX_CHANNELS 16
#define TRX_MAX_RF_PORT  TRX_MAX_CHANNELS

typedef struct TRXState TRXState;

typedef int64_t trx_timestamp_t;

/* Currently only complex floating point samples are supported */
typedef struct {
    float re;
    float im;
} TRXComplex;

typedef struct {
    int num;
    int den;
} TRXFraction;

typedef struct {
    uint8_t period_asn1;
    uint8_t dl_slots;
    uint8_t dl_symbs;
    uint8_t ul_slots;
    uint8_t ul_symbs;
} TDDPattern;

typedef struct {

    int         rf_port_index;
    uint32_t    dl_earfcn;  /* aka arfcn for NR */
    uint32_t    ul_earfcn;  /* aka arfcn for NR */
    int         n_rb_dl;
    int         n_rb_ul;

    enum {
        TRX_CYCLIC_PREFIX_NORMAL,
        TRX_CYCLIC_PREFIX_EXTENDED,
    } dl_cyclic_prefix, ul_cyclic_prefix;

    enum {
        TRX_CELL_TYPE_FDD,
        TRX_CELL_TYPE_TDD,
        TRX_NBCELL_TYPE_FDD,
        TRX_NBCELL_TYPE_TDD, /* Not available */
        TRX_NRCELL_TYPE_FDD,
        TRX_NRCELL_TYPE_TDD,
    } type;

    union {
        struct {
            uint8_t uldl_config;
            uint8_t special_subframe_config;
        } tdd;
        struct {
            int band;
            int dl_subcarrier_spacing; /* mu: [0..3] */
            int ul_subcarrier_spacing; /* mu: [0..3] */
        } nr_fdd;
        struct {
            int band;
            int dl_subcarrier_spacing; /* mu: [0..3] */
            int ul_subcarrier_spacing; /* mu: [0..3] */

            /* TDD */
            uint8_t period_asn1;
            uint8_t dl_slots;
            uint8_t dl_symbs;
            uint8_t ul_slots;
            uint8_t ul_symbs;
            TDDPattern *pattern2;
        } nr_tdd;

        uint64_t pad[32];
    } u;

} TRXCellInfo;

typedef struct {
    /* number of RX channels (=RX antennas) */
    int rx_channel_count;
    /* number of TX channels (=TX antennas) */
    int tx_channel_count;
    /* RX center frequency in Hz for each channel */
    int64_t rx_freq[TRX_MAX_CHANNELS];
    /* TX center frequency in Hz for each channel */
    int64_t tx_freq[TRX_MAX_CHANNELS];
    /* initial rx_gain for each channel, same unit as trx_set_rx_gain_func() */
    double rx_gain[TRX_MAX_CHANNELS];
    /* initial tx_gain for each channel, same unit as trx_set_tx_gain_func() */
    double tx_gain[TRX_MAX_CHANNELS];
    /* RX bandwidth, in Hz for each channel */
    int rx_bandwidth[TRX_MAX_CHANNELS];
    /* TX bandwidth, in Hz for each channel */
    int tx_bandwidth[TRX_MAX_CHANNELS];

    /* Number of RF ports.
     * A separate trx_write() is be done for each TX port on different thread.
     * Each TX port can have a different TDD configuration.
     * A separate trx_read() is be done for each RX port on different thread.
     */
    int rf_port_count;

    /* the sample rate for both transmit and receive.
     * One for each port */
    TRXFraction sample_rate[TRX_MAX_RF_PORT];

    /* Arrays of rf_port_count elements containing the number of
       channels per TX/RX port. Their sum must be equal to
       tx_channel_count/rx_channel_count. */
    int tx_port_channel_count[TRX_MAX_RF_PORT];
    int rx_port_channel_count[TRX_MAX_RF_PORT];

    /* Array of cell_count elements pointing to information
     * on each cell */
    TRXCellInfo *cell_info;
    int cell_count;

    int s72_enable; /* 0 = I/Q samples mode, 1 = split 7.2 mode with trx_write_packet()/trx_read_packet() */
} TRXDriverParams;

/* New TRXDriverParams2 structure used by trx_start2 */
typedef struct {
    /* number of RX channels (=RX antennas) */
    int rx_channel_count;
    /* number of TX channels (=TX antennas) */
    int tx_channel_count;

    /* Following are pointers to arrays with size rx_channel_count or tx_channel_cout */

    /* RX center frequency in Hz for each channel */
    int64_t *rx_freq;
    /* TX center frequency in Hz for each channel */
    int64_t *tx_freq;
    /* initial rx_gain for each channel, same unit as trx_set_rx_gain_func() */
    double *rx_gain;
    /* initial tx_gain for each channel, same unit as trx_set_tx_gain_func() */
    double *tx_gain;
    /* RX bandwidth, in Hz for each channel */
    int *rx_bandwidth;
    /* TX bandwidth, in Hz for each channel */
    int *tx_bandwidth;

    /* Number of RF ports.
     * A separate trx_write() is be done for each TX port on different thread.
     * Each TX port can have a different TDD configuration.
     * A separate trx_read() is be done for each RX port on different thread.
     */
    int rf_port_count;

    /* Following are pointers to arrays with size rf_port_count */

    /* the sample rate for both transmit and receive.
     * One for each port */
    TRXFraction *sample_rate;
    /* Arrays of rf_port_count elements containing the number of
       channels per TX/RX port. Their sum must be equal to
       tx_channel_count/rx_channel_count. */
    int *tx_port_channel_count;
    int *rx_port_channel_count;


    /* Array of cell_count elements pointing to information
     * on each cell */
    TRXCellInfo *cell_info;
    int cell_count;

    /* assume little endian CPU, s72_enable is now a uint8_t */
    uint8_t s72_enable; /* 0 = I/Q samples mode, 1 = split 7.2 mode with trx_write_packet()/trx_read_packet() */
    /* 3 flags added to keep original size */
    uint8_t flags1;
#define PARM_FLAGS1_TX_ABS_POWER    1   /* tx_gain array contains tx_abs_power_dbm */

    uint8_t flags2;
    uint8_t flags3;

} TRXDriverParams2;

typedef struct {
    /* Number of times the data was sent too late by the application. */
    int64_t tx_underflow_count;
    /* Number of times the receive FIFO overflowed. */
    int64_t rx_overflow_count;
} TRXStatistics;

typedef void __attribute__ ((format (printf, 2, 3))) (*trx_printf_cb)(void *, const char *fmt, ... );


/*
 * Structure used for trx_write_func2:
 */
typedef struct {

#define TRX_WRITE_MD_FLAG_PADDING            (1 << 0)       /* Cf trx_write_func */
#define TRX_WRITE_MD_FLAG_END_OF_BURST       (1 << 1)       /* Cf trx_write_func */
#define TRX_WRITE_MD_FLAG_HARQ_ACK_PRESENT   (1 << 2)       /* Cf harq_flags */
#define TRX_WRITE_MD_FLAG_TA_PRESENT         (1 << 3)       /* Cf ta */
#define TRX_WRITE_MD_FLAG_CUR_TIMESTAMP_REQ  (1 << 4)       /* Cf cur_timestamp field */
#define TRX_WRITE_MD_UNDERFLOW               (1 << 10)

    uint32_t flags; /* In: set by application */

    /* HARQ
     * only used for TDD
     * HARQ/ACK info, only used for eNodeB testing
     * set if HARQ ACK/NACK info is present in the subframe
     */
#define TRX_WRITE_MD_HARQ_ACK0              (1 << 0)
#define TRX_WRITE_MD_HARQ_ACK1              (1 << 1)
    uint8_t harq_ack; /* In: set by application */

    /* TA: 6 bits */
    uint8_t timing_advance; /* In: set by application */

    /* Current real time timestamp.
     * Out: set by driver
     * If TRX_WRITE_MD_FLAG_CUR_TIMESTAMP_REQ, cur_timestamp may be set by callee.
     * It represents real time timestamp used to compute RX/TX latency.
     * Set cur_timestamp_set to non 0 to notify caller.
     */
    uint8_t cur_timestamp_set;
    trx_timestamp_t cur_timestamp;

    uint8_t underflow; /* Out: set by driver to indicate if an underflow has occured */
} TRXWriteMetadata;

/*
 * Structure used for trx_read_func2:
 */
typedef struct {
#define TRX_READ_MD_OVERFLOW                (1 << 0)
#define TRX_READ_MD_AGC_CHANGE              (1 << 1)

    uint32_t flags;

    uint8_t overflow;   /* 0/1 */
    uint8_t agc_change; /* 0/1 */
} TRXReadMetadata;


/*
 * Structure used for trx_xxx_packet:
 */
typedef struct {
    uint8_t *data;
    size_t len;
    void *user_data;
    /* TX only
     * in samples since first private start of slot
     * packet returned by trx_read_packet
     * Defines when the packet content is supposed to be sent to the air
     * If < 0, the packet needs to be sent right now
     */
    int64_t timestamp;
} TRXPacketVec;

typedef struct {
#define TRX_WRITE_PACKET_MD_FLAG_TIME_BUDGET  (1 << 0)  /* Driver met report time budget */
#define TRX_WRITE_PACKET_MD_UNDERFLOWS        (1 << 1)
    uint64_t flags; /* In: set by application */

    uint8_t time_budget_set;
    int64_t time_budget_us;
    uint16_t underflows;

} TRXWritePacketMetadata;

typedef struct {
#define TRX_READ_PACKET_MD_FLAG_OVERFLOW  (1 << 0)
#define TRX_READ_PACKET_MD_FLAG_RELEASE   (1 << 1)
    uint64_t flags;

    uint8_t overflow;
    int release_count;
} TRXReadPacketMetadata;

typedef struct {
    uint16_t max_pdu_size; /* Maximum eCPRI PDU size (TX) */
    uint16_t max_read_packets; /* Maximum number of packets for trx_read_packet */

    /* If non zero, packets provided by a call to trx_read_packet
     * may not be available on next call. In that case, trx_read_packet
     * implementation must look at TRXReadPacketMetadata->release_count
     * to know which packet can be reused
     * If set to 0, TRXReadPacketMetadata->release_count can be skipped
     * And application won't use packets after next call to
     * trx_read_packe*/
    uint8_t rx_async_release;
    uint8_t reserved[251];

} TRXPacketConfig;

/* Deprecated, used for trx_write_func
 * Refer to trx_write_func2 flags */
#define TRX_WRITE_FLAG_PADDING              TRX_WRITE_MD_FLAG_PADDING
#define TRX_WRITE_FLAG_END_OF_BURST         TRX_WRITE_MD_FLAG_END_OF_BURST

/* Structure used for set_agc and get_agc */
typedef struct {
#define TRX_AGC_OFF            0x00
#define TRX_AGC_ON             0x01
#define TRX_AGC_USE_DB         0x02    /* Use high_db and low_db values from struct (else: default)*/
    /* high_db must be < 0.0 and low_db < high_db - 3.0 */
#define TRX_AGC_USE_MAX_GAIN   0x04    /* Use max_gain from struct */
#define TRX_AGC_USE_MIN_GAIN   0x08    /* Use min_gain from struct */

    int flags;

    float high_db;
    float low_db;

    float max_gain;
    float min_gain;
} TRXAGCParams;


typedef struct TRXMsg TRXMsg;

typedef enum {
    TRX_MSG_GET_OK,
    TRX_MSG_GET_NOT_SET = -1,
    TRX_MSG_GET_BAD_TYPE = -2,
    TRX_MSG_GET_ALREADY_SENT = -3,
} TRXMsgGetCode;

struct TRXMsg {
    void *opaque;

    /* Message in API
     * Only used for received messages
     */
    TRXMsgGetCode (*get_double)(TRXMsg *msg, double *pval, const char *prop_name);
    TRXMsgGetCode (*get_string)(TRXMsg *msg, const char **pval, const char *prop_name);

    /* Message out API */
    void (*set_double)(TRXMsg *msg, const char *prop_name, double val);
    void (*set_string)(TRXMsg *msg, const char *prop_name, const char *string);

    /* Always call this once, even in timeout_cb */
    void (*send)(TRXMsg *msg);

    /* Async handling
     * Only used for received messages
     */
    void (*set_timeout)(TRXMsg *msg, void (*timeout_cb)(TRXMsg*), int timeout_ms);

    /* User defined data */
    void *user_data;
};

typedef struct {
    int64_t gps_time;   /* in seconds since 1 Jan 1970, UTC, -1 if error */
    double latitude;    /* in degres, positive for 'N', negative for 'S' [-90 .. 90] */
    double longitude;   /* in degres, positive for 'E', negative for 'W' [-180 .. 180] */
    double height;      /* height in meter */
    int nb_sats;        /* number of sats [0..12] */
    double geoid_sep;   /* difference between ellipsoid and mean sea level (m) */
} TRXGPSInfo;

typedef enum {
    TRX_LOG_NONE,
    TRX_LOG_ERROR,
    TRX_LOG_WARN,
    TRX_LOG_INFO,
    TRX_LOG_DEBUG,
} TRXLogLevel;

typedef enum {
    TRX_LOG_DIR_UNKNOWN,
    TRX_LOG_TX,
    TRX_LOG_RX,
} TRXLogDir;

typedef struct {
    const char *name;
    void *reserved[63]; /* For further use */

} TRXPThreadConfig;

struct TRXState {
    /* API version */
    int trx_api_version;

    /* set by the application - do not modify */
    void *app_opaque;
    /* set by the application - do not modify */
    char *(*trx_get_param_string)(void *app_opaque, const char *prop_name);
    /* set by the application - do not modify */
    int (*trx_get_param_double)(void *app_opaque, double *pval,
                                const char *prop_name);
    /* set by the application - do not modify */
    void (*trx_log_func)(void *app_opaque2, TRXLogLevel level, TRXLogDir dir, const char *fmt, va_list ap);

    /* If the drivers uses its own threads and trx_log_func or trx_msg_send_func
     * are called from them, the threads should be posix threads and those function
     * should be called at thread start/end
     * trx_pthread_init:
     *      config must be memseted to 0
     *      return 0 if success, -1 otherwise
     */
    int (*trx_pthread_init)(void *app_opaque2, const TRXPThreadConfig *config);
    void (*trx_pthread_terminate)(void *app_opaque2);


    void *reserved[123];

    /* set by the application - do not modify */
    void *app_opaque2;

    /* Path of the config file, not terminated by / */
    const char *path;

    /* Can be set by the driver to point to internal data */
    void *opaque;

    /* the following functions are called by the application */

    /* Return in *psample_rate the sample rate supported by the device
       corresponding to a LTE bandwidth of 'bandwidth' Hz. Also return
       in n=*psample_rate_num the wanted sample rate before the
       interpolator as 'n * 1.92' MHz. 'n' must currently be of the
       form 2^n1*3^n2*5^n3.

       Return 0 if OK, -1 if none. */
    int (*trx_get_sample_rate_func)(TRXState *s, TRXFraction *psample_rate,
                                    int *psample_rate_num,
                                    int bandwidth);

    /* Obsolete: not supported on recent TRX drivers
     * Use trx_start_func2 instead
     */
    int (*trx_start_func)(TRXState *s, const TRXDriverParams *p);

    /* Deprecated, use trx_write_func2 instead. */
    void (*trx_write_func)(TRXState *s, trx_timestamp_t timestamp, const void **samples, int count, int flags, int tx_port_index);

    /* Deprecated, use trx_read_func2 instead. */
    int (*trx_read_func)(TRXState *s, trx_timestamp_t *ptimestamp, void **samples, int count, int rx_port_index);

    /* Dynamic set the transmit gain (in dB). The origin and range are
       driver dependent.

       Note: this function is only used for user supplied dynamic
       adjustements.
    */
    void (*trx_set_tx_gain_func)(TRXState *s, double gain, int channel_num);

    /* Dynamic set the receive gain (in dB). The origin and range are
       driver dependent.

       Note: this function is only used for user supplied dynamic
       adjustements.
    */
    void (*trx_set_rx_gain_func)(TRXState *s, double gain, int channel_num);

    /* Terminate operation of the transceiver - free all associated
       resources, do not call other APIs after trx_end */
    void (*trx_end_func)(TRXState *s);

    /* Return the maximum number of samples per TX packet. Called by
     * the application after trx_start_func.
     * Optional
     */
    int (*trx_get_tx_samples_per_packet_func)(TRXState *s);

    /* Return some statistics. Return 0 if OK, < 0 if not available. */
    int (*trx_get_stats)(TRXState *s, TRXStatistics *m);

    /* Callback must allocate info buffer that will be displayed */
    void (*trx_dump_info)(TRXState *s, trx_printf_cb cb, void *opaque);

    /* Return the absolute TX power in dBm for the TX channel
       'channel_num' assuming a square signal of maximum
       amplitude. This function can be called from any thread and
       needs to be fast. Return 0 if OK, -1 if the result is not
       available. */
    int (*trx_get_abs_tx_power_func)(TRXState *s,
                                     float *presult, int channel_num);

    /* Return the absolute RX power in dBm for the RX channel
       'channel_num' assuming a square signal of maximum
       amplitude. This function can be called from any thread and
       needs to be fast. Return 0 if OK, -1 if the result is not
       available. */
    int (*trx_get_abs_rx_power_func)(TRXState *s,
                                     float *presult, int channel_num);

    /* Read/Write IQ samples
       Available since API v13
       Write 'count' samples on each channel of the TX port
       'tx_port_index'. samples[0] is the array for the first
       channel. timestamp is the time (in samples) at which the first
       sample must be sent. When the TRX_WRITE_FLAG_PADDING flag is
       set, samples is set to NULL. It indicates that no data should
       be sent (TDD receive time). TRX_WRITE_FLAG_END_OF_BURST is set
       to indicate in advance that the next write call will have the
       TRX_WRITE_FLAG_PADDING flag set. Note:
       TRX_WRITE_FLAG_END_OF_BURST and TRX_WRITE_FLAG_PADDING are
       never set simultaneously.
    */
    void (*trx_write_func2)(TRXState *s, trx_timestamp_t timestamp, const void **samples,
                            int count, int tx_port_index, TRXWriteMetadata *md);

    /* Read IQ samples
       Available since API v13
       Read at most 'count' samples from each channel. samples[0] is the array
       for the first channel. *ptimestamp is the time at which the
       first samples was received. Return the number of sample read
       (<=count).

       Note: It is explicitly allowed that the application calls
       trx_write_func, trx_read_func, trx_set_tx_gain_func and
       trx_set_rx_gain_func from different threads.
     */
    int (*trx_read_func2)(TRXState *s, trx_timestamp_t *ptimestamp, void **samples, int count,
                          int rx_port_index, TRXReadMetadata *md);

    /* Remote API communication
     * Available since API v14
     * trx_msg_recv_func: called for each trx received messages
     * trx_msg_send_func: call it to send trx messages (They must be registered by client)
     * For each message, a call to send API must be done
     */
    void (*trx_msg_recv_func)(TRXState *s, TRXMsg *msg);
    TRXMsg* (*trx_msg_send_func)(TRXState *s); /* set by application, do notmodify it */

    /* Return actual transmit gain (in dB). The origin and range are
       driver dependent.
    */
    void (*trx_get_tx_gain_func)(TRXState *s, double *gain, int channel_num);

    /* Returns actual receive gain (in dB). The origin and range are
       driver dependent.
    */
    void (*trx_get_rx_gain_func)(TRXState *s, double *gain, int channel_num);

    /* Stop operation of the transceiver - to be called after trx_start.
       resources allocated in init are not released, so trx_call can be called again */
    void (*trx_stop_func)(TRXState *s);

    /* Split 7.2 API */

    /* Write ECPRI packets. The TRX driver is responsible to send them
       at the correct time to meet the RU timing requirement.
       Should not be blocking */
    void (*trx_write_packet)(TRXState *s, const TRXPacketVec **packets, int count,
                             int tx_port_index, TRXWritePacketMetadata *md);
    /* Read a ECPRI packets. Return number of packet received. The BBU
       expects a private ECPRI packet at the beginning of each new
       uplink slot.
       It is up to the driver to allocate packet data and pass it to the application
       using packets[n].data. Data will be kept by application depending
       on TRXPacketConfig->rx_async_release value.
       If set rx_async_release is set, the packets array parameter acts as input and
       output. As input, array will be filled with TRXReadPacketMetadata->release_count
       packets that can be reused by the driver or deallocated.
       This mechanism will help to reduce latency between calls ro read packets.
     */
    int (*trx_read_packet)(TRXState *s, TRXPacketVec **packets, int max_packets,
                           int rx_port_index, TRXReadPacketMetadata *md);
    int (*trx_get_packet_config)(TRXState *s, int rf_port_index, TRXPacketConfig *cfg);

    /* If set, drivers needs to allocate data of requested size in packet->data
     * Each allocated packet will be filled with ecpri data and passed through
     * trx_write_packet.
     * Release of data is up to the driver after being called by trx_write_packet
     * If not set, allocation will be done by application and release will be done
     * right after calling trx_write_packet.
     */
    int (*trx_get_packet)(TRXState *s, TRXPacketVec *packet, size_t size, int tx_port_index);

    /* Called to start the tranceiver. Can use more than 16 channels */
    /* p can be NULL if trx_set_start_params() was called before. */
    int (*trx_start_func2)(TRXState *s, const TRXDriverParams2 *p);

    /* AGC functions */
    int (*trx_set_agc_func)(TRXState *s, const TRXAGCParams *p, int channel);
    int (*trx_get_agc_func)(TRXState *s, TRXAGCParams *p, int channel);

    /* Log function:
     * Called by application to notify current log level and allow early decision of
     * log sending via trx_log_func
     * If this function is not used, the application will check log level anyway
     */
    void (*trx_log_set_level_func)(TRXState *s, TRXLogLevel level); /* Maximum (included) log level */

    /* Deprecated: see trx_get_numa_nodes2 */
    int (*trx_get_numa_nodes)(TRXState *s, const TRXDriverParams2 *params, uint64_t *numa_nodes);

    /* Mulithreaded read/write IQ samples */

    /* Requires trx_get_timestamp_func to be implemented
     * Must fill samples buffer with the exact number of samples
     * Must not be blocking
     * Return <0 in case of samples not available
     * Return 0 on success
     */
    int (*trx_read_mt_func)(TRXState *s, trx_timestamp_t timestamp, void **samples, int count,
                            int rx_port_index, TRXReadMetadata *md);
    /* Must return the greatest sample timestamp available +1
     * Must block until *ptimestamp becomes greater than the one returned by previous call
     * Will always be called from same thread (For each RF port)
     * trx_read_mt_func will never be called with timestamp greater than the last one reported
     * Returns 0 on success, <0 on error
     */
    int (*trx_read_timestamp_func)(TRXState *s, trx_timestamp_t *ptimestamp, int rx_port_index);
    /* Same as trx_write_func2 but must support multi-threading */
    void (*trx_write_mt_func)(TRXState *s, trx_timestamp_t timestamp, const void **samples,
                              int count, int tx_port_index, TRXWriteMetadata *md);

    /* Sets the maximum tx_gain to reach the requested TX power in dBm at 0dBFS.
       trx_start must have been used before calling this function.
       Return 0 if OK, -1 if the result is not available. */
    int (*trx_set_abs_tx_power_func)(TRXState *s, double power, int channel);

    /* Fills the GPS Info structure with data parsed from NMEA stream
       Return 0 if OK, -1 if the result is not available. */
    int (*trx_get_gps_info)(TRXState *s, TRXGPSInfo *gps);

    /* When using multi-threaded write mode, this call will guarantee
     * trx_write_mt_func will never be called with a lower timestamp
     * May be used to flush pending IQ samples
     */
    void (*trx_write_mt_end_func)(TRXState *s, trx_timestamp_t timestamp, int tx_port_index);

    /* set start parameters. can be called before trx_start_func2()
       Returns 0 on success, <0 on error */
    int (*trx_set_start_params)(TRXState *s, const TRXDriverParams2 *p);

    /* Dynamic multi_thread write / read indicator (may depend on compression scheme) */
    /* should be called AFTER compression format is known */
    int (*trx_get_mt_status)(TRXState *s, int *mt_write, int *mt_read, int rf_port_index);

    /* takes a tx_power and a channel number and returns the matching tx_gain */
    /* to be called after set_start_params */
    int (*trx_get_tx_gain_from_power)(TRXState *s, double *tx_gain, double power, int channel);

    /* set cell info, requires trx_set_start_params to be implemented
     * and will be called after trx_set_start_params and before trx_start_func2 */
    void (*trx_set_cell_info)(TRXState *s, const TRXCellInfo *cell_info, int cell_count);

    /* Provide hint for to upper layer for memory allocator and cpu affinity
     * If your hardware is bound to specific NUMA node(s), the software can take advantage
     * of this information to reduce inter node memory bandwidth usage
     * As this method is use before trx_start_func2, it uses same TRXDriverParams2 structure
     * returns:
     *   <0 on error
     *   0 on success
     * numa_nodes is an array of 64bits values.
     * Each 64 bits value is a bitmask for each rf port.
     * If numa_nodes[rf_port_index] is 0, no NUMA config will be applied
     * Else, each bit set to 1 will tell application to use this node (bit 0 is for node 0...)
     */
    int (*trx_get_numa_nodes2)(TRXState *s, uint64_t *numa_nodes);

    /* Reserved for further usage */
    void *reserved2[112];

};

/* return 0 if OK, < 0 if error. */
int trx_driver_init(TRXState *s);

/* Get string parameter from configuration. Must be freed with
   free(). Return NULL if property does not exists. Can only be called
   in trx_driver_init(). */
static inline char *trx_get_param_string(TRXState *s, const char *prop_name)
{
    return s->trx_get_param_string(s->app_opaque, prop_name);
}

/* Get floating point parameter from configuration. Must be freed with
   free(). Return 0 if OK or < 0 if property does not exists. Can only be called
   in trx_driver_init(). */
static inline int trx_get_param_double(TRXState *s, double *pval,
                                       const char *prop_name)
{
    return s->trx_get_param_double(s->app_opaque, pval, prop_name);
}

/* Add logs */
static inline void trx_log_full(TRXState *s, TRXLogLevel level, TRXLogDir dir, const char *fmt, va_list ap)
{
    if (s->trx_log_func) {
        s->trx_log_func(s->app_opaque2, level, dir, fmt, ap);
    }
}

static inline void __attribute__ ((format (printf, 3, 4))) trx_log(TRXState *s, TRXLogLevel level, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    trx_log_full(s, level, TRX_LOG_DIR_UNKNOWN, fmt, ap);
    va_end(ap);
}

static inline void __attribute__ ((format (printf, 4, 5)))
    trx_log1(TRXState *s, TRXLogLevel level, TRXLogDir dir, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    trx_log_full(s, level, dir, fmt, ap);
    va_end(ap);
}

#endif /* TRX_DRIVER_H */
