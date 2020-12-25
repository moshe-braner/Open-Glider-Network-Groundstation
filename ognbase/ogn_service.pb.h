/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.4-dev */

#ifndef PB_OGN_SERVICE_PB_H_INCLUDED
#define PB_OGN_SERVICE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _type_4 {
    pb_callback_t header;
    pb_callback_t value1;
    pb_callback_t value2;
    pb_callback_t value3;
} type_4;

typedef struct _type_5 {
    pb_callback_t header;
} type_5;

typedef struct _FanetService {
    float latitude;
    float longitude;
    bool has_Type_4;
    type_4 Type_4;
    bool has_Type_5;
    type_5 Type_5;
} FanetService;

typedef struct _ReceiverConfiguration {
    bool has_maxrange;
    int32_t maxrange;
    bool has_band;
    int32_t band;
    bool has_protocol;
    int32_t protocol;
    bool has_aprsd;
    bool aprsd;
    bool has_aprsp;
    int32_t aprsp;
    bool has_itrackb;
    bool itrackb;
    bool has_istealthb;
    bool istealthb;
    bool has_sleepm;
    int32_t sleepm;
    bool has_rxidle;
    int32_t rxidle;
    bool has_wakeup;
    int32_t wakeup;
    bool has_reset;
    bool reset;
    bool has_reboot;
    bool reboot;
    bool has_alt;
    float alt;
    bool has_zabbixen;
    bool zabbixen;
} ReceiverConfiguration;

typedef struct _OneMessage {
    bool has_fanetService;
    FanetService fanetService;
    bool has_receiverConfiguration;
    ReceiverConfiguration receiverConfiguration;
} OneMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define type_4_init_default                      {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define type_5_init_default                      {{{NULL}, NULL}}
#define FanetService_init_default                {0, 0, false, type_4_init_default, false, type_5_init_default}
#define ReceiverConfiguration_init_default       {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define OneMessage_init_default                  {false, FanetService_init_default, false, ReceiverConfiguration_init_default}
#define type_4_init_zero                         {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define type_5_init_zero                         {{{NULL}, NULL}}
#define FanetService_init_zero                   {0, 0, false, type_4_init_zero, false, type_5_init_zero}
#define ReceiverConfiguration_init_zero          {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define OneMessage_init_zero                     {false, FanetService_init_zero, false, ReceiverConfiguration_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define type_4_header_tag                        1
#define type_4_value1_tag                        2
#define type_4_value2_tag                        3
#define type_4_value3_tag                        4
#define type_5_header_tag                        1
#define FanetService_latitude_tag                1
#define FanetService_longitude_tag               2
#define FanetService_Type_4_tag                  3
#define FanetService_Type_5_tag                  4
#define ReceiverConfiguration_maxrange_tag       1
#define ReceiverConfiguration_band_tag           2
#define ReceiverConfiguration_protocol_tag       3
#define ReceiverConfiguration_aprsd_tag          4
#define ReceiverConfiguration_aprsp_tag          5
#define ReceiverConfiguration_itrackb_tag        6
#define ReceiverConfiguration_istealthb_tag      7
#define ReceiverConfiguration_sleepm_tag         8
#define ReceiverConfiguration_rxidle_tag         9
#define ReceiverConfiguration_wakeup_tag         10
#define ReceiverConfiguration_reset_tag          11
#define ReceiverConfiguration_reboot_tag         12
#define ReceiverConfiguration_alt_tag            13
#define ReceiverConfiguration_zabbixen_tag       14
#define OneMessage_fanetService_tag              1
#define OneMessage_receiverConfiguration_tag     2

/* Struct field encoding specification for nanopb */
#define type_4_FIELDLIST(X, a) \
X(a, CALLBACK, REQUIRED, BYTES,    header,            1) \
X(a, CALLBACK, OPTIONAL, BYTES,    value1,            2) \
X(a, CALLBACK, OPTIONAL, BYTES,    value2,            3) \
X(a, CALLBACK, OPTIONAL, BYTES,    value3,            4)
#define type_4_CALLBACK pb_default_field_callback
#define type_4_DEFAULT NULL

#define type_5_FIELDLIST(X, a) \
X(a, CALLBACK, REQUIRED, BYTES,    header,            1)
#define type_5_CALLBACK pb_default_field_callback
#define type_5_DEFAULT NULL

#define FanetService_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    latitude,          1) \
X(a, STATIC,   REQUIRED, FLOAT,    longitude,         2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  Type_4,            3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  Type_5,            4)
#define FanetService_CALLBACK NULL
#define FanetService_DEFAULT NULL
#define FanetService_Type_4_MSGTYPE type_4
#define FanetService_Type_5_MSGTYPE type_5

#define ReceiverConfiguration_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    maxrange,          1) \
X(a, STATIC,   OPTIONAL, INT32,    band,              2) \
X(a, STATIC,   OPTIONAL, INT32,    protocol,          3) \
X(a, STATIC,   OPTIONAL, BOOL,     aprsd,             4) \
X(a, STATIC,   OPTIONAL, INT32,    aprsp,             5) \
X(a, STATIC,   OPTIONAL, BOOL,     itrackb,           6) \
X(a, STATIC,   OPTIONAL, BOOL,     istealthb,         7) \
X(a, STATIC,   OPTIONAL, INT32,    sleepm,            8) \
X(a, STATIC,   OPTIONAL, INT32,    rxidle,            9) \
X(a, STATIC,   OPTIONAL, INT32,    wakeup,           10) \
X(a, STATIC,   OPTIONAL, BOOL,     reset,            11) \
X(a, STATIC,   OPTIONAL, BOOL,     reboot,           12) \
X(a, STATIC,   OPTIONAL, FLOAT,    alt,              13) \
X(a, STATIC,   OPTIONAL, BOOL,     zabbixen,         14)
#define ReceiverConfiguration_CALLBACK NULL
#define ReceiverConfiguration_DEFAULT NULL

#define OneMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  fanetService,      1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  receiverConfiguration,   2)
#define OneMessage_CALLBACK NULL
#define OneMessage_DEFAULT NULL
#define OneMessage_fanetService_MSGTYPE FanetService
#define OneMessage_receiverConfiguration_MSGTYPE ReceiverConfiguration

extern const pb_msgdesc_t type_4_msg;
extern const pb_msgdesc_t type_5_msg;
extern const pb_msgdesc_t FanetService_msg;
extern const pb_msgdesc_t ReceiverConfiguration_msg;
extern const pb_msgdesc_t OneMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define type_4_fields &type_4_msg
#define type_5_fields &type_5_msg
#define FanetService_fields &FanetService_msg
#define ReceiverConfiguration_fields &ReceiverConfiguration_msg
#define OneMessage_fields &OneMessage_msg

/* Maximum encoded size of messages (where known) */
/* type_4_size depends on runtime parameters */
/* type_5_size depends on runtime parameters */
/* FanetService_size depends on runtime parameters */
#define ReceiverConfiguration_size               94
/* OneMessage_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
