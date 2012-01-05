// Copyright (c) 2010 Joseph D Poirier
// Distributable under the terms of The New BSD License
// that can be found in the LICENSE file.

#ifndef DEFS_H
#define DEFS_H

#ifdef USE_NED
  #define free nedfree
#endif

#ifdef NDEBUG
 #ifndef _STATIC_ASSERT_
  #define static_assert(e) \
    do { \
      enum {static_assert__ = 1/(e)}; \
    } while (0)
 #endif

 #ifndef  _STATIC_ASSERT_MSG_
  #define static_assert_msg(e, msg) \
    do { \
      enum { static_assert_msg__ ## msg = 1/(e)}; \
    } while (0)
 #endif
#endif

// Standard printf
#ifdef NPRINTF
    #define PRINTF(fmt)
    #define PRINTF_VA(fmt, ...)
#else
# ifdef __TESTING__
    #define PRINTF(fmt)                 pout.putf(fmt)
    #define PRINTF_VA(fmt, ...)         pout.putf(fmt, __VA_ARGS__)
# else
    #define PRINTF(fmt)                 XPLMDebugString(fmt)
    #define PRINTF_VA(fmt, ...)         XPLMDebugString(fmt, __VA_ARGS__)
# endif
#endif

// Debug printf
#ifdef NDEBUG
    #define DPRINTF(fmt)
    #define DPRINTF_VA(fmt, ...)
#else
# ifdef __TESTING__
    #define DPRINTF(fmt)                pout.putf(fmt)
    #define DPRINTF_VA(fmt, ...)        pout.putf(fmt, __VA_ARGS__)
# else
    #define DPRINTF(fmt)                XPLMDebugString(fmt)
    #define DPRINTF_VA(fmt, ...)
# endif
#endif

#ifdef LOGPRINTF
    #define LPRINTF(fmt)                XPLMDebugString(fmt)
    #define LPRINTF_VA(fmt, ...)
#else
    #define LPRINTF(fmt)
    #define LPRINTF_VA(fmt, ...)
#endif

// Error printf
#ifdef NEPRINTF
    #define EPRINTF(fmt)
    #define EPRINTF_VA(fmt, ...)
#else
# ifdef __TESTING__
    #define EPRINTF(fmt)                pout.putf(fmt)
    #define EPRINTF_VA(fmt, ...)        pout.putf(fmt, __VA_ARGS__)
# else
    #define EPRINTF(fmt)                XPLMDebugString(fmt)
    #define EPRINTF_VA(fmt, ...)        XPLMDebugString(fmt, __VA_ARGS__)
# endif
#endif

// CommandHandler pre-event and post-event designators
#define CMD_HNDLR_PROLOG (1)
#define CMD_HNDLR_EPILOG (0)

#define IN_BUF_CNT  (4)
#define OUT_BUF_CNT (13)
#define MSG_NOWAIT  (0)
#define MSG_WAIT    (-1)

typedef void (*pHidInit) ();
typedef unsigned char* (*pProcOutData) (unsigned int);


#define MP_APBTN_BITPOS     (0)
#define MP_HDGBTN_BITPOS    (1)
#define MP_NAVBTN_BITPOS    (2)
#define MP_IASBTN_BITPOS    (3)
#define MP_ALTBTN_BITPOS    (4)
#define MP_VSBTN_BITPOS     (5)
#define MP_APRBTN_BITPOS    (6)
#define MP_REVBTN_BITPOS    (7)

#define MP_READ_KNOB_MODE_MASK      (0x0000001F)
#define MP_READ_BTNS_MASK           (0x00007F80)
#define MP_READ_FLAPS_MASK          (0x00030000)
#define MP_READ_TRIM_MASK           (0x000C0000)
#define MP_READ_TUNING_MASK         (0x00000060)
#define MP_READ_THROTTLE_MASK       (0x00008000)
#define MP_READ_KNOB_ALT            (0x00000001)
#define MP_READ_KNOB_VS             (0x00000002)
#define MP_READ_KNOB_IAS            (0x00000004)
#define MP_READ_KNOB_HDG            (0x00000008)
#define MP_READ_KNOB_CRS            (0x00000010)
#define MP_READ_TUNING_RIGHT        (0x00000020)
#define MP_READ_TUNING_LEFT         (0x00000040)
#define MP_READ_AP_BTN              (0x00000080)
#define MP_READ_HDG_BTN             (0x00000100)
#define MP_READ_NAV_BTN             (0x00000200)
#define MP_READ_IAS_BTN             (0x00000400)
#define MP_READ_ALT_BTN             (0x00000800)
#define MP_READ_VS_BTN              (0x00001000)
#define MP_READ_APR_BTN             (0x00002000)
#define MP_READ_REV_BTN             (0x00004000)
#define MP_READ_THROTTLE_ON         (0x00008000)
#define MP_READ_THROTTLE_OFF        (0x00000000)
#define MP_READ_FLAPS_UP            (0x00010000)
#define MP_READ_FLAPS_DN            (0x00020000)
#define MP_READ_TRIM_DOWN           (0x00040000)
#define MP_READ_TRIM_UP             (0x00080000)
#define MP_READ_NOMSG               (0xFFFFFFFF)

#define READ_SP_MASTER_BAT_MASK       (0x000001)
#define READ_SP_MASTER_ALT_MASK       (0x000002)
#define READ_SP_AVIONICS_MASTER_MASK  (0x000004)
#define READ_SP_FUEL_PUMP_MASK        (0x000008)
#define READ_SP_DE_ICE_MASK           (0x000010)
#define READ_SP_PITOT_HEAT_MASK       (0x000020)
#define READ_SP_COWL_MASK             (0x000040)
#define READ_SP_LIGHTS_PANEL_MASK     (0x000080)
#define READ_SP_LIGHTS_BEACON_MASK    (0x000100)
#define READ_SP_LIGHTS_NAV_MASK       (0x000200)
#define READ_SP_LIGHTS_STROBE_MASK    (0x000400)
#define READ_SP_LIGHTS_TAXI_MASK      (0x000800)
#define READ_SP_LIGHTS_LANDING_MASK   (0x001000)
#define READ_SP_ENGINES_KNOB_MASK     (0x03E000)
#define READ_SP_GEARLEVER_DOWN_MASK   (0x040000)
#define READ_SP_GEARLEVER_UP_MASK     (0x080000)

enum {
    MP_MPM_CNT              = 3, // MP multi-part message
    HID_READ_CNT            = 4,
    HID_ERROR               = -1,
    VENDOR_ID               = 0x06A3,
    RP_PROD_ID              = 0x0D05,
    MP_PROD_ID              = 0x0D06,
    SP_PROD_ID              = 0x0D67,
    RP_ERROR_THRESH         = 40,
    MP_ERROR_THRESH         = 40,
    SP_ERROR_THRESH         = 40,
    PANEL_CHECK_INTERVAL    = 3,

    // global message ids
    // pt::MSG_USER = 0 and pt::message id = MSG_USER + 1
    AVIONICS_ON_MSG = 2,
    AVIONICS_OFF_MSG,
    BAT1_ON_MSG,
    BAT1_OFF_MSG,
    MP_BLANK_SCRN_MSG,
    MP_ZERO_SCRN_MSG,
    MP_PLANE_CRASH_MSG,
    MP_BTN_AP_OFF_MSG,
    MP_BTN_AP_ON_MSG,
    MP_BTN_AP_ARMED_MSG,
    MP_BTN_HDG_OFF_MSG,
    MP_BTN_HDG_ARMED_MSG,
    MP_BTN_HDG_CAPT_MSG,
    MP_BTN_NAV_OFF_MSG,
    MP_BTN_NAV_ARMED_MSG,
    MP_BTN_NAV_CAPT_MSG,
    MP_BTN_IAS_OFF_MSG,
    MP_BTN_IAS_ARMED_MSG,
    MP_BTN_IAS_CAPT_MSG,
    MP_BTN_ALT_OFF_MSG,
    MP_BTN_ALT_ARMED_MSG,
    MP_BTN_ALT_CAPT_MSG,
    MP_BTN_VS_OFF_MSG,
    MP_BTN_VS_ARMED_MSG,
    MP_BTN_VS_CAPT_MSG,
    MP_BTN_APR_OFF_MSG,
    MP_BTN_APR_ARMED_MSG,
    MP_BTN_APR_CAPT_MSG,
    MP_BTN_REV_OFF_MSG,
    MP_BTN_REV_ARMED_MSG,
    MP_BTN_REV_CAPT_MSG,
    MP_KNOB_ALT_POS_MSG,
    MP_KNOB_VS_POS_MSG,
    MP_KNOB_IAS_POS_MSG,
    MP_KNOB_HDG_POS_MSG,
    MP_KNOB_CRS_POS_MSG,
    MP_ALT_VAL_MSG,
    MP_VS_VAL_POS_MSG,
    MP_VS_VAL_NEG_MSG,
    MP_IAS_VAL_MSG,
    MP_HDG_VAL_MSG,
    MP_CRS_VAL_MSG,

    // multi panel command messages
    MP_PITCHTRIM_UP_CMD_MSG,
    MP_PITCHTRIM_DN_CMD_MSG,
    MP_FLAPS_UP_CMD_MSG,
    MP_FLAPS_DN_CMD_MSG,
    MP_AUTOTHROTTLE_ON_CMD_MSG,
    MP_AUTOTHROTTLE_OFF_CMD_MSG,
    MP_BTN_AP_TOGGLE_CMD_MSG,
    MP_BTN_HDG_TOGGLE_CMD_MSG,
    MP_BTN_NAV_TOGGLE_CMD_MSG,
    MP_BTN_IAS_TOGGLE_CMD_MSG,
    MP_BTN_ALT_TOGGLE_CMD_MSG,
    MP_BTN_VS_TOGGLE_CMD_MSG,
    MP_BTN_APR_TOGGLE_CMD_MSG,
    MP_BTN_REV_TOGGLE_CMD_MSG,
    MP_ALT_UP_CMD_MSG,
    MP_ALT_DN_CMD_MSG = MP_ALT_UP_CMD_MSG + 1,
    MP_VS_UP_CMD_MSG,
    MP_VS_DN_CMD_MSG = MP_VS_UP_CMD_MSG + 1,
    MP_IAS_UP_CMD_MSG,
    MP_IAS_DN_CMD_MSG = MP_IAS_UP_CMD_MSG + 1,
    MP_HDG_UP_CMD_MSG,
    MP_HDG_DN_CMD_MSG = MP_HDG_UP_CMD_MSG + 1,
    MP_CRS_UP_CMD_MSG,
    MP_CRS_DN_CMD_MSG = MP_CRS_UP_CMD_MSG + 1,

    // switch panel
    SP_MAGNETOS_OFF,
    SP_MAGNETOS_RIGHT,
    SP_MAGNETOS_LEFT,
    SP_MAGNETOS_BOTH,
    SP_MAGNETOS_START,
    SP_MASTER_BATTERY_ON,
    SP_MASTER_BATTERY_OFF,
    SP_MASTER_ALT_BATTERY_ON,
    SP_MASTER_ALT_BATTERY_OFF,
    SP_MASTER_AVIONICS_ON,
    SP_MASTER_AVIONICS_OFF,
    SP_FUEL_PUMP_ON,
    SP_FUEL_PUMP_OFF,
    SP_DEICE_ON,
    SP_DEICE_OFF,
    SP_PITOT_HEAT_ON,
    SP_PITOT_HEAT_OFF,
    SP_COWL_CLOSED,
    SP_COWL_OPEN,
    SP_LIGHTS_PANEL_ON,
    SP_LIGHTS_PANEL_OFF,
    SP_LIGHTS_BEACON_ON,
    SP_LIGHTS_BEACON_OFF,
    SP_LIGHTS_NAV_ON,
    SP_LIGHTS_NAV_OFF,
    SP_LIGHTS_STROBE_ON,
    SP_LIGHTS_STROBE_OFF,
    SP_LIGHTS_TAXI_ON,
    SP_LIGHTS_TAXI_OFF,
    SP_LIGHTS_LANDING_ON,
    SP_LIGHTS_LANDING_OFF,
    SP_LANDING_GEAR_UP,
    SP_LANDING_GEAR_DOWN,

    MP_MPM                  = 0x0FFFFFFF, // multi-part message
    EXITING_THREAD_LOOP     = 0xFFFFFFFF,
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif  /* DEFS_H */

