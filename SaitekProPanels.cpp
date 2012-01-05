// Copyright (c) 2010 Joseph D Poirier
// Distributable under the terms of The New BSD License
// that can be found in the LICENSE file.

#if defined(__WINDOWS__)
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif

#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "pport.h"
#include "ptypes.h"
#include "pasync.h"
#include "ptime.h"
#include "pstreams.h"

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

//#include "overloaded.h"
#include "defs.h"
#include "utils.h"
#include "hidapi.h"
#include "radiopanel.h"
#include "multipanel.h"
#include "switchpanel.h"
#include "PanelThreads.h"
#include "SaitekProPanels.h"


int RadioPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void*             inRefcon);

int MultiPanelCommandHandler(XPLMCommandRef inCommand,
                             XPLMCommandPhase inPhase,
                             void* inRefcon);

int SwitchPanelCommandHandler(XPLMCommandRef    inCommand,
                              XPLMCommandPhase  inPhase,
                              void*             inRefcon);

float RadioPanelFlightLoopCallback(float inElapsedSinceLastCall,
                                   float inElapsedTimeSinceLastFlightLoop,
                                   int inCounter,
                                   void* inRefcon);

float MultiPanelFlightLoopCallback(float inElapsedSinceLastCall,
                                   float inElapsedTimeSinceLastFlightLoop,
                                   int inCounter,
                                   void* inRefcon);

float SwitchPanelFlightLoopCallback(float inElapsedSinceLastCall,
                                    float inElapsedTimeSinceLastFlightLoop,
                                    int inCounter,
                                    void* inRefcon);

void mp_do_init();

//logfile* gLogFile;
//char gLogFilePath[512] = {};

/*
sim/flightmodel2/controls/flap_handle_deploy_ratio
sim/aircraft/specialcontrols/acf_ail1flaps
*/

USING_PTYPES

// Multi panel
enum {
    MP_CMD_EAT_EVENT = 0,
    MP_CMD_PASS_EVENT = 1,
    MP_CMD_OTTO_AUTOTHROTTLE_ON = 0,
    MP_CMD_OTTO_AUTOTHROTTLE_OFF,
    MP_CMD_FLAPS_UP,
    MP_CMD_FLAPS_DOWN,
    MP_CMD_PITCHTRIM_UP,
    MP_CMD_PITCHTRIM_DOWN,
    MP_CMD_OTTO_ON,
    MP_CMD_OTTO_OFF,
    MP_CMD_OTTO_ARMED,
    MP_CMD_OTTO_AP_BTN,
    MP_CMD_OTTO_HDG_BTN,
    MP_CMD_OTTO_NAV_BTN,
    MP_CMD_OTTO_IAS_BTN,
    MP_CMD_OTTO_ALT_ARM_BTN,
    MP_CMD_OTTO_ALT_HOLD_BTN,
    MP_CMD_OTTO_VS_BTN,
    MP_CMD_OTTO_APR_BTN,
    MP_CMD_OTTO_REV_BTN,
    MP_CMD_OTTO_ALT_UP,
    MP_CMD_OTTO_ALT_DN,
    MP_CMD_OTTO_VS_UP,
    MP_CMD_OTTO_VS_DN,
    MP_CMD_OTTO_IAS_UP,
    MP_CMD_OTTO_IAS_DN,
    MP_CMD_OTTO_HDG_UP,
    MP_CMD_OTTO_HDG_DN,
    MP_CMD_OTTO_CRS_UP,
    MP_CMD_OTTO_CRS_DN,

    CMD_SYS_AVIONICS_ON,
    CMD_SYS_AVIONICS_OFF,
    CMD_ELEC_BATTERY1_ON,
    CMD_ELEC_BATTERY1_OFF,
};

// Switch panel
enum {
    CMD_MAGNETOS_OFF,
    CMD_MAGNETOS_RIGHT,
    CMD_MAGNETOS_LEFT,
    CMD_MAGNETOS_BOTH,
    CMD_MAGNETOS_START,
    CMD_MASTER_BATTERY_ON,
    CMD_MASTER_BATTERY_OFF,
    CMD_MASTER_ALT_BATTERY_ON,
    CMD_MASTER_ALT_BATTERY_OFF,
    CMD_MASTER_AVIONICS_ON,
    CMD_MASTER_AVIONICS_OFF,
    CMD_FUEL_PUMP_ON,
    CMD_FUEL_PUMP_OFF,
    CMD_DEICE_ON,
    CMD_DEICE_OFF,
    CMD_PITOT_HEAT_ON,
    CMD_PITOT_HEAT_OFF,
    CMD_COWL_CLOSED,
    CMD_COWL_OPEN,
    CMD_LIGHTS_PANEL_ON,
    CMD_LIGHTS_PANEL_OFF,
    CMD_LIGHTS_BEACON_ON,
    CMD_LIGHTS_BEACON_OFF,
    CMD_LIGHTS_NAV_ON,
    CMD_LIGHTS_NAV_OFF,
    CMD_LIGHTS_STROBE_ON,
    CMD_LIGHTS_STROBE_OFF,
    CMD_LIGHTS_TAXI_ON,
    CMD_LIGHTS_TAXI_OFF,
    CMD_LIGHTS_LANDING_ON,
    CMD_LIGHTS_LANDING_OFF,
    CMD_GEAR_UP,
    CMD_GEAR_DOWN
};

// Flightloop callback message queue processing count defaults.
// TODO: create a small user menu with a user adjustment slider (range ?)
enum {
    RP_MSGPROC_CNT = 50,
    MP_MSGPROC_CNT = 50,
    SP_MSGPROC_CNT = 50
};

// Flightloop callback message queue processing globals.
// These should be adjustable by the user via a menu item.
// For X-Plane v9.x, and possibly v10, a higher count means
// we interrupt X-Plane proper and a lower count increases
// panel input/output latency.
int32_t gRp_MsgProc_Cnt = RP_MSGPROC_CNT;
int32_t gMp_MsgProc_Cnt = MP_MSGPROC_CNT;
int32_t gSp_MsgProc_Cnt = SP_MSGPROC_CNT;

int32_t gAvPwrOn = false;
int32_t gBat1On = false;
int32_t gPlaneLoaded = false;
int32_t gPluginEnabled = false;
uint32_t gFlCbCnt = 0;

// rp = Rp = RP = Radio Panel
// mp = Mp = MP = Milti Panel
// sp = Sp = SP = Switch Panel
// cb = Cb = CB = Callback
// Flightloop Callback INterval
static const float FL_CB_INTERVAL = -1.0;

XPLMDataRef gAvPwrOnDataRef = NULL;
XPLMDataRef gBatPwrOnDataRef = NULL;

/* Multi Panel Command Refs */
XPLMCommandRef gAvPwrOnCmdRef = NULL;
XPLMCommandRef gAvPwrOffCmdRef = NULL;
XPLMCommandRef gBatPwrOnCmdRef = NULL;
XPLMCommandRef gBatPwrOffCmdRef = NULL;

XPLMCommandRef gMpAsDnCmdRef = NULL;
XPLMCommandRef gMpAsUpCmdRef = NULL;
XPLMCommandRef gMpAltDnCmdRef = NULL;
XPLMCommandRef gMpAltUpCmdRef = NULL;
XPLMCommandRef gMpAltHoldCmdRef = NULL;
XPLMCommandRef gMpAltArmCmdRef = NULL;
XPLMCommandRef gMpAppCmdRef = NULL;
XPLMCommandRef gMpAtThrrtlOnCmdRef = NULL;
XPLMCommandRef gMpAtThrrtlOffCmdRef = NULL;
XPLMCommandRef gMpAtThrrtlTgglCmdRef = NULL;
XPLMCommandRef gMpBkCrsCmdRef = NULL;
XPLMCommandRef gMpFlpsDnCmdRef = NULL;
XPLMCommandRef gMpFlpsUpCmdRef = NULL;
XPLMCommandRef gMpHdgCmdRef = NULL;
XPLMCommandRef gMpHdgDnCmdRef = NULL;
XPLMCommandRef gMpHdgUpCmdRef = NULL;
XPLMCommandRef gMpLvlChngCmdRef = NULL;
XPLMCommandRef gMpNavArmCmdRef = NULL;
XPLMCommandRef gMpObsHsiDnCmdRef = NULL;
XPLMCommandRef gMpObsHsiUpCmdRef = NULL;
XPLMCommandRef gMpPtchTrmDnCmdRef = NULL;
XPLMCommandRef gMpPtchTrmUpCmdRef = NULL;
XPLMCommandRef gMpPtchTrmTkOffCmdRef = NULL;
XPLMCommandRef gMpApOnCmdRef = NULL;
XPLMCommandRef gMpApOffCmdRef = NULL;
XPLMCommandRef gMpApArmedCmdRef = NULL;
XPLMCommandRef gMpApToggleCmdRef = NULL;
XPLMCommandRef gMpVrtclSpdDnCmdRef = NULL;
XPLMCommandRef gMpVrtclSpdUpCmdRef = NULL;
XPLMCommandRef gMpVrtclSpdCmdRef = NULL;

// Multi Panel Data Refs
XPLMDataRef gMpOttoOvrrde = NULL;
//XPLMDataRef gMpAltDialFtDataRef = NULL;
XPLMDataRef gMpAltHoldStatDataRef = NULL;
//XPLMDataRef gMpAltArmStatDataRef = NULL;
XPLMDataRef gMpApprchStatDataRef = NULL;

XPLMDataRef gMpApOnDataRef = NULL;
XPLMDataRef gMpArspdDataRef = NULL;

XPLMDataRef gMpBckCrsStatDataRef = NULL;
XPLMDataRef gMpFlghtDirModeDataRef = NULL;
XPLMDataRef gMpHdgMagDataRef = NULL;
XPLMDataRef gMpHdgStatDataRef = NULL;
XPLMDataRef gMpHsiObsDegMagPltDataRef = NULL;
XPLMDataRef gMpNavStatDataRef = NULL;
XPLMDataRef gMpSpdStatDataRef = NULL;
XPLMDataRef gMpVrtVelDataRef = NULL;
XPLMDataRef gMpVviStatDataRef = NULL;

XPLMDataRef gMpAltHoldFtDataRef = NULL;

// Multi Panel reference counters
int32_t gMpPitchTrimUpPending = 0;
int32_t gMpPitchTrimDnPending = 0;
int32_t gMpFlapsUpPending = 0;
int32_t gMpFlapsDnPending = 0;
int32_t gMpBtn_Ap_TogglePending = 0;
int32_t gMpBtn_Hdg_TogglePending = 0;
int32_t gMpBtn_Nav_TogglePending = 0;
int32_t gMpBtn_Ias_TogglePending = 0;
int32_t gMpBtn_Alt_TogglePending = 0;
int32_t gMpBtn_Vs_TogglePending = 0;
int32_t gMpBtn_Apr_TogglePending = 0;
int32_t gMpBtn_Rev_TogglePending = 0;
int32_t gMpAutothrottle_offPending = 0;
int32_t gMpAutothrottle_onPending = 0;
int32_t gMpAlt_Pending = 0;
int32_t gMpVs_Pending = 0;
int32_t gMpIas_Pending = 0;
int32_t gMpHdg_Pending = 0;
int32_t gMpCrs_Pending = 0;

/* SWITCH PANEL Command Refs */
XPLMCommandRef gSpMagnetosOffCmdRef = NULL;
XPLMCommandRef gSpMagnetosRightCmdRef = NULL;
XPLMCommandRef gSpMagnetosLeftCmdRef = NULL;
XPLMCommandRef gSpMagnetosBothCmdRef = NULL;
XPLMCommandRef gSpMagnetosStartCmdRef = NULL;
XPLMCommandRef gSpMasterBatteryOnCmdRef = NULL;
XPLMCommandRef gSpMasterBatteryOffCmdRef = NULL;
XPLMCommandRef gSpMasterAltBatteryOnCmdRef = NULL;
XPLMCommandRef gSpMasterAltBatteryOffCmdRef = NULL;
XPLMCommandRef gSpMasterAvionicsOnCmdRef = NULL;
XPLMCommandRef gSpMasterAvionicsOffCmdRef = NULL;
XPLMCommandRef gSpFuelPumpOnCmdRef = NULL;
XPLMCommandRef gSpFuelPumpOffCmdRef = NULL;
XPLMCommandRef gSpDeIceOnCmdRef = NULL;
XPLMCommandRef gSpDeIceOffCmdRef = NULL;
XPLMCommandRef gSpPitotHeatOnCmdRef = NULL;
XPLMCommandRef gSpPitotHeatOffCmdRef = NULL;
XPLMCommandRef gSpCowlClosedCmdRef = NULL;
XPLMCommandRef gSpCowlOpenCmdRef = NULL;
XPLMCommandRef gSpLightsPanelOnCmdRef = NULL;
XPLMCommandRef gSpLightsPanelOffCmdRef = NULL;
XPLMCommandRef gSpLightsBeaconOnCmdRef = NULL;
XPLMCommandRef gSpLightsBeaconOffCmdRef = NULL;
XPLMCommandRef gSpLightsNavOnCmdRef = NULL;
XPLMCommandRef gSpLightsNavOffCmdRef = NULL;
XPLMCommandRef gSpLightsStrobeOnCmdRef = NULL;
XPLMCommandRef gSpLightsStrobeOffCmdRef = NULL;
XPLMCommandRef gSpLightsTaxiOnCmdRef = NULL;
XPLMCommandRef gSpLightsTaxiOffCmdRef = NULL;
XPLMCommandRef gSpLightsLandingOnCmdRef = NULL;
XPLMCommandRef gSpLightsLandingOffCmdRef = NULL;
XPLMCommandRef gSpLandingGearUpCmdRef = NULL;
XPLMCommandRef gSpLandingGearDownCmdRef = NULL;

/* SWITCH PANEL Data Refs */
XPLMDataRef gSpMagnetosOffDataRef = NULL;
XPLMDataRef gSpMagnetosRightDataRef = NULL;
XPLMDataRef gSpMagnetosLeftDataRef = NULL;
XPLMDataRef gSpMagnetosBothDataRef = NULL;
XPLMDataRef gSpMagnetosStartDataRef = NULL;
XPLMDataRef gSpMasterBatteryOnDataRef = NULL;
XPLMDataRef gSpMasterBatteryOffDataRef = NULL;
XPLMDataRef gSpMasterAltBatteryOnDataRef = NULL;
XPLMDataRef gSpMasterAltBatteryOffDataRef = NULL;
XPLMDataRef gSpMasterAvionicsOnDataRef = NULL;
XPLMDataRef gSpMasterAvionicsOffDataRef = NULL;
XPLMDataRef gSpFuelPumpOnDataRef = NULL;
XPLMDataRef gSpFuelPumpOffDataRef = NULL;
XPLMDataRef gSpDeIceOnDataRef = NULL;
XPLMDataRef gSpDeIceOffDataRef = NULL;
XPLMDataRef gSpPitotHeatOnDataRef = NULL;
XPLMDataRef gSpPitotHeatOffDataRef = NULL;
XPLMDataRef gSpCowlClosedDataRef = NULL;
XPLMDataRef gSpCowlOpenDataRef = NULL;
XPLMDataRef gSpLightsBeaconOffDataRef = NULL;
XPLMDataRef gSpLightsPanelOffDataRef = NULL;
XPLMDataRef gSpLightsBeaconOnDataRef = NULL;
XPLMDataRef gSpLightsNavOnOnDataRef = NULL;
XPLMDataRef gSpLightsNavOffDataRef = NULL;
XPLMDataRef gSpLightsStrobeOnDataRef = NULL;
XPLMDataRef gSpLightsStrobeOffDataRef = NULL;
XPLMDataRef gSpLightsTaxiOnDataRef = NULL;
XPLMDataRef gSpLightsTaxiOffDataRef = NULL;
XPLMDataRef gSpLightsLandingOnDataRef = NULL;
XPLMDataRef gSpLightsLandingOffDataRef = NULL;
XPLMDataRef gSpLandingGearUpDataRef = NULL;
XPLMDataRef gSpLandingGearDownDataRef = NULL;

// Radio Panel resources
jobqueue    gRp_ojq;
jobqueue    gRp_ijq;
jobqueue    gRp_sjq;

// Multi Panel resources
jobqueue    gMp_ijq;
jobqueue    gMp_ojq;
jobqueue    gMp_sjq;

// Switch Panel resources
jobqueue    gSp_ijq;
jobqueue    gSp_ojq;
jobqueue    gSp_sjq;

// Multi Panel command and data refs
// command refs
#define sMP_ALTITUDE_DOWN              "sim/autopilot/altitude_down"
#define sMP_ALTITUDE_UP                "sim/autopilot/altitude_up"
#define sMP_VERTICAL_SPEED_DOWN        "sim/autopilot/vertical_speed_down"
#define sMP_VERTICAL_SPEED_UP          "sim/autopilot/vertical_speed_up"
#define sMP_AIRSPEED_DOWN              "sim/autopilot/airspeed_down"
#define sMP_AIRSPEED_UP                "sim/autopilot/airspeed_up"
#define sMP_HEADING_DOWN               "sim/autopilot/heading_down"
#define sMP_HEADING_UP                 "sim/autopilot/heading_up"
#define sMP_OBS_HSI_DOWN               "sim/radios/obs_HSI_down"
#define sMP_OBS_HSI_UP                 "sim/radios/obs_HSI_up"
#define sMP_FLIGHT_DIR_ON_ONLY         "sim/autopilot/flight_dir_on_only"
#define sMP_FDIR_SERVOS_TOGGLE         "sim/autopilot/fdir_servos_toggle"
#define sMP_SERVOS_AND_FLIGHT_DIR_ON   "sim/autopilot/servos_and_flight_dir_on"
#define sMP_SERVOS_AND_FLIGHT_DIR_OFF  "sim/autopilot/servos_and_flight_dir_off"
#define sMP_HEADING                    "sim/autopilot/heading"
#define sMP_NAV_ARM                    "sim/autopilot/NAV"
#define sMP_LEVEL_CHANGE               "sim/autopilot/level_change"
#define sMP_ALTITUDE_HOLD              "sim/autopilot/altitude_hold"
#define sMP_ALTITUDE_ARM               "sim/autopilot/altitude_arm"
#define sMP_VERTICAL_SPEED             "sim/autopilot/vertical_speed"
#define sMP_APPROACH                   "sim/autopilot/approach"
#define sMP_BACK_COURSE                "sim/autopilot/back_course"
#define sMP_AUTOTHROTTLE_ON            "sim/autopilot/autothrottle_on"
#define sMP_AUTOTHROTTLE_OFF           "sim/autopilot/autothrottle_off"
#define sMP_AUTOTHROTTLE_TOGGLE        "sim/autopilot/autothrottle_toggle"
#define sMP_FLAPS_DOWN                 "sim/flight_controls/flaps_down"
#define sMP_FLAPS_UP                   "sim/flight_controls/flaps_up"
#define sMP_PITCH_TRIM_DOWN            "sim/flight_controls/pitch_trim_down"
#define sMP_PITCH_TRIM_UP              "sim/flight_controls/pitch_trim_up"
#define sMP_PITCH_TRIM_TAKEOFF         "sim/flight_controls/pitch_trim_takeoff"

// data refs
#define sAVIONICS_POWER_ON              "sim/cockpit2/switches/avionics_power_on"
#define sBATTERY_ON                     "sim/cockpit/electrical/battery_on"
#define sMP_AUTOPILOT_ON               "sim/cockpit2/autopilot/autopilot_on"
#define sMP_FLIGHT_DIRECTOR_MODE       "sim/cockpit2/autopilot/flight_director_mode"

#define sMP_ALTITUDE_HOLD_FT           "sim/cockpit2/autopilot/altitude_hold_ft"
#define sMP_VVI_DIAL_FPM               "sim/cockpit2/autopilot/vvi_dial_fpm"
#define sMP_AIRSPEED                   "sim/cockpit/autopilot/airspeed"
#define sMP_HEADING_DIAL_DEG_MAG_PILOT "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"
#define sMP_HSI_OBS_DEG_MAG_PILOT      "sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot"

//#define sMP_ALTITUDE_DIAL_FT           "sim/cockpit2/autopilot/altitude_dial_ft"
//#define sMP_AIRSPEED_DIAL_KTS_MACH     "sim/cockpit2/autopilot/airspeed_dial_kts_mach"
//#define sMP_VERTICAL_VELOCITY          "sim/cockpit/autopilot/vertical_velocity"
#define sMP_ALTITUDE                   "sim/cockpit/autopilot/altitude"
#define sMP_HEADING_MAG                "sim/cockpit/autopilot/heading_mag"


#define sMP_HEADING_STATUS             "sim/cockpit2/autopilot/heading_status"
#define sMP_NAV_STATUS                 "sim/cockpit2/autopilot/nav_status"
#define sMP_SPEED_STATUS               "sim/cockpit2/autopilot/speed_status"
#define sMP_ALTITUDE_HOLD_STATUS       "sim/cockpit2/autopilot/altitude_hold_status"
#define sMP_VVI_STATUS                 "sim/cockpit2/autopilot/vvi_status"
#define sMP_APPROACH_STATUS            "sim/cockpit2/autopilot/approach_status"
#define sMP_BACKCOURSE_STATUS          "sim/cockpit2/autopilot/backcourse_status"


/* SWITCH PANEL */
#define sMAGNETOS_OFF               "sim/magnetos/magnetos_off"
#define sMAGNETOS_RIGHT             "sim/magnetos/magnetos_right_1"
#define sMAGNETOS_LEFT              "sim/magnetos/magnetos_left_1"
#define sMAGNETOS_BOTH              "sim/magnetos/magnetos_both"
#define sMAGNETOS_START             "sim/starters/engage_start_run"

#define sMASTER_BATTERY_ON          "sim/electrical/battery_1_on"
#define sMASTER_BATTERY_OFF         "sim/electrical/battery_1_off"
#define sMASTER_ALT_BATTERY_ON      "sim/electrical/battery_2_on"
#define sMASTER_ALT_BATTERY_OFF     "sim/electrical/battery_2_off"

#define sMASTER_AVIONICS_ON         "sim/systems/avionics_on"
#define sMASTER_AVIONICS_OFF        "sim/systems/avionics_off"
#define sFUEL_PUMP_ON               "sim/fuel/fuel_pump_1_on"
#define sFUEL_PUMP_OFF              "sim/fuel/fuel_pump_1_off"
#define sDE_ICE_ON                  "sim/ice/detect_on"
#define sDE_ICE_OFF                 "sim/ice/detect_off"
#define sPITOT_HEAT_ON              "sim/ice/pitot_heat_on"
#define sPITOT_HEAT_OFF             "sim/ice/pitot_heat_off"

#define sCOWL_CLOSED                "sim/flight_controls/cowl_flaps_closed"
#define sCOWL_OPEN                  "sim/flight_controls/cowl_flaps_open"

#define sLIGHTS_PANEL_ON            "sim/instruments/panel_bright_down"
#define sLIGHTS_PANEL_OFF           "sim/instruments/panel_bright_up"
#define sLIGHTS_BEACON_ON           "sim/lights/beacon_lights_on"
#define sLIGHTS_BEACON_OFF          "sim/lights/beacon_lights_off"
#define sLIGHTS_NAV_ON              "sim/lights/nav_lights_on"
#define sLIGHTS_NAV_OFF             "sim/lights/nav_lights_off"
#define sLIGHTS_STROBE_ON           "sim/lights/strobe_lights_on"
#define sLIGHTS_STROBE_OFF          "sim/lights/strobe_lights_off"
#define sLIGHTS_TAXI_ON             "sim/lights/taxi_lights_on"
#define sLIGHTS_TAXI_OFF            "sim/lights/taxi_lights_off"
#define sLIGHTS_LANDING_ON          "sim/lights/landing_lights_on"
#define sLIGHTS_LANDING_OFF         "sim/lights/landing_lights_off"

#define sLANDING_GEAR_UP            "sim/flight_controls/landing_gear_down"
#define sLANDING_GEAR_DOWN          "sim/flight_controls/landing_gear_up"


/*
 * - register the plugin
 * - check for hid connected panels
 * - register callbacks
 * - start threads
 *
 */
PLUGIN_API int
XPluginStart(char* outName, char* outSig, char* outDesc) {
    XPLMCommandRef cmd_ref;

    LPRINTF("Saitek ProPanels Plugin: XPluginStart\n");
    strcpy(outName, "SaitekProPanels");
    strcpy(outSig , "jdp.panels.saitek");
    strcpy(outDesc, "Saitek Pro Panels Plugin.");

    gAvPwrOnDataRef = XPLMFindDataRef(sAVIONICS_POWER_ON);
    gBatPwrOnDataRef = XPLMFindDataRef(sBATTERY_ON);

    // A questionable way to use the (evil) preprocessor
    // but it makes it easier to work on this file.
    #include "multipanel_refs.cpp"
    #include "switchpanel_refs.cpp"
    #include "radiopanel_refs.cpp"

    LPRINTF("Saitek ProPanels Plugin: commands initialized\n");

    if (init_hid(&gRpHandle, RP_PROD_ID))
        rp_init(gRpHandle);

    if (init_hid(&gMpHandle, MP_PROD_ID))
        mp_init(gMpHandle);

    if (init_hid(&gSpHandle, SP_PROD_ID))
        sp_init(gSpHandle);

    pexchange((int*)&threads_run, true);

    ToPanelThread*     tp;
    FromPanelThread*   fp;

    // radio panel
    tp = new ToPanelThread(gRpHandle, &gRp_ojq, &gRpTrigger, RP_PROD_ID);
    fp = new FromPanelThread(gRpHandle, &gRp_ijq, &gRp_ojq, &gRpTrigger, RP_PROD_ID);

    tp->start();
    fp->start();

    // multi panel
    tp = new ToPanelThread(gMpHandle, &gMp_ojq, &gMpTrigger, MP_PROD_ID);
    fp = new FromPanelThread(gMpHandle, &gMp_ijq, &gMp_ojq, &gMpTrigger, MP_PROD_ID);

    tp->start();
    fp->start();

    // switch panel
    tp = new ToPanelThread(gSpHandle, &gSp_ojq, &gSpTrigger, SP_PROD_ID);
    fp = new FromPanelThread(gSpHandle, &gSp_ijq, &gSp_ojq, &gSpTrigger, SP_PROD_ID);

    tp->start();
    fp->start();

#ifdef DO_USBPANEL_CHECK
    pexchange((int*)&pc_run, true);
    PanelsCheckThread* pc = new PanelsCheckThread();
    pc->start();
#endif

    if (gRpHandle) { LPRINTF("Saitek ProPanels Plugin: gRpHandle\n"); gRpTrigger.post(); }
    if (gMpHandle) { LPRINTF("Saitek ProPanels Plugin: gMpHandle\n"); gMpTrigger.post(); }
    if (gSpHandle) { LPRINTF("Saitek ProPanels Plugin: gSpHandle\n"); gSpTrigger.post(); }

    LPRINTF("Saitek ProPanels Plugin: Panel threads running\n");

    XPLMRegisterFlightLoopCallback(RadioPanelFlightLoopCallback, FL_CB_INTERVAL, NULL);
    XPLMRegisterFlightLoopCallback(MultiPanelFlightLoopCallback, FL_CB_INTERVAL, NULL);
    XPLMRegisterFlightLoopCallback(SwitchPanelFlightLoopCallback, FL_CB_INTERVAL, NULL);

//XPLM_API void XPLMSetErrorCallback(XPLMError_f inCallback);
//XPLM_API void XPLMGetVersions(int* outXPlaneVersion, int* outXPLMVersion, XPLMHostApplicationID* outHostID);

    LPRINTF("Saitek ProPanels Plugin: startup completed\n");

    uint32_t* x = new uint32_t;
    *x = MP_BLANK_SCRN;
    gMp_ojq.post(new myjob(x));

    mp_do_init();

    return 1;
}


/*
 *
 *
 */
int MultiPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void*             inRefcon) {
    int32_t t = 0;
    uint32_t x = 0;
    float f = 0.0;
    uint32_t* m = NULL;
    int status = MP_CMD_PASS_EVENT;

    switch (reinterpret_cast<uint32_t>(inRefcon)) {
    case MP_CMD_FLAPS_UP:
        pexchange((int*)&t, gMpFlapsUpPending);
        if (t > 0) {
            pdecrement(&gMpFlapsUpPending);
            status = MP_CMD_PASS_EVENT;
        } else {
            status = MP_CMD_EAT_EVENT;
        }
        break;
    case MP_CMD_FLAPS_DOWN:
        pexchange((int*)&t, gMpFlapsDnPending);
        if (t > 0) {
            pdecrement(&gMpFlapsDnPending);
            status = MP_CMD_PASS_EVENT;
        } else {
            status = MP_CMD_EAT_EVENT;
        }
        break;
//    case MP_CMD_PITCHTRIM_UP:
//        pexchange((int*)&t, gMpPitchTrimUpPending);
//        if (t > 0) {
//            pdecrement(&gMpPitchTrimUpPending);
//            status = MP_CMD_PASS_EVENT;
//        } else {
//            status = MP_CMD_EAT_EVENT;
//        }
//        break;
//    case MP_CMD_PITCHTRIM_DOWN:
//        pexchange((int*)&t, gMpPitchTrimDnPending);
//        if (t > 0) {
//            pdecrement(&gMpPitchTrimDnPending);
//            status = MP_CMD_PASS_EVENT;
//        } else {
//            status = MP_CMD_EAT_EVENT;
//        }
//        break;
    case MP_CMD_OTTO_AUTOTHROTTLE_ON:
        pexchange((int*)&t, gMpAutothrottle_onPending);
        if (t > 0) {
            pdecrement(&gMpAutothrottle_onPending);
            status = MP_CMD_PASS_EVENT;
        } else {
            status = MP_CMD_EAT_EVENT;
        }
        break;
    case MP_CMD_OTTO_AUTOTHROTTLE_OFF:
        pexchange((int*)&t, gMpAutothrottle_offPending);
        if (t > 0) {
            pdecrement(&gMpAutothrottle_offPending);
            status = MP_CMD_PASS_EVENT;
        } else {
            status = MP_CMD_EAT_EVENT;
        }
        break;
    case CMD_SYS_AVIONICS_ON:
        pexchange((int*)&gAvPwrOn, true);
        m = new uint32_t;
        *m = AVIONICS_ON;
        gMp_ojq.post(new myjob(m));
        break;
    case CMD_SYS_AVIONICS_OFF:
        pexchange((int*)&gAvPwrOn, false);
        m = new uint32_t;
        *m = AVIONICS_OFF;
        gMp_ojq.post(new myjob(m));
        break;
    case CMD_ELEC_BATTERY1_ON:
        pexchange((int*)&gBat1On, true);
        m = new uint32_t;
        *m = BAT1_ON;
        gMp_ojq.post(new myjob(m));
        break;
    case CMD_ELEC_BATTERY1_OFF:
        pexchange((int*)&gBat1On, false);
        m = new uint32_t;
        *m = BAT1_OFF;
        gMp_ojq.post(new myjob(m));
        break;
    case MP_CMD_OTTO_ON:
        m = new uint32_t;
        *m = MP_BTN_AP_ON;
        gMp_ojq.post(new myjob(m));
        break;
    case MP_CMD_OTTO_OFF:
        m = new uint32_t;
        *m = MP_BTN_AP_OFF;
        gMp_ojq.post(new myjob(m));
        break;
    case MP_CMD_OTTO_ARMED:
        m = new uint32_t;
        *m = MP_BTN_AP_ARMED;
        gMp_ojq.post(new myjob(m));
        break;
    case MP_CMD_OTTO_ALT_UP:
    case MP_CMD_OTTO_ALT_DN:
//        pexchange((int*)&t, gMpAlt_Pending);
//        if (t > 0) {
//            pdecrement(&gMpAlt_Pending);
//        } else {
            m = new uint32_t[MP_MPM_CNT];
            m[0] = MP_MPM;
            m[1] = MP_ALT_VAL;
            m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpAltHoldFtDataRef));
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_VS_UP:
    case MP_CMD_OTTO_VS_DN:
//        pexchange((int*)&t, gMpVs_Pending);
//        if (t > 0) {
//            pdecrement(&gMpIas_Pending);
//        } else {
            m = new uint32_t[MP_MPM_CNT];
            f = XPLMGetDataf(gMpVrtVelDataRef);
            m[1] = (f < 0) ? MP_VS_VAL_NEG : MP_VS_VAL_POS;
            m[0] = MP_MPM;
            m[2] = static_cast<uint32_t>(fabs(f));
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_IAS_UP:
    case MP_CMD_OTTO_IAS_DN:
//        pexchange((int*)&t, gMpIas_Pending);
//        if (t > 0) {
//            pdecrement(&gMpIas_Pending);
//        } else {
            m = new uint32_t[MP_MPM_CNT];
            m[0] = MP_MPM;
            m[1] = MP_IAS_VAL;
            m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpArspdDataRef));
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_HDG_UP:
    case MP_CMD_OTTO_HDG_DN:
//        pexchange((int*)&t, gMpHdg_Pending);
//        if (t > 0) {
//            pdecrement(&gMpHdg_Pending);
//        } else {
            m = new uint32_t[MP_MPM_CNT];
            m[0] = MP_MPM;
            m[1] = MP_HDG_VAL;
            m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpHdgMagDataRef));
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_CRS_UP:
    case MP_CMD_OTTO_CRS_DN:
//        pexchange((int*)&t, gMpCrs_Pending);
//        if (t > 0) {
//            pdecrement(&gMpCrs_Pending);
//        } else {
            m = new uint32_t[MP_MPM_CNT];
            m[0] = MP_MPM;
            m[1] = MP_CRS_VAL;
            m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpHsiObsDegMagPltDataRef));
            gMp_ojq.post(new myjob(m));
//        }
        break;
    //--- Buttons
    case MP_CMD_OTTO_ALT_HOLD_BTN:
//        pexchange((int*)&t, gMpBtn_Alt_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Alt_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpAltHoldStatDataRef);
            *m = (x == 0) ? MP_BTN_ALT_OFF : ((x == 2) ? MP_BTN_ALT_CAPT : MP_BTN_ALT_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
//    case MP_CMD_OTTO_ALT_ARM_BTN:
//        pexchange((int*)&t, gMpBtn_Alt_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Alt_TogglePending);
//        } else {
//            m = new uint32_t;
//            x = (uint32_t)XPLMGetDatai(gMpAltArmStatDataRef);
//            *m = (x == 0) ? MP_BTN_ALT_OFF : ((x == 2) ? MP_BTN_ALT_CAPT : MP_BTN_ALT_ARMED);
//            gMp_ojq.post(new myjob(m));
//        }
//        break;
    case MP_CMD_OTTO_APR_BTN:
//        pexchange((int*)&t, gMpBtn_Apr_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Apr_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpApprchStatDataRef);
            *m = (x == 0) ? MP_BTN_APR_OFF : ((x == 2) ? MP_BTN_APR_CAPT : MP_BTN_APR_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_REV_BTN:
//        pexchange((int*)&t, gMpBtn_Rev_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Rev_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpBckCrsStatDataRef);
            *m = (x == 0) ? MP_BTN_REV_OFF : ((x == 2) ? MP_BTN_REV_CAPT : MP_BTN_REV_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_HDG_BTN:
//        pexchange((int*)&t, gMpBtn_Hdg_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Hdg_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpHdgStatDataRef);
            *m = (x == 0) ? MP_BTN_HDG_OFF : ((x == 2) ? MP_BTN_HDG_CAPT : MP_BTN_HDG_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_NAV_BTN:
//        pexchange((int*)&t, gMpBtn_Nav_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Nav_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpNavStatDataRef);
            *m = (x == 0) ? MP_BTN_NAV_OFF : ((x == 2) ? MP_BTN_NAV_CAPT : MP_BTN_NAV_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_IAS_BTN:
//        pexchange((int*)&t, gMpBtn_Ias_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Ias_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpSpdStatDataRef);
            *m = (x == 0) ? MP_BTN_IAS_OFF : ((x == 2) ? MP_BTN_IAS_CAPT : MP_BTN_IAS_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
    case MP_CMD_OTTO_VS_BTN:
//        pexchange((int*)&t, gMpBtn_Vs_TogglePending);
//        if (t > 0) {
//            pdecrement(&gMpBtn_Vs_TogglePending);
//        } else {
            m = new uint32_t;
            x = (uint32_t)XPLMGetDatai(gMpVviStatDataRef);
            *m = (x == 0) ? MP_BTN_VS_OFF : ((x == 2) ? MP_BTN_VS_CAPT : MP_BTN_VS_ARMED);
            gMp_ojq.post(new myjob(m));
//        }
        break;
    default:
        break;
    }

    return status;
}


/*
 *
 *
 */
int RadioPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void*             inRefcon) {
    int status = MP_CMD_PASS_EVENT;

    return status;
}


/*
 *
 *
 */
int SwitchPanelCommandHandler(XPLMCommandRef   inCommand,
                             XPLMCommandPhase  inPhase,
                             void*             inRefcon) {
    uint32_t* m;
//    uint32_t x;
//    float f;
    int status = MP_CMD_PASS_EVENT;
    LPRINTF("Saitek ProPanels Plugin: switch panel lights landing on\n");

    switch (reinterpret_cast<uint32_t>(inRefcon)) {
    case CMD_MAGNETOS_OFF:
        m = new uint32_t;
        *m = SP_MAGNETOS_OFF;
        //
        break;
    case CMD_MAGNETOS_RIGHT:
        m = new uint32_t;
        *m = SP_MAGNETOS_RIGHT;
        //
        break;
    case CMD_MAGNETOS_LEFT:
        m = new uint32_t;
        *m = SP_MAGNETOS_LEFT;
        //
        break;
    case CMD_MAGNETOS_BOTH:
        m = new uint32_t;
        *m = SP_MAGNETOS_BOTH;
        //
        break;
    case CMD_MAGNETOS_START:
        m = new uint32_t;
        *m = SP_MAGNETOS_START;
        //
        break;
    case CMD_MASTER_BATTERY_ON:
        m = new uint32_t;
        *m = SP_MASTER_BATTERY_ON;
        //
        break;
    case CMD_MASTER_BATTERY_OFF:
        m = new uint32_t;
        *m = SP_MASTER_BATTERY_OFF;
        //
        break;
    case CMD_MASTER_ALT_BATTERY_ON:
        m = new uint32_t;
        *m = SP_MASTER_ALT_BATTERY_ON;
        //
        break;
    case CMD_MASTER_ALT_BATTERY_OFF:
        m = new uint32_t;
        *m = SP_MASTER_ALT_BATTERY_OFF;
        //
        break;
    case CMD_MASTER_AVIONICS_ON:
        m = new uint32_t;
        *m = SP_MASTER_AVIONICS_ON;
        //
        break;
    case CMD_MASTER_AVIONICS_OFF:
        m = new uint32_t;
        *m = SP_MASTER_AVIONICS_OFF;
        //
        break;
    case CMD_FUEL_PUMP_ON:
        m = new uint32_t;
        *m = SP_FUEL_PUMP_ON;
        //
        break;
    case CMD_FUEL_PUMP_OFF:
        m = new uint32_t;
        *m = SP_FUEL_PUMP_OFF;
        //
        break;
    case CMD_DEICE_ON:
        m = new uint32_t;
        *m = SP_DEICE_ON;
        //
        break;
    case CMD_DEICE_OFF:
        m = new uint32_t;
        *m = SP_DEICE_OFF;
        //
        break;
    case CMD_PITOT_HEAT_ON:
        m = new uint32_t;
        *m = SP_PITOT_HEAT_ON;
        //
        break;
    case CMD_PITOT_HEAT_OFF:
        m = new uint32_t;
        *m = SP_PITOT_HEAT_OFF;
        //
        break;
    case CMD_COWL_CLOSED:
        m = new uint32_t;
        *m = SP_COWL_CLOSED;
        //
        break;
    case CMD_COWL_OPEN:
        m = new uint32_t;
        *m = SP_COWL_OPEN;
        //
        break;
    case CMD_LIGHTS_PANEL_ON:
        m = new uint32_t;
        *m = SP_LIGHTS_PANEL_ON;
        //
        break;
    case CMD_LIGHTS_PANEL_OFF:
        m = new uint32_t;
        *m = SP_LIGHTS_PANEL_OFF;
        //
        break;
    case CMD_LIGHTS_BEACON_ON:
        m = new uint32_t;
        *m = SP_LIGHTS_BEACON_ON;
        //
        break;
    case CMD_LIGHTS_BEACON_OFF:
        m = new uint32_t;
        *m = SP_LIGHTS_BEACON_OFF;
        //
        break;
    case CMD_LIGHTS_NAV_ON:
        m = new uint32_t;
        *m = SP_LIGHTS_NAV_ON;
        //
        break;
    case CMD_LIGHTS_NAV_OFF:
        m = new uint32_t;
        *m = SP_LIGHTS_NAV_OFF;
        //
        break;
    case CMD_LIGHTS_STROBE_ON:
        m = new uint32_t;
        *m = SP_LIGHTS_STROBE_ON;
        //
        break;
    case CMD_LIGHTS_STROBE_OFF:
        m = new uint32_t;
        *m = SP_LIGHTS_STROBE_OFF;
        //
        break;
    case CMD_LIGHTS_TAXI_ON:
        m = new uint32_t;
        *m = SP_LIGHTS_TAXI_ON;
        //
        break;
    case CMD_LIGHTS_TAXI_OFF:
        m = new uint32_t;
        *m = SP_LIGHTS_TAXI_OFF;
        //
        break;
    case CMD_LIGHTS_LANDING_ON:
        m = new uint32_t;
        *m = SP_LIGHTS_LANDING_ON;
        //
        break;
    case CMD_LIGHTS_LANDING_OFF:
        m = new uint32_t;
        *m = SP_LIGHTS_LANDING_OFF;
        //
        break;
    case CMD_GEAR_UP:
        m = new uint32_t;
        *m = SP_LANDING_GEAR_UP;
        //
        break;
    case CMD_GEAR_DOWN:
        m = new uint32_t;
        *m = SP_LANDING_GEAR_DOWN;
        //
        break;
    default:
        break;
    }

    return status;
}

/*
 *
 *
 */
float RadioPanelFlightLoopCallback(float   inElapsedSinceLastCall,
                                   float   inElapsedTimeSinceLastFlightLoop,
                                   int     inCounter,
                                   void*   inRefcon) {
// #ifndef NDEBUG
//     static char tmp[100];
// #endif

//    uint32_t x;
    int msg_cnt = gRp_MsgProc_Cnt;

//    if ((gFlCbCnt % PANEL_CHECK_INTERVAL) == 0) {
//        if ((bool)gPluginEnabled) {
//            gPcTrigger.post();
//        }
//    }

    while (msg_cnt--) {
        message* msg = gRp_ijq.getmessage(MSG_NOWAIT);

        if (!msg) {
            break;
        } else {

        } // if (msg)
        delete msg;
    } // while

    return 1.0;
}


/*
 *
 *
 */
float MultiPanelFlightLoopCallback(float   inElapsedSinceLastCall,
                                   float   inElapsedTimeSinceLastFlightLoop,
                                   int     inCounter,
                                   void*   inRefcon) {
// #ifndef NDEBUG
//     static char tmp[100];
// #endif

    uint32_t x;
    int msg_cnt = gMp_MsgProc_Cnt;

//    if ((gFlCbCnt % PANEL_CHECK_INTERVAL) == 0) {
//        if ((boolgPluginEnabled) {
//            gPcTrigger.post();
//        }
//    }

    while (msg_cnt--) {
        message* msg = gMp_ijq.getmessage(MSG_NOWAIT);
        if (msg) {
            //sprintf(tmp, "Saitek ProPanels Plugin: msg received  0x%0.8X \n", *(uint32_t*)((myjob*) msg)->buf);
            //LPRINTF(tmp);
            if (gAvPwrOn && gBat1On) {
                x = *((myjob*)msg)->buf;
                switch (x) {
                //--- pitch
                case MP_PITCHTRIM_UP:
//                    pincrement(&gMpPitchTrimUpPending);
                    XPLMCommandOnce(gMpPtchTrmUpCmdRef);
                    break;
                case MP_PITCHTRIM_DN:
//                    pincrement(&gMpPitchTrimDnPending);
                    XPLMCommandOnce(gMpPtchTrmDnCmdRef);
                    break;
                //--- flaps
                case MP_FLAPS_UP:
                    pincrement(&gMpFlapsUpPending);
                    XPLMCommandOnce(gMpFlpsUpCmdRef);
                    break;
                case MP_FLAPS_DN:
                    pincrement(&gMpFlapsDnPending);
                    XPLMCommandOnce(gMpFlpsDnCmdRef);
                    break;
                //--- autothrottle
                case MP_AUTOTHROTTLE_OFF:
                    pincrement(&gMpAutothrottle_offPending);
                    XPLMCommandOnce(gMpAtThrrtlOffCmdRef);
                    break;
                case MP_AUTOTHROTTLE_ON:
                    pincrement(&gMpAutothrottle_onPending);
                    XPLMCommandOnce(gMpAtThrrtlOnCmdRef);
                    break;
                //-- buttons
                case MP_BTN_AP_TOGGLE:
//                    pincrement(&gMpBtn_Ap_TogglePending);
                    XPLMCommandOnce(gMpApToggleCmdRef);
                    break;
                case MP_BTN_HDG_TOGGLE:
//                    pincrement(&gMpBtn_Hdg_TogglePending);
                    XPLMCommandOnce(gMpHdgCmdRef);
                    break;
                case MP_BTN_NAV_TOGGLE:
//                    pincrement(&gMpBtn_Nav_TogglePending);
                    XPLMCommandOnce(gMpNavArmCmdRef);
                    break;
                case MP_BTN_IAS_TOGGLE:
//                    pincrement(&gMpBtn_Ias_TogglePending);
                    XPLMCommandOnce(gMpLvlChngCmdRef);
                    break;
                case MP_BTN_ALT_TOGGLE:
//                    pincrement(&gMpBtn_Alt_TogglePending);
//                    XPLMCommandOnce(&gMpAltArmCmdRef);
                    XPLMCommandOnce(&gMpAltHoldCmdRef);
                    break;
                case MP_BTN_VS_TOGGLE:
//                    pincrement(&gMpBtn_Vs_TogglePending);
                    XPLMCommandOnce(gMpVrtclSpdCmdRef);
                    break;
                case MP_BTN_APR_TOGGLE:
//                    pincrement(&gMpBtn_Apr_TogglePending);
                    XPLMCommandOnce(gMpAppCmdRef);
                    break;
                case MP_BTN_REV_TOGGLE:
//                    pincrement(&gMpBtn_Rev_TogglePending);
                    XPLMCommandOnce(gMpBkCrsCmdRef);
                    break;
                //--- tuning
                case MP_ALT_UP:
//                    pincrement(&gMpAlt_Pending);
                    XPLMCommandOnce(gMpAltUpCmdRef);
                    break;
                case MP_ALT_DN:
//                    pincrement(&gMpAlt_Pending);
                    XPLMCommandOnce(gMpAltDnCmdRef);
                    break;
                case MP_VS_UP:
//                    pincrement(&gMpVs_Pending);
                    XPLMCommandOnce(gMpVrtclSpdUpCmdRef);
                    break;
                case MP_VS_DN:
//                    pincrement(&gMpVs_Pending);
                    XPLMCommandOnce(gMpVrtclSpdDnCmdRef);
                    break;
                case MP_IAS_UP:
//                    pincrement(&gMpIas_Pending);
                    XPLMCommandOnce(gMpAsUpCmdRef);
                    break;
                case MP_IAS_DN:
//                    pincrement(&gMpIas_Pending);
                    XPLMCommandOnce(gMpAsDnCmdRef);
                    break;
                case MP_HDG_UP:
//                    pincrement(&gMpHdg_Pending);
                    XPLMCommandOnce(gMpHdgUpCmdRef);
                    break;
                case MP_HDG_DN:
//                    pincrement(&gMpHdg_Pending);
                    XPLMCommandOnce(gMpHdgDnCmdRef);
                    break;
                case MP_CRS_UP:
//                    pincrement(&gMpCrs_Pending);
                    XPLMCommandOnce(gMpObsHsiUpCmdRef);
                    break;
                case MP_CRS_DN:
//                    pincrement(&gMpCrs_Pending);
                    XPLMCommandOnce(gMpObsHsiDnCmdRef);
                    break;
                default:
                    // DPRINTF("Saitek ProPanels Plugin: UNKNOWN MSG -------\n");
                    // TODO: log error
                    break;
                } // switch (x)
            } // if (gAvPwrOn && gBat1On)
        } // if (msg)
        delete msg;
    } // while

// sprintf(tmp, "Saitek ProPanels Plugin: msg received - 0x%0.8X \n", *(uint32_t*)((myjob*) msg)->buf);
// DPRINTF("Saitek ProPanels Plugin: msg received -------\n");
// DPRINTF(tmp);

//    gFlCbCnt++;

    return 1.0;
}


/*
 *
 *
 */
float SwitchPanelFlightLoopCallback(float   inElapsedSinceLastCall,
                                    float   inElapsedTimeSinceLastFlightLoop,
                                    int     inCounter,
                                    void*   inRefcon) {
// #ifndef NDEBUG
//     static char tmp[100];
// #endif
    uint32_t x;
    int msg_cnt = gSp_MsgProc_Cnt;

//    if ((gFlCbCnt % PANEL_CHECK_INTERVAL) == 0) {
//        if ((boolgPluginEnabled) {
//            gPcTrigger.post();
//        }
//    }

    while (msg_cnt--) {
        message* msg = gSp_ijq.getmessage(MSG_NOWAIT);
        if (msg) {
            x = *((myjob*)msg)->buf;

            switch (x) {
            case SP_MAGNETOS_OFF:
                XPLMCommandOnce(gSpMagnetosOffCmdRef);
                break;
            case SP_MAGNETOS_RIGHT:
                XPLMCommandOnce(gSpMagnetosRightCmdRef);
                break;
            case SP_MAGNETOS_LEFT:
                XPLMCommandOnce(gSpMagnetosLeftCmdRef);
                break;
            case SP_MAGNETOS_BOTH:
                XPLMCommandOnce(gSpMagnetosBothCmdRef);
                break;
            case SP_MAGNETOS_START:
                XPLMCommandOnce(gSpMagnetosStartCmdRef);
                break;
            case SP_MASTER_BATTERY_ON:
                XPLMCommandOnce(gSpMasterBatteryOnCmdRef);
                break;
            case SP_MASTER_BATTERY_OFF:
                XPLMCommandOnce(gSpMasterBatteryOffCmdRef);
                break;
            case SP_MASTER_ALT_BATTERY_ON:
                XPLMCommandOnce(gSpMasterAltBatteryOnCmdRef);
                break;
            case SP_MASTER_ALT_BATTERY_OFF:
                XPLMCommandOnce(gSpMasterAltBatteryOffCmdRef);
                break;
            case SP_MASTER_AVIONICS_ON:
                XPLMCommandOnce(gSpMasterAvionicsOnCmdRef);
                break;
            case SP_MASTER_AVIONICS_OFF:
                XPLMCommandOnce(gSpMasterAvionicsOffCmdRef);
                break;
            case SP_FUEL_PUMP_ON:
                XPLMCommandOnce(gSpFuelPumpOnCmdRef);
                break;
            case SP_FUEL_PUMP_OFF:
                XPLMCommandOnce(gSpFuelPumpOffCmdRef);
                break;
            case SP_DEICE_ON:
                XPLMCommandOnce(gSpDeIceOnCmdRef);
                break;
            case SP_DEICE_OFF:
                XPLMCommandOnce(gSpDeIceOffCmdRef);
                break;
            case SP_PITOT_HEAT_ON:
                XPLMCommandOnce(gSpPitotHeatOnCmdRef);
                break;
            case SP_PITOT_HEAT_OFF:
                XPLMCommandOnce(gSpPitotHeatOffCmdRef);
                break;
            case SP_COWL_CLOSED:
                XPLMCommandOnce(gSpCowlClosedCmdRef);
                break;
            case SP_COWL_OPEN:
                XPLMCommandOnce(gSpCowlOpenCmdRef);
                break;
            case SP_LIGHTS_PANEL_ON:
                XPLMCommandOnce(gSpLightsPanelOnCmdRef);
                break;
            case SP_LIGHTS_PANEL_OFF:
                XPLMCommandOnce(gSpLightsPanelOffCmdRef);
                break;
            case SP_LIGHTS_BEACON_ON:
                XPLMCommandOnce(gSpLightsBeaconOnCmdRef);
                break;
            case SP_LIGHTS_BEACON_OFF:
                XPLMCommandOnce(gSpLightsBeaconOffCmdRef);
                break;
            case SP_LIGHTS_NAV_ON:
                XPLMCommandOnce(gSpLightsNavOnCmdRef);
                break;
            case SP_LIGHTS_NAV_OFF:
                XPLMCommandOnce(gSpLightsNavOffCmdRef);
                break;
            case SP_LIGHTS_STROBE_ON:
                XPLMCommandOnce(gSpLightsStrobeOnCmdRef);
                break;
            case SP_LIGHTS_STROBE_OFF:
                XPLMCommandOnce(gSpLightsStrobeOffCmdRef);
                break;
            case SP_LIGHTS_TAXI_ON:
                XPLMCommandOnce(gSpLightsTaxiOnCmdRef);
                break;
            case SP_LIGHTS_TAXI_OFF:
                XPLMCommandOnce(gSpLightsTaxiOffCmdRef);
                break;
            case SP_LIGHTS_LANDING_ON:
                XPLMCommandOnce(gSpLightsLandingOnCmdRef);
                break;
            case SP_LIGHTS_LANDING_OFF:
                XPLMCommandOnce(gSpLightsLandingOffCmdRef);
                break;
            case SP_LANDING_GEAR_UP:
                XPLMCommandOnce(gSpLandingGearUpCmdRef);
                break;
            case SP_LANDING_GEAR_DOWN:
                XPLMCommandOnce(gSpLandingGearDownCmdRef);
                break;
           default:
                break;
            }
        } // if (msg)
        delete msg;
    } // while

    return 1.0;
}


/*
 *
 */
PLUGIN_API void
XPluginStop(void) {
    LPRINTF("Saitek ProPanels Plugin: XPluginStop\n");

//    uint32_t* x = new uint32_t;
//    *x = MP_BLANK_SCRN;
//    gMp_ojq.post(new myjob(x));
//    psleep(500);
/*
    uint32_t* x;

    x = new uint32_t;
    *x = EXITING_THREAD_LOOP;
    gRp_ojq.post(new myjob(x));

    x =  new uint32_t;
    *x = EXITING_THREAD_LOOP;
    gMp_ojq.post(new myjob(x));

    x = new uint32_t;
    *x = EXITING_THREAD_LOOP;
    gSp_ojq.post(new myjob(x));
*/

#ifdef DO_USBPANEL_CHECK
    pexchange((int*)&pc_run, false);
    gPcTrigger.post();
#endif

    pexchange((int*)&threads_run, false);

    if (gRpHandle) {
        gRpTrigger.post();
        close_hid(gRpHandle);
    }

    if (gMpHandle) {
        gMpTrigger.post();
        close_hid(gMpHandle);
    }

    if (gSpHandle) {
        gSpTrigger.post();
        close_hid(gSpHandle);
    }

    psleep(500);

    XPLMUnregisterFlightLoopCallback(RadioPanelFlightLoopCallback, NULL);
    XPLMUnregisterFlightLoopCallback(MultiPanelFlightLoopCallback, NULL);
    XPLMUnregisterFlightLoopCallback(SwitchPanelFlightLoopCallback, NULL);
}


/*
 *
 */
PLUGIN_API void
XPluginDisable(void) {
    LPRINTF("Saitek ProPanels Plugin: XPluginDisable\n");

    pexchange((int*)&gPluginEnabled, false);
    gRpTrigger.reset();
    gMpTrigger.reset();
    gSpTrigger.reset();

    // set any panel specific globals here
    if (gMpHandle) {
        XPLMSetDatai(gMpOttoOvrrde, false);
    }
    if (gSpHandle) {
    }
    if (gRpHandle) {
    }
}


/*
 *
 */
PLUGIN_API int
XPluginEnable(void) {
    LPRINTF("Saitek ProPanels Plugin: XPluginEnable\n");

    pexchange((int*)&gPluginEnabled, false);

    // set any panel specific globals here
    if (gMpHandle) {
        XPLMSetDatai(gMpOttoOvrrde, true);
    }
    if (gSpHandle) {
    }
    if (gRpHandle) {
    }

    gRpTrigger.post();
    gMpTrigger.post();
    gSpTrigger.post();

    return 1;
}


void mp_do_init() {
    float f;
    int32_t t;
    uint32_t* m;
    uint32_t* x;

    if (gPlaneLoaded) {
        // not the user's plane
        return;
    }
    pexchange((int*)&gPlaneLoaded, true); // always first
    x = new uint32_t;
//    if (XPLMGetDatai(gAvPwrOnDataRef)) {
        pexchange((int*)&gAvPwrOn, true);
        *x = AVIONICS_ON;
        gMp_ojq.post(new myjob(x));
//    } else {
//        pexchange((int*)&gAvPwrOn, false);
//        *x = AVIONICS_OFF;
//        gMp_ojq.post(new myjob(x));
//    }
    x = new uint32_t;
//    if (XPLMGetDatai(gBatPwrOnDataRef)) {
        pexchange((int*)&gBat1On, true);
        *x = BAT1_ON;
        gMp_ojq.post(new myjob(x));
//    } else {
//        pexchange((int*)&gBat1On, false);
//        *x = BAT1_OFF;
//        gMp_ojq.post(new myjob(x));
//    }
    // ALT val init
    m = new uint32_t[MP_MPM_CNT];
    m[0] = MP_MPM;
    m[1] = MP_ALT_VAL;
    m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpAltHoldFtDataRef));
    gMp_ojq.post(new myjob(m));
    // VS val init
    m = new uint32_t[MP_MPM_CNT];
    f = XPLMGetDataf(gMpVrtVelDataRef);
    m[1] = (f < 0) ? MP_VS_VAL_NEG : MP_VS_VAL_POS;
    m[0] = MP_MPM;
    m[2] = static_cast<uint32_t>(fabs(f));
    gMp_ojq.post(new myjob(m));
    // IAS val init
    m = new uint32_t[MP_MPM_CNT];
    m[0] = MP_MPM;
    m[1] = MP_IAS_VAL;
    m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpArspdDataRef));
    gMp_ojq.post(new myjob(m));
    // HDG val init
    m = new uint32_t[MP_MPM_CNT];
    m[0] = MP_MPM;
    m[1] = MP_HDG_VAL;
    m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpHdgMagDataRef));
    gMp_ojq.post(new myjob(m));
    // CRS val init
    x = new uint32_t;
    m[0] = MP_MPM;
    m[1] = MP_CRS_VAL;
    m[2] = static_cast<uint32_t>(XPLMGetDataf(gMpHsiObsDegMagPltDataRef));
    gMp_ojq.post(new myjob(m));
    //--- buttons
    // AP button
    x = new uint32_t;
    t = XPLMGetDatai(gMpFlghtDirModeDataRef);
    *x = (t == 0) ? MP_BTN_AP_OFF : ((t == 2) ? MP_BTN_AP_ON : MP_BTN_AP_ARMED);
//    t = XPLMGetDatai(gMpApOnDataRef);
//    *x = (t == 0) ? MP_BTN_AP_OFF : MP_BTN_AP_ON;
    gMp_ojq.post(new myjob(x));
    // ALT button
    x = new uint32_t;
    t = XPLMGetDatai(gMpAltHoldStatDataRef);
    *x = (t == 0) ? MP_BTN_ALT_OFF : ((t == 2) ? MP_BTN_ALT_CAPT : MP_BTN_ALT_ARMED);
    gMp_ojq.post(new myjob(x));
    // APR button
    x = new uint32_t;
    t = XPLMGetDatai(gMpApprchStatDataRef);
    *x = (t == 0) ? MP_BTN_APR_OFF : ((t == 2) ? MP_BTN_APR_CAPT : MP_BTN_APR_ARMED);
    gMp_ojq.post(new myjob(x));
    // REV button
    x = new uint32_t;
    t = XPLMGetDatai(gMpBckCrsStatDataRef);
    *x = (t == 0) ? MP_BTN_REV_OFF : ((t == 2) ? MP_BTN_REV_CAPT : MP_BTN_REV_ARMED);
    gMp_ojq.post(new myjob(x));
    // HDG button
    x = new uint32_t;
    t = XPLMGetDatai(gMpHdgStatDataRef);
    *x = (t == 0) ? MP_BTN_HDG_OFF : ((t == 2) ? MP_BTN_HDG_CAPT : MP_BTN_HDG_ARMED);
    gMp_ojq.post(new myjob(x));
    // NAV button
    x = new uint32_t;
    t = XPLMGetDatai(gMpNavStatDataRef);
    *x = (t == 0) ? MP_BTN_NAV_OFF : ((t == 2) ? MP_BTN_NAV_CAPT : MP_BTN_NAV_ARMED);
    gMp_ojq.post(new myjob(x));
    // IAS button
    x = new uint32_t;
    t = XPLMGetDatai(gMpSpdStatDataRef);
    *x = (t == 0) ? MP_BTN_IAS_OFF : ((t == 2) ? MP_BTN_IAS_CAPT : MP_BTN_IAS_ARMED);
    gMp_ojq.post(new myjob(x));
    // VS button
    x = new uint32_t;
    t = XPLMGetDatai(gMpVviStatDataRef);
    *x = (t == 0) ? MP_BTN_VS_OFF : ((t == 2) ? MP_BTN_VS_CAPT : MP_BTN_VS_ARMED);
    gMp_ojq.post(new myjob(x));
}


/*
 *
 */
PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID inFrom, long inMsg, void* inParam) {
//    LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage\n");

    uint32_t* x;
    if (inFrom == XPLM_PLUGIN_XPLANE) {
    int inparam = reinterpret_cast<int>(inParam);
        switch (inMsg) {
        case XPLM_MSG_PLANE_LOADED:
            if (inparam != 0 || gPlaneLoaded) {
                // not the user's plane
                break;
            }
            mp_do_init();
            LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage XPLM_MSG_PLANE_LOADED\n");
            break;
        case XPLM_MSG_AIRPORT_LOADED:
            LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage XPLM_MSG_AIRPORT_LOADED\n");
            break;
        case XPLM_MSG_SCENERY_LOADED:
            LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage XPLM_MSG_SCENERY_LOADED\n");
            break;
        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
            LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage XPLM_MSG_AIRPLANE_COUNT_CHANGED\n");
            break;
// XXX: what's different between an unloaded and crashed plane
// as far as system state and procedure?
        case XPLM_MSG_PLANE_CRASHED:
            if ((int)inParam != 0) {
                // not the user's plane
                break;
            }
            x = new uint32_t;
            *x = MP_PLANE_CRASH;
            gMp_ojq.post(new myjob(x));
            LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage XPLM_MSG_PLANE_CRASHED\n");
            break;
        case XPLM_MSG_PLANE_UNLOADED:
            if ((int)inParam != 0) {
                // not the user's plane
                break;
            }
            x = new uint32_t;
            if ((bool)XPLMGetDatai(gAvPwrOnDataRef)) {
                pexchange((int*)&gAvPwrOn, false);
                *x = AVIONICS_OFF;
                gMp_ojq.post(new myjob(x));
            } else {
                pexchange((int*)&gAvPwrOn, true);
                *x = AVIONICS_ON;
                gMp_ojq.post(new myjob(x));
            }
            x = new uint32_t;
            if ((bool)XPLMGetDatai(gBatPwrOnDataRef)) {
                pexchange((int*)&gBat1On, false);
                *x = BAT1_OFF;
                gMp_ojq.post(new myjob(x));
            } else {
                pexchange((int*)&gBat1On, true);
                *x = BAT1_ON;
                gMp_ojq.post(new myjob(x));
            }
            pexchange((int*)&gPlaneLoaded, false); // always last
            LPRINTF("Saitek ProPanels Plugin: XPluginReceiveMessage XPLM_MSG_PLANE_UNLOADED\n");
            break;
        default:
            // unknown
            break;
        } // switch (inMsg)
    } // if (inFrom == XPLM_PLUGIN_XPLANE)
}
