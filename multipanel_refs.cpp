// Copyright (c) 2010 Joseph D Poirier
// Distributable under the terms of The New BSD License
// that can be found in the LICENSE file.

    /*  MultiPanel command and data refs file.
     *
     *  The code in this file is included in SaitekProPanels.cpp via the
     *  preprocessor, therefore, it isn't compilable and doesn't need to be
     *  included in the build file list.
     */

    /*----- MultiPanel Command Ref assignment -----*/

    /* readouts */
    gMpAltDnCmdRef          = XPLMFindCommand(sMP_ALTITUDE_DOWN);
    gMpAltUpCmdRef          = XPLMFindCommand(sMP_ALTITUDE_UP);
    gMpVrtclSpdDnCmdRef     = XPLMFindCommand(sMP_VERTICAL_SPEED_DOWN);
    gMpVrtclSpdUpCmdRef     = XPLMFindCommand(sMP_VERTICAL_SPEED_UP);
    gMpAsDnCmdRef           = XPLMFindCommand(sMP_AIRSPEED_DOWN);
    gMpAsUpCmdRef           = XPLMFindCommand(sMP_AIRSPEED_UP);
    gMpHdgDnCmdRef          = XPLMFindCommand(sMP_HEADING_DOWN);
    gMpHdgUpCmdRef          = XPLMFindCommand(sMP_HEADING_UP);
    gMpObsHsiDnCmdRef       = XPLMFindCommand(sMP_OBS_HSI_DOWN);
    gMpObsHsiUpCmdRef       = XPLMFindCommand(sMP_OBS_HSI_UP);

    /* buttons */
    gMpApToggleCmdRef       = XPLMFindCommand(sMP_FDIR_SERVOS_TOGGLE);
    gMpApArmedCmdRef        = XPLMFindCommand(sMP_FLIGHT_DIR_ON_ONLY);
    gMpApOnCmdRef           = XPLMFindCommand(sMP_SERVOS_AND_FLIGHT_DIR_ON);
    gMpApOffCmdRef          = XPLMFindCommand(sMP_SERVOS_AND_FLIGHT_DIR_OFF);
    gMpHdgCmdRef            = XPLMFindCommand(sMP_HEADING);
    gMpNavArmCmdRef         = XPLMFindCommand(sMP_NAV_ARM);
    gMpLvlChngCmdRef        = XPLMFindCommand(sMP_LEVEL_CHANGE);
    gMpAltHoldCmdRef        = XPLMFindCommand(sMP_ALTITUDE_HOLD);
    gMpAltArmCmdRef         = XPLMFindCommand(sMP_ALTITUDE_ARM);
    gMpVrtclSpdCmdRef       = XPLMFindCommand(sMP_VERTICAL_SPEED);
    gMpAppCmdRef            = XPLMFindCommand(sMP_APPROACH);
    gMpBkCrsCmdRef          = XPLMFindCommand(sMP_BACK_COURSE);

    /* auto throttle switch */
    gMpAtThrrtlOnCmdRef     = XPLMFindCommand(sMP_AUTOTHROTTLE_ON);
    gMpAtThrrtlOffCmdRef    = XPLMFindCommand(sMP_AUTOTHROTTLE_OFF);
    gMpAtThrrtlTgglCmdRef   = XPLMFindCommand(sMP_AUTOTHROTTLE_TOGGLE);

    /* flap handle */
    gMpFlpsDnCmdRef         = XPLMFindCommand(sMP_FLAPS_DOWN);
    gMpFlpsUpCmdRef         = XPLMFindCommand(sMP_FLAPS_UP);

    /* pitch trim wheel */
    gMpPtchTrmDnCmdRef      = XPLMFindCommand(sMP_PITCH_TRIM_DOWN);
    gMpPtchTrmUpCmdRef      = XPLMFindCommand(sMP_PITCH_TRIM_UP);
    gMpPtchTrmTkOffCmdRef   = XPLMFindCommand(sMP_PITCH_TRIM_TAKEOFF);

    /*----- MultiPanel Data Ref assignment -----*/
    // 0: off, 1: on, 2: autopilot engaged
    gMpFlghtDirModeDataRef   = XPLMFindDataRef(sMP_FLIGHT_DIRECTOR_MODE);
    gMpAltHoldFtDataRef      = XPLMFindDataRef(sMP_ALTITUDE_HOLD_FT);
    gMpVrtVelDataRef        = XPLMFindDataRef(sMP_VVI_DIAL_FPM);
    gMpArspdDataRef          = XPLMFindDataRef(sMP_AIRSPEED);
    gMpHdgMagDataRef         = XPLMFindDataRef(sMP_HEADING_DIAL_DEG_MAG_PILOT);
    gMpHsiObsDegMagPltDataRef = XPLMFindDataRef(sMP_HSI_OBS_DEG_MAG_PILOT);
//    gMpAltDialFtDataRef      = XPLMFindDataRef(sMP_ALTITUDE_DIAL_FT);
//    gMpArspdDataRef          = XPLMFindDataRef(sMP_AIRSPEED_DIAL_KTS_MACH);
//    gMpVrtVelDataRef          = XPLMFindDataRef(sMP_VERTICAL_VELOCITY);
//    gMpHdgMagDataRef          = XPLMFindDataRef(sMP_HEADING_MAG);

    // 0 = off, 1 = armed, 2 = captured
    gMpApOnDataRef            = XPLMFindDataRef(sMP_AUTOPILOT_ON);
    gMpAltHoldStatDataRef     = XPLMFindDataRef(sMP_ALTITUDE_HOLD_STATUS);
    gMpApprchStatDataRef      = XPLMFindDataRef(sMP_APPROACH_STATUS);
    gMpBckCrsStatDataRef      = XPLMFindDataRef(sMP_BACKCOURSE_STATUS);
    gMpHdgStatDataRef         = XPLMFindDataRef(sMP_HEADING_STATUS);
    gMpNavStatDataRef         = XPLMFindDataRef(sMP_NAV_STATUS);
    gMpSpdStatDataRef         = XPLMFindDataRef(sMP_SPEED_STATUS);
    gMpVviStatDataRef         = XPLMFindDataRef(sMP_VVI_STATUS);

#if 0
    /*----- MultiPanel Command Handlers -----*/
    cmd_ref = XPLMCreateCommand(sAVIONICS_POWER_ON, "Avionics Power On");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)CMD_SYS_AVIONICS_ON);

    cmd_ref = XPLMCreateCommand(, "Avionics Off");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)CMD_SYS_AVIONICS_OFF);

    cmd_ref = XPLMCreateCommand(sBATTERY_ON, "Battery 1 On");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)CMD_ELEC_BATTERY1_ON);

    cmd_ref = XPLMCreateCommand(, "Battery 1 Off");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)CMD_ELEC_BATTERY1_OFF);
#endif

    //--- MultiPanel
    // auto throttle
    cmd_ref = XPLMCreateCommand(sMP_AUTOTHROTTLE_ON, "Auto Throttle On");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)MP_CMD_OTTO_AUTOTHROTTLE_ON);

    cmd_ref = XPLMCreateCommand(sMP_AUTOTHROTTLE_OFF, "Auto Throttle Off");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)MP_CMD_OTTO_AUTOTHROTTLE_OFF);

    // flaps
    cmd_ref = XPLMCreateCommand(sMP_FLAPS_UP, "Flaps Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)MP_CMD_FLTCTL_FLAPS_UP);

    cmd_ref = XPLMCreateCommand(sMP_FLAPS_DOWN, "Flaps Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)MP_CMD_FLTCTL_FLAPS_DOWN);

    // pitch trim
    cmd_ref = XPLMCreateCommand(sMP_PITCH_TRIM_UP, "Pitch Trim Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)MP_CMD_FLTCTL_PITCHTRIM_UP);

    cmd_ref = XPLMCreateCommand(sMP_PITCH_TRIM_DOWN, "Pitch Trim Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void*)MP_CMD_FLTCTL_PITCHTRIM_DOWN);

    // readouts
    cmd_ref = XPLMCreateCommand(sMP_ALTITUDE_DOWN, "AutoPilot ALT Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_ALT_DN);

    cmd_ref = XPLMCreateCommand(sMP_ALTITUDE_UP, "AutoPilot ALT Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_ALT_UP);

    cmd_ref = XPLMCreateCommand(sMP_VERTICAL_SPEED_DOWN, "AutoPilot VS Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_VS_DN);

    cmd_ref = XPLMCreateCommand(sMP_VERTICAL_SPEED_UP, "AutoPilot VS Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_VS_UP);

    cmd_ref = XPLMCreateCommand(sMP_AIRSPEED_DOWN, "AutoPilot IAS Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_IAS_DN);

    cmd_ref = XPLMCreateCommand(sMP_AIRSPEED_UP, "AutoPilot IAS Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_IAS_UP);

    cmd_ref = XPLMCreateCommand(sMP_HEADING_DOWN, "AutoPilot HDG Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_HDG_DN);

    cmd_ref = XPLMCreateCommand(sMP_HEADING_UP, "AutoPilot HDG Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_HDG_UP);

    cmd_ref = XPLMCreateCommand(sMP_OBS_HSI_DOWN, "AutoPilot CRS Down");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_CRS_DN);

    cmd_ref = XPLMCreateCommand(sMP_OBS_HSI_UP, "AutoPilot CRS Up");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_CRS_UP);

    // buttons
    cmd_ref = XPLMCreateCommand(sMP_FLIGHT_DIR_ON_ONLY, "AutoPilot Armed");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_ARMED);

    cmd_ref = XPLMCreateCommand(sMP_SERVOS_AND_FLIGHT_DIR_ON, "AutoPilot On");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_ON);

    cmd_ref = XPLMCreateCommand(sMP_SERVOS_AND_FLIGHT_DIR_OFF, "AutoPilot Off");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_OFF);

    cmd_ref = XPLMCreateCommand(sMP_NAV_ARM, "AutoPilot NAV");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_NAV_BTN);

    cmd_ref = XPLMCreateCommand(sMP_LEVEL_CHANGE, "AutoPilot IAS");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_IAS_BTN);

//    cmd_ref = XPLMCreateCommand(sMP_ALTITUDE_ARM, "AutoPilot ALT Arm");
//    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_ALT_ARM_BTN);

    cmd_ref = XPLMCreateCommand(sMP_ALTITUDE_HOLD, "AutoPilot ALT Hold");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_ALT_HOLD_BTN);

    cmd_ref = XPLMCreateCommand(sMP_VERTICAL_SPEED, "AutoPilot VS");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_VS_BTN);

    cmd_ref = XPLMCreateCommand(sMP_APPROACH, "AutoPilot APR");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_APR_BTN);

    cmd_ref = XPLMCreateCommand(sMP_BACK_COURSE, "AutoPilot REV");
    XPLMRegisterCommandHandler(cmd_ref, MultiPanelCommandHandler, CMD_HNDLR_EPILOG, (void*)MP_CMD_OTTO_REV_BTN);

