/*
 * Tuya device constants
 *
 * Includes source from:
 * - https://github.com/Koenkk/zigbee-herdsman-converters/blob/53ef9cb0b38afa71d4acf46d7c85c806f1ece382/src/lib/legacy.ts
 * - https://github.com/zigbeefordomoticz/wiki/blob/master/en-eng/Technical/Tuya-0xEF00.md
 *
 * Original license of that source is:
 *
 * MIT License
 * Copyright (c) 2018 Koen Kanters
 */
#ifndef LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_TUYA_H_
#define LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_TUYA_H_

#include <cstdint>

//               Ty payl
//      Frame DP pe size payload
//
// --------------------------
// Tuya 601 dimmer
// --------------------------
// off   0100 01 01 0001 00
// on    0100 01 01 0001 01
// m50   0100 02 02 0004 000001f6 // 502/1000??
// --------------------------
//
// Zenismart blinds
// --------------------------
// open  0300 01 04 0001 00
// close 0400 01 04 0001 02
// --------------------------

namespace libp::tuya {

constexpr uint16_t cluster_id = 0xef00;

enum class ZclCmd : uint8_t {
	data_request = 0x00
};

enum class OnOffState : uint8_t {
    off = 0,
    one = 1,
};

enum class WindowCoveringState : uint8_t {
    open  = 0,
    stop  = 1,
    close = 2,
};


enum class DataType : uint8_t {
    raw_t    = 0, // [ bytes ]
    bool_t   = 1, // [0/1]
    value_t  = 2, // [ 4 byte value ]
    string_t = 3, // [ N byte string ]
    enum_t   = 4, // [ 0-255 ]
    bitmap_t = 5, // [ 1,2,4 bytes ] as bits
    invalid = 0xff,
};


enum class DataPoint : uint8_t {
    invalid = 0,
    wateringTimer_valve_state_auto_shutdown = 2,
	wateringTimer_water_flow = 3,
	wateringTimer_shutdown_timer = 11,
	wateringTimer_remaining_watering_time = 101,
	wateringTimer_valve_state = 102,
	wateringTimer_last_watering_duration = 107,
	wateringTimer_battery = 110,
    // Common data points
    // Below data points are usually shared between devices
    state = 1,
    heatingSetpoint = 2,
    coverPosition = 2,
    dimmerLevel = 3,
    dimmerMinLevel = 3,
    localTemp = 3,
    coverArrived = 3,
    occupancy = 3,
    mode = 4,
    fanMode = 5,
    dimmerMaxLevel = 5,
    motorDirection = 5,
    config = 5,
    childLock = 7,
    coverChange = 7,
    runningState = 14,
    valveDetection = 20,
    battery = 21,
    tempCalibration = 44,
    // Data points above 100 are usually custom function data points
    waterLeak = 101,
    minTemp = 102,
    maxTemp = 103,
    windowDetection = 104,
    boostTime = 105,
    coverSpeed = 105,
    forceMode = 106,
    comfortTemp = 107,
    ecoTemp = 108,
    valvePos = 109,
    batteryLow = 110,
    weekFormat = 111,
    scheduleWorkday = 112,
    scheduleHoliday = 113,
    awayTemp = 114,
    windowOpen = 115,
    autoLock = 116,
    awayDays = 117,
    // Manufacturer specific
    // Earda
    eardaDimmerLevel = 2,
    // Siterwell Thermostat
    siterwellWindowDetection = 18,
    // Moes Thermostat
    moesHold = 2,
    moesScheduleEnable = 3,
    moesHeatingSetpoint = 16,
    moesMaxTempLimit = 18,
    moesMaxTemp = 19,
    moesDeadZoneTemp = 20,
    moesLocalTemp = 24,
    moesMinTempLimit = 26,
    moesTempCalibration = 27,
    moesValve = 36,
    moesChildLock = 40,
    moesSensor = 43,
    moesSchedule = 101,
    etopErrorStatus = 13,
    // MoesS Thermostat
    moesSsystemMode = 1,
    moesSheatingSetpoint = 2,
    moesSlocalTemp = 3,
    moesSboostHeating = 4,
    moesSboostHeatingCountdown = 5,
    moesSreset = 7,
    moesSwindowDetectionFunktion_A2 = 8,
    moesSwindowDetection = 9,
    moesSchildLock = 13,
    moesSbattery = 14,
    moesSschedule = 101,
    moesSvalvePosition = 104,
    moesSboostHeatingCountdownTimeSet = 103,
    moesScompensationTempSet = 105,
    moesSecoMode = 106,
    moesSecoModeTempSet = 107,
    moesSmaxTempSet = 108,
    moesSminTempSet = 109,
    moesCoverCalibration = 3,
    moesCoverBacklight = 7,
    moesCoverMotorReversal = 8,
    // Neo T&H
    neoOccupancy = 101,
    neoPowerType = 101,
    neoMelody = 102,
    neoDuration = 103,
    neoTamper = 103,
    neoAlarm = 104,
    neoTemp = 105,
    neoTempScale = 106,
    neoHumidity = 106,
    neoMinTemp = 107,
    neoMaxTemp = 108,
    neoMinHumidity = 109,
    neoMaxHumidity = 110,
    neoUnknown2 = 112,
    neoTempAlarm = 113,
    neoTempHumidityAlarm = 113,
    neoHumidityAlarm = 114,
    neoUnknown3 = 115,
    neoVolume = 116,
    // Neo AlarmOnly
    neoAOBattPerc = 15,
    neoAOMelody = 21,
    neoAODuration = 7,
    neoAOAlarm = 13,
    neoAOVolume = 5,
    // Saswell TRV
    saswellHeating = 3,
    saswellWindowDetection = 8,
    saswellFrostDetection = 10,
    saswellTempCalibration = 27,
    saswellChildLock = 40,
    saswellState = 101,
    saswellLocalTemp = 102,
    saswellHeatingSetpoint = 103,
    saswellValvePos = 104,
    saswellBatteryLow = 105,
    saswellAwayMode = 106,
    saswellScheduleMode = 107,
    saswellScheduleEnable = 108,
    saswellScheduleSet = 109,
    saswellSetpointHistoryDay = 110,
    saswellTimeSync = 111,
    saswellSetpointHistoryWeek = 112,
    saswellSetpointHistoryMonth = 113,
    saswellSetpointHistoryYear = 114,
    saswellLocalHistoryDay = 115,
    saswellLocalHistoryWeek = 116,
    saswellLocalHistoryMonth = 117,
    saswellLocalHistoryYear = 118,
    saswellMotorHistoryDay = 119,
    saswellMotorHistoryWeek = 120,
    saswellMotorHistoryMonth = 121,
    saswellMotorHistoryYear = 122,
    saswellScheduleSunday = 123,
    saswellScheduleMonday = 124,
    saswellScheduleTuesday = 125,
    saswellScheduleWednesday = 126,
    saswellScheduleThursday = 127,
    saswellScheduleFriday = 128,
    saswellScheduleSaturday = 129,
    saswellAntiScaling = 130,
    // HY thermostat
    hyHeating = 102,
    hyExternalTemp = 103,
    hyAwayDays = 104,
    hyAwayTemp = 105,
    hyMaxTempProtection = 106,
    hyMinTempProtection = 107,
    hyTempCalibration = 109,
    hyHysteresis = 110,
    hyProtectionHysteresis = 111,
    hyProtectionMaxTemp = 112,
    hyProtectionMinTemp = 113,
    hyMaxTemp = 114,
    hyMinTemp = 115,
    hySensor = 116,
    hyPowerOnBehavior = 117,
    hyWeekFormat = 118,
    hyWorkdaySchedule1 = 119,
    hyWorkdaySchedule2 = 120,
    hyHolidaySchedule1 = 121,
    hyHolidaySchedule2 = 122,
    hyState = 125,
    hyHeatingSetpoint = 126,
    hyLocalTemp = 127,
    hyMode = 128,
    hyChildLock = 129,
    hyAlarm = 130,
    // Silvercrest
    silvercrestChangeMode = 2,
    silvercrestSetBrightness = 3,
    silvercrestSetColorTemp = 4,
    silvercrestSetColor = 5,
    silvercrestSetEffect = 6,
    // Fantem
    fantemPowerSupplyMode = 101,
    fantemReportingTime = 102,
    fantemExtSwitchType = 103,
    fantemTempCalibration = 104,
    fantemHumidityCalibration = 105,
    fantemLoadDetectionMode = 105,
    fantemLuxCalibration = 106,
    fantemExtSwitchStatus = 106,
    fantemTemp = 107,
    fantemHumidity = 108,
    fantemMotionEnable = 109,
    fantemControlMode = 109,
    fantemBattery = 110,
    fantemLedEnable = 111,
    fantemReportingEnable = 112,
    fantemLoadType = 112,
    fantemLoadDimmable = 113,
    // Woox
    wooxSwitch = 102,
    wooxBattery = 14,
    wooxSmokeTest = 8,
    // FrankEver
    frankEverTimer = 9,
    frankEverTreshold = 101,
    // Dinrail power meter switch
    dinrailPowerMeterTotalEnergy = 17,
    dinrailPowerMeterCurrent = 18,
    dinrailPowerMeterPower = 19,
    dinrailPowerMeterVoltage = 20,
    dinrailPowerMeterTotalEnergy2 = 101,
    dinrailPowerMeterPower2 = 103,
    // tuya smart air box
    tuyaSabCO2 = 2,
    tuyaSabTemp = 18,
    tuyaSabHumidity = 19,
    tuyaSabVOC = 21,
    tuyaSabFormaldehyd = 22,
    // tuya Smart Air House Keeper, Multifunctionale air quality detector.
    // CO2, Temp, Humidity, VOC and Formaldehyd same as Smart Air Box
    tuyaSahkMP25 = 2,
    tuyaSahkCO2 = 22,
    tuyaSahkFormaldehyd = 20,
    // Tuya CO (carbon monoxide) smart air box
    tuyaSabCOalarm = 1,
    tuyaSabCO = 2,
    // Moes MS-105 Dimmer
    moes105DimmerState1 = 1,
    moes105DimmerLevel1 = 2,
    moes105DimmerState2 = 7,
    moes105DimmerLevel2 = 8,
    // Tuya Radar Sensor
    trsPresenceState = 1,
    trsSensitivity = 2,
    trsMotionState = 102,
    trsIlluminanceLux = 103,
    trsDetectionData = 104,
    trsScene = 112,
    trsMotionDirection = 114,
    trsMotionSpeed = 115,
    // Tuya Radar Sensor with fall function
    trsfPresenceState = 1,
    trsfSensitivity = 2,
    trsfMotionState = 102,
    trsfIlluminanceLux = 103,
    trsfTumbleSwitch = 105,
    trsfTumbleAlarmTime = 106,
    trsfScene = 112,
    trsfMotionDirection = 114,
    trsfMotionSpeed = 115,
    trsfFallDownStatus = 116,
    trsfStaticDwellAlarm = 117,
    trsfFallSensitivity = 118,
    // Human Presence Sensor AIR
    msVSensitivity = 101,
    msOSensitivity = 102,
    msVacancyDelay = 103,
    msMode = 104,
    msVacantConfirmTime = 105,
    msReferenceLuminance = 106,
    msLightOnLuminancePrefer = 107,
    msLightOffLuminancePrefer = 108,
    msLuminanceLevel = 109,
    msLedStatus = 110,
    // TV01 Moes Thermostat
    tvMode = 2,
    tvWindowDetection = 8,
    tvFrostDetection = 10,
    tvHeatingSetpoint = 16,
    tvLocalTemp = 24,
    tvTempCalibration = 27,
    tvWorkingDay = 31,
    tvHolidayTemp = 32,
    tvBattery = 35,
    tvChildLock = 40,
    tvErrorStatus = 45,
    tvHolidayMode = 46,
    tvBoostTime = 101,
    tvOpenWindowTemp = 102,
    tvComfortTemp = 104,
    tvEcoTemp = 105,
    tvWeekSchedule = 106,
    tvHeatingStop = 107,
    tvMondaySchedule = 108,
    tvWednesdaySchedule = 109,
    tvFridaySchedule = 110,
    tvSundaySchedule = 111,
    tvTuesdaySchedule = 112,
    tvThursdaySchedule = 113,
    tvSaturdaySchedule = 114,
    tvBoostMode = 115,
    // HOCH / WDYK DIN Rail
    hochCountdownTimer = 9,
    hochFaultCode = 26,
    hochRelayStatus = 27,
    hochChildLock = 29,
    hochVoltage = 101,
    hochCurrent = 102,
    hochActivePower = 103,
    hochLeakageCurrent = 104,
    hochTemperature = 105,
    hochRemainingEnergy = 106,
    hochRechargeEnergy = 107,
    hochCostParameters = 108,
    hochLeakageParameters = 109,
    hochVoltageThreshold = 110,
    hochCurrentThreshold = 111,
    hochTemperatureThreshold = 112,
    hochTotalActivePower = 113,
    hochEquipmentNumberType = 114,
    hochClearEnergy = 115,
    hochLocking = 116,
    hochTotalReverseActivePower = 117,
    hochHistoricalVoltage = 118,
    hochHistoricalCurrent = 119,
    // NOUS SMart LCD Temperature and Humidity Sensor E6
    nousTemperature = 1,
    nousHumidity = 2,
    nousBattery = 4,
    nousTempUnitConvert = 9,
    nousMaxTemp = 10,
    nousMinTemp = 11,
    nousMaxHumi = 12,
    nousMinHumi = 13,
    nousTempAlarm = 14,
    nousHumiAlarm = 15,
    nousHumiSensitivity = 20,
    nousTempSensitivity = 19,
    nousTempReportInterval = 17,
    nousHumiReportInterval = 18,
    // TUYA Temperature and Humidity Sensor
    tthTemperature = 1,
    tthHumidity = 2,
    tthBatteryLevel = 3,
    tthBattery = 4,
    // TUYA / HUMIDITY/ILLUMINANCE/TEMPERATURE SENSOR
    thitBatteryPercentage = 3,
    thitIlluminanceLux = 7,
    tIlluminanceLux = 2,
    thitHumidity = 9,
    thitTemperature = 8,
    // TUYA SMART VIBRATION SENSOR
    tuyaVibration = 10,
    // TUYA WLS-100z Water Leak Sensor
    wlsWaterLeak = 1,
    wlsBatteryPercentage = 4,
    // Evanell
    evanellMode = 2,
    evanellHeatingSetpoint = 4,
    evanellLocalTemp = 5,
    evanellBattery = 6,
    evanellChildLock = 8,
    // ZMAM02 Zemismart RF Courtain Converter
    AM02Control = 1,
    AM02PercentControl = 2,
    AM02PercentState = 3,
    AM02Mode = 4,
    AM02Direction = 5,
    AM02WorkState = 7,
    AM02CountdownLeft = 9,
    AM02TimeTotal = 10,
    AM02SituationSet = 11,
    AM02Fault = 12,
    AM02Border = 16,
    AM02MotorWorkingMode = 20,
    AM02AddRemoter = 101,
    // Matsee Tuya Garage Door Opener
    garageDoorTrigger = 1,
    garageDoorContact = 3,
    garageDoorStatus = 12,
    // Moes switch with optional neutral
    moesSwitchPowerOnBehavior = 14,
    moesSwitchIndicateLight = 15,
    // X5H thermostat
    x5hState = 1,
    x5hMode = 2,
    x5hWorkingStatus = 3,
    x5hSound = 7,
    x5hFrostProtection = 10,
    x5hSetTemp = 16,
    x5hSetTempCeiling = 19,
    x5hCurrentTemp = 24,
    x5hTempCorrection = 27,
    x5hWeeklyProcedure = 30,
    x5hWorkingDaySetting = 31,
    x5hFactoryReset = 39,
    x5hChildLock = 40,
    x5hSensorSelection = 43,
    x5hFaultAlarm = 45,
    x5hTempDiff = 101,
    x5hProtectionTempLimit = 102,
    x5hOutputReverse = 103,
    x5hBackplaneBrightness = 104,
    // Connected thermostat
    connecteState = 1,
    connecteMode = 2,
    connecteHeatingSetpoint = 16,
    connecteLocalTemp = 24,
    connecteTempCalibration = 28,
    connecteChildLock = 30,
    connecteTempFloor = 101,
    connecteSensorType = 102,
    connecteHysteresis = 103,
    connecteRunningState = 104,
    connecteTempProgram = 105,
    connecteOpenWindow = 106,
    connecteMaxProtectTemp = 107,
    // Tuya Smart Human Presence Sensor
    tshpsPresenceState = 1,
    tshpscSensitivity = 2,
    tshpsMinimumRange = 3,
    tshpsMaximumRange = 4,
    tshpsTargetDistance = 9,
    tshpsDetectionDelay = 101,
    tshpsFadingTime = 102,
    tshpsIlluminanceLux = 104,
    tshpsCLI = 103, // not recognize
    tshpsSelfTest = 6, // not recognize
    // Tuya Luminance Motion sensor
    lmsState = 1,
    lmsBattery = 4,
    lmsSensitivity = 9,
    lmsKeepTime = 10,
    lmsIlluminance = 12,
    // Alecto SMART-SMOKE10
    alectoSmokeState = 1,
    alectoSmokeValue = 2,
    alectoSelfChecking = 8,
    alectoCheckingResult = 9,
    alectoSmokeTest = 11,
    alectoLifecycle = 12,
    alectoBatteryState = 14,
    alectoBatteryPercentage = 15,
    alectoSilence = 16,
    // BAC-002-ALZB - Moes like thermostat with Fan control
    bacFanMode = 28,
    // Human Presence Sensor Zigbee Radiowave Tuya
    HPSZInductionState = 1,
    HPSZPresenceTime = 101,
    HPSZLeavingTime = 102,
    HPSZLEDState = 103,
    giexWaterValve_battery = 108,
	giexWaterValve_currentTemperature = 106,
	giexWaterValve_cycleIrrigationInterval = 105,
	giexWaterValve_cycleIrrigationNumTimes = 103,
	giexWaterValve_irrigationEndTime = 102,
	giexWaterValve_irrigationStartTime = 101,
	giexWaterValve_irrigationTarget = 104,
	giexWaterValve_lastIrrigationDuration = 114,
	giexWaterValve_mode = 1,
	giexWaterValve_state = 2,
	giexWaterValve_waterConsumed = 111,
    zsHeatingSetpoint = 16,
    zsChildLock = 40,
    zsTempCalibration = 104,
    zsLocalTemp = 24,
    zsBatteryVoltage = 35,
    zsComfortTemp = 101,
    zsEcoTemp = 102,
    zsHeatingSetpointAuto = 105,
    zsOpenwindowTemp = 116,
    zsOpenwindowTime = 117,
    zsErrorStatus = 45,
    zsMode = 2,
    zsAwaySetting = 103,
    zsBinaryOne = 106,
    zsBinaryTwo = 107,
    zsScheduleMonday = 109,
    zsScheduleTuesday = 110,
    zsScheduleWednesday = 111,
    zsScheduleThursday = 112,
    zsScheduleFriday = 113,
    zsScheduleSaturday = 114,
    zsScheduleSunday = 115,
};

} // namespace libp::tuya

#endif /* LIBPEKIN_NETWORK_ZIGBEE_ZCL_SCRIPT_TUYA_H_ */
