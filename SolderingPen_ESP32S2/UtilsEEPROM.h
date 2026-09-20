#ifndef UTILS_EEPROM_H
#define UTILS_EEPROM_H

#include <EEPROM.h>
#include <string.h>

#define ADDR_SYSTEM_INIT_FLAG (0)
#define ADDR_DEFAULT_TEMP     (ADDR_SYSTEM_INIT_FLAG + 4)
#define ADDR_SLEEP_TEMP       (ADDR_DEFAULT_TEMP + 2)
#define ADDR_BOOST_TEMP       (ADDR_SLEEP_TEMP + 2)
#define ADDR_TIME_2_SLEEP     (ADDR_BOOST_TEMP + 1)
#define ADDR_TIME_2_OFF       (ADDR_TIME_2_SLEEP + 2)
#define ADDR_TIME_OF_BOOST    (ADDR_TIME_2_OFF + 1)
#define ADDR_MAIN_SCREEN      (ADDR_TIME_OF_BOOST + 1)
#define ADDR_PID_ENABLE       (ADDR_MAIN_SCREEN + 1)
#define ADDR_BEEP_ENABLE      (ADDR_PID_ENABLE + 1)
#define ADDR_VOLTAGE_VALUE    (ADDR_BEEP_ENABLE + 1)
#define ADDR_QC_ENABLE        (ADDR_VOLTAGE_VALUE + 1)
#define ADDR_WAKEUP_THRESHOLD (ADDR_QC_ENABLE + 1)
#define ADDR_CURRENT_TIP      (ADDR_WAKEUP_THRESHOLD + 1)
#define ADDR_NUMBER_OF_TIPS   (ADDR_CURRENT_TIP + 1)

#define ADDR_TIP_NAME         (ADDR_NUMBER_OF_TIPS + 1)
#define ADDR_CAL_TEMP         (ADDR_TIP_NAME + (TIPNAMELENGTH * TIPMAX))

#define ADDR_LANGUAGE         (ADDR_CAL_TEMP + (2 * CALNUM * TIPMAX))
#define ADDR_HAND_SIDE        (ADDR_LANGUAGE + 1)

#define ADDR_EEPROM_SIZE      (ADDR_HAND_SIDE + 1)

extern uint16_t DefaultTemp;
extern uint16_t SleepTemp;
extern uint8_t BoostTemp;
extern uint16_t time2sleep;
extern uint8_t time2off;
extern uint8_t timeOfBoost;
extern uint8_t MainScrType;
extern bool PIDenable;
extern bool beepEnable;
extern volatile uint8_t VoltageValue;
extern bool QCEnable;
extern uint8_t WAKEUPthreshold;
extern uint8_t CurrentTip;
extern uint8_t NumberOfTips;

extern char TipName[TIPMAX][TIPNAMELENGTH];
extern uint16_t CalTemp[TIPMAX][CALNUM];

extern uint8_t language;
extern uint8_t hand_side;

bool write_default_EEPROM() {
  Serial.println("Writing default config to EEPROM");

  EEPROM.writeUShort(ADDR_DEFAULT_TEMP, TEMP_DEFAULT);
  EEPROM.writeUShort(ADDR_SLEEP_TEMP, TEMP_SLEEP);
  EEPROM.writeUChar(ADDR_BOOST_TEMP, TEMP_BOOST);
  EEPROM.writeUShort(ADDR_TIME_2_SLEEP, TIME2SLEEP);
  EEPROM.writeUChar(ADDR_TIME_2_OFF, TIME2OFF);
  EEPROM.writeUChar(ADDR_TIME_OF_BOOST, TIMEOFBOOST);
  EEPROM.writeUChar(ADDR_MAIN_SCREEN, MAINSCREEN);
  EEPROM.writeBool(ADDR_PID_ENABLE, PID_ENABLE);
  EEPROM.writeBool(ADDR_BEEP_ENABLE, BEEP_ENABLE);
  EEPROM.writeUChar(ADDR_VOLTAGE_VALUE, VOLTAGE_VALUE);
  EEPROM.writeBool(ADDR_QC_ENABLE, QC_ENABLE);
  EEPROM.writeUChar(ADDR_WAKEUP_THRESHOLD, WAKEUP_THRESHOLD);
  EEPROM.writeUChar(ADDR_CURRENT_TIP, 0);
  EEPROM.writeUChar(ADDR_NUMBER_OF_TIPS, 1);

  CalTemp[0][0] = TEMP200;
  CalTemp[0][1] = TEMP280;
  CalTemp[0][2] = TEMP360;
  CalTemp[0][3] = TEMPCHP;

  memset(TipName[0], 0, TIPNAMELENGTH);
  strncpy(TipName[0], TIPNAME, TIPNAMELENGTH - 1);

  EEPROM.writeBytes(ADDR_TIP_NAME, TipName[0], TIPNAMELENGTH);
  for (uint8_t j = 0; j < CALNUM; j++) {
    EEPROM.writeUShort(ADDR_CAL_TEMP + j * 2, CalTemp[0][j]);
  }

  EEPROM.writeUChar(ADDR_LANGUAGE, DEFAULT_LANGUAGE);
  EEPROM.writeUChar(ADDR_HAND_SIDE, DEFAULT_HAND_SIDE);
  EEPROM.writeUInt(ADDR_SYSTEM_INIT_FLAG, VERSION_NUM);

  if (EEPROM.commit()) {
    Serial.println("Default config Done");
    return true;
  }
  Serial.println("Default config Failed");
  return false;
}

bool init_EEPROM() {
  Serial.println("Initialising EEPROM");
  if (!EEPROM.begin(ADDR_EEPROM_SIZE)) {
    Serial.println("Failed to initialise EEPROM");
    return false;
  }
  Serial.println("EEPROM Done");
  return true;
}

bool update_EEPROM() {
  Serial.println("Updating EEPROM");

  EEPROM.writeUShort(ADDR_DEFAULT_TEMP, DefaultTemp);
  EEPROM.writeUShort(ADDR_SLEEP_TEMP, SleepTemp);
  EEPROM.writeUChar(ADDR_BOOST_TEMP, BoostTemp);
  EEPROM.writeUShort(ADDR_TIME_2_SLEEP, time2sleep);
  EEPROM.writeUChar(ADDR_TIME_2_OFF, time2off);
  EEPROM.writeUChar(ADDR_TIME_OF_BOOST, timeOfBoost);
  EEPROM.writeUChar(ADDR_MAIN_SCREEN, MainScrType);
  EEPROM.writeBool(ADDR_PID_ENABLE, PIDenable);
  EEPROM.writeBool(ADDR_BEEP_ENABLE, beepEnable);
  EEPROM.writeUChar(ADDR_VOLTAGE_VALUE, VoltageValue);
  EEPROM.writeBool(ADDR_QC_ENABLE, QCEnable);
  EEPROM.writeUChar(ADDR_WAKEUP_THRESHOLD, WAKEUPthreshold);
  EEPROM.writeUChar(ADDR_CURRENT_TIP, CurrentTip);
  EEPROM.writeUChar(ADDR_NUMBER_OF_TIPS, NumberOfTips);

  for (uint8_t i = 0; i < NumberOfTips; i++) {
    EEPROM.writeBytes(ADDR_TIP_NAME + i * TIPNAMELENGTH, TipName[i], TIPNAMELENGTH);
    for (uint8_t j = 0; j < CALNUM; j++) {
      EEPROM.writeUShort(ADDR_CAL_TEMP + i * 2 * CALNUM + j * 2, CalTemp[i][j]);
    }
  }

  EEPROM.writeUChar(ADDR_LANGUAGE, language);
  EEPROM.writeUChar(ADDR_HAND_SIDE, hand_side);
  EEPROM.writeUInt(ADDR_SYSTEM_INIT_FLAG, VERSION_NUM);

  if (EEPROM.commit()) {
    Serial.println("EEPROM Update Done");
    return true;
  }
  Serial.println("EEPROM Update Failed");
  return false;
}

bool read_EEPROM() {
  Serial.println("Reading EEPROM");

  if (EEPROM.readUInt(ADDR_SYSTEM_INIT_FLAG) != VERSION_NUM) {
    Serial.println("System uninitialized or version mismatch; resetting");
    if (!write_default_EEPROM()) return false;
  }

  bool dirty = false;

  DefaultTemp = EEPROM.readUShort(ADDR_DEFAULT_TEMP);
  SleepTemp = EEPROM.readUShort(ADDR_SLEEP_TEMP);
  BoostTemp = EEPROM.readUChar(ADDR_BOOST_TEMP);
  time2sleep = EEPROM.readUShort(ADDR_TIME_2_SLEEP);
  time2off = EEPROM.readUChar(ADDR_TIME_2_OFF);
  timeOfBoost = EEPROM.readUChar(ADDR_TIME_OF_BOOST);
  MainScrType = EEPROM.readUChar(ADDR_MAIN_SCREEN);
  VoltageValue = EEPROM.readUChar(ADDR_VOLTAGE_VALUE);
  WAKEUPthreshold = EEPROM.readUChar(ADDR_WAKEUP_THRESHOLD);
  CurrentTip = EEPROM.readUChar(ADDR_CURRENT_TIP);
  NumberOfTips = EEPROM.readUChar(ADDR_NUMBER_OF_TIPS);
  language = EEPROM.readUChar(ADDR_LANGUAGE);
  hand_side = EEPROM.readUChar(ADDR_HAND_SIDE);

  uint8_t rawPID = EEPROM.readUChar(ADDR_PID_ENABLE);
  if (rawPID <= 1) PIDenable = rawPID; else { PIDenable = PID_ENABLE; dirty = true; }
  
  uint8_t rawBeep = EEPROM.readUChar(ADDR_BEEP_ENABLE);
  if (rawBeep <= 1) beepEnable = rawBeep; else { beepEnable = BEEP_ENABLE; dirty = true; }
  
  uint8_t rawQC = EEPROM.readUChar(ADDR_QC_ENABLE);
  if (rawQC <= 1) QCEnable = rawQC; else { QCEnable = QC_ENABLE; dirty = true; }

  if (DefaultTemp < TEMP_MIN || DefaultTemp > TEMP_MAX) { DefaultTemp = TEMP_DEFAULT; dirty = true; }
  if (SleepTemp < 50 || SleepTemp > TEMP_MAX) { SleepTemp = TEMP_SLEEP; dirty = true; }
  if (BoostTemp < 10 || BoostTemp > 100) { BoostTemp = TEMP_BOOST; dirty = true; }
  if (time2sleep > 600) { time2sleep = TIME2SLEEP; dirty = true; }
  if (time2off > 60) { time2off = TIME2OFF; dirty = true; }
  if (timeOfBoost > 180) { timeOfBoost = TIMEOFBOOST; dirty = true; }
  if (WAKEUPthreshold > 50) { WAKEUPthreshold = WAKEUP_THRESHOLD; dirty = true; }

  if (NumberOfTips == 0 || NumberOfTips > TIPMAX) { NumberOfTips = 1; dirty = true; }
  if (CurrentTip >= NumberOfTips) { CurrentTip = 0; dirty = true; }
  if (VoltageValue > 4) { VoltageValue = VOLTAGE_VALUE; dirty = true; }
  if (language >= language_types) { language = DEFAULT_LANGUAGE; dirty = true; }
  if (hand_side > 1) { hand_side = DEFAULT_HAND_SIDE; dirty = true; }
  if (MainScrType > 1) { MainScrType = MAINSCREEN; dirty = true; }

  for (uint8_t i = 0; i < NumberOfTips; i++) {
    memset(TipName[i], 0, TIPNAMELENGTH);
    EEPROM.readBytes(ADDR_TIP_NAME + i * TIPNAMELENGTH, TipName[i], TIPNAMELENGTH);
    TipName[i][TIPNAMELENGTH - 1] = '\0';

    for (uint8_t j = 0; j < CALNUM; j++) {
      CalTemp[i][j] = EEPROM.readUShort(ADDR_CAL_TEMP + i * 2 * CALNUM + j * 2);
    }

    if (CalTemp[i][0] < 100 || CalTemp[i][0] > 500 ||
        CalTemp[i][1] < 100 || CalTemp[i][1] > 500 ||
        CalTemp[i][2] < 100 || CalTemp[i][2] > 500 ||
        CalTemp[i][0] + 10 >= CalTemp[i][1] ||
        CalTemp[i][1] + 10 >= CalTemp[i][2]) {
      CalTemp[i][0] = TEMP200;
      CalTemp[i][1] = TEMP280;
      CalTemp[i][2] = TEMP360;
      CalTemp[i][3] = TEMPCHP;
      dirty = true;
    }
  }

  if (dirty) {
    if (!update_EEPROM()) {
      Serial.println("EEPROM auto-repair failed");
      return false;
    }
  }

  return true;
}

#endif // UTILS_EEPROM_H
