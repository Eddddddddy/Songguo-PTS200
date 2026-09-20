#ifndef LANGUAGES_H
#define LANGUAGES_H

#include <Arduino.h>

const uint8_t language_types = 1;

const char *SetupItems[][language_types] = {
  {"Setup"},
  {"Tip"},
  {"Temp"},
  {"Timer"},
  {"Display"},
  {"Info"},
  {"Voltage"},
  {"QC3.0"},
  {"Buzzer"},
  {"Reset Config"},
  {"Update FW"},
  {"L/R Hand"},
  {"Back"}
};

const char *TipItems[][language_types] = {
  {"Tip"},
  {"Select Tip"},
  {"Calibrate"},
  {"Edit Name"},
  {"Delete Tip"},
  {"Add Tip"},
  {"Back"}
};

const char *TempItems[][language_types] = {
  {"Temp"},
  {"Default Temp"},
  {"Sleep Temp"},
  {"Boost Temp"},
  {"Back"}
};

const char *TimerItems[][language_types] = {
  {"Timer"},
  {"Sleep Timer"},
  {"Off Timer"},
  {"Boost Timer"},
  {"Sensitivity"},
  {"Back"}
};

const char *MainScreenItems[][language_types] = {
  {"Display"},
  {"Simple"},
  {"Detailed"}
};

const char *StoreItems[][language_types] = {
  {"Save Cal?"},
  {"No"},
  {"Yes"}
};

const char *DefaultItems[][language_types] = {
  {"Reset Config?"},
  {"No"},
  {"Yes"}
};

const char *SureItems[][language_types] = {
  {"Are you sure?"},
  {"No"},
  {"Yes"}
};

const char *VoltageItems[][language_types] = {
  {"Voltage"},
  {"9V"},
  {"12V"},
  {"15V"},
  {"20V (3A)"},
  {"20V (5A)"}
};

const char *QCItems[][language_types] = {
  {"QC3.0"},
  {"Disable"},
  {"Enable"}
};

const char *BuzzerItems[][language_types] = {
  {"Buzzer"},
  {"Disable"},
  {"Enable"}
};

const char *DefaultTempItems[][language_types] = {
  {"Default Temp"},
  {"C"}
};

const char *SleepTempItems[][language_types] = {
  {"Sleep Temp"},
  {"C"}
};

const char *BoostTempItems[][language_types] = {
  {"Boost Temp"},
  {"C"}
};

const char *SleepTimerItems[][language_types] = {
  {"Sleep Timer"},
  {"s"}
};

const char *WAKEUPthresholdItems[][language_types] = {
  {"Sensitivity"},
  {" "}
};

const char *OffTimerItems[][language_types] = {
  {"Off Timer"},
  {"m"}
};

const char *BoostTimerItems[][language_types] = {
  {"Boost Timer"},
  {"s"}
};

const char *DeleteMessage[][language_types] = {
  {"Cannot delete!"},
  {"At least 1 tip"},
  {"must exist."}
};

const char *MaxTipMessage[][language_types] = {
  {"Cannot add!"},
  {"Max tip count"},
  {"reached."}
};

const char *txt_set_temp[] = {"Set:"};
const char *txt_error[] = {"ERROR"};
const char *txt_off[] = {"OFF"};
const char *txt_sleep[] = {"SLEEP"};
const char *txt_boost[] = {"BOOST"};
const char *txt_worky[] = {"READY"};
const char *txt_on[] = {"HEATING"};
const char *txt_hold[] = {"HOLD"};
const char *txt_Deactivated[] = {"Disabled"};
const char *txt_temp[] = {"Temp: "};
const char *txt_voltage[] = {"Voltage: "};
const char *txt_Version[] = {"Ver: "};
const char *txt_select_tip[] = {"Select Tip:"};
const char *txt_calibrate[] = {"Calibrate:"};
const char *txt_step[] = {"Step "};
const char *txt_set_measured[] = {"Set Measured:"};
const char *txt_s_temp[] = {"Temp: "};
const char *txt_temp_2[] = {"ADC: "};
const char *txt_wait_pls[] = {"Please wait..."};
const char *txt_enter_tip_name[] = {"Enter Name:"};

#endif // LANGUAGES_H
