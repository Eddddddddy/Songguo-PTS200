#include "config.h"
#include "UtilsEEPROM.h"
#include "Languages.h"

#include "USB.h"
#include "FirmwareMSC.h"

#include <QC3Control.h>
QC3Control QC(14, 13);
// QC.set12V();

#include <U8g2lib.h> // https://github.com/olikraus/u8g2
// #include<analogWrite.h>
#include "esp_adc_cal.h"
#include <ESP32AnalogRead.h> //Click here to get the library: http://librarymanager/All#ESP32AnalogRead

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <PID_v1.h> // https://github.com/wagiminator/ATmega-Soldering-Station/blob/master/software/libraries/Arduino-PID-Library.zip
// (old cpp version of https://github.com/mblythe86/C-PID-Library/tree/master/PID_v1)
#include <EEPROM.h> // 用于将用户设置存储到EEPROM

// 选择加速度计芯片
// #define MPU
#define LIS

/*#if defined(MPU)
  #include <MPU6050_tockn.h> //https://github.com/tockn/MPU6050_tockn
  MPU6050 mpu6050(Wire);*/

// #if defined(LIS)
#include "SparkFun_LIS2DH12.h" //Click here to get the library: http://librarymanager/All#SparkFun_LIS2DH12
SPARKFUN_LIS2DH12 accel;       // Create instance

/*#else
  #error Wrong SENSOR type!
  #endif*/

int16_t gx = 0, gy = 0, gz = 0;

// 定义积极和保守的PID调整参数
double aggKp = 11, aggKi = 0.5, aggKd = 1;
double consKp = 11, consKi = 3, consKd = 5;

// 用户可以更改并存储在EEPROM中的默认值
uint16_t DefaultTemp = TEMP_DEFAULT;
uint16_t SleepTemp = TEMP_SLEEP;
uint8_t BoostTemp = TEMP_BOOST;
uint16_t time2sleep = TIME2SLEEP;
uint8_t time2off = TIME2OFF;
uint8_t timeOfBoost = TIMEOFBOOST;
uint8_t MainScrType = MAINSCREEN;
bool PIDenable = PID_ENABLE;
bool beepEnable = BEEP_ENABLE;
uint8_t VoltageValue = VOLTAGE_VALUE;
bool QCEnable = QC_ENABLE;
uint8_t WAKEUPthreshold = WAKEUP_THRESHOLD;
bool restore_default_config = false;

// T12的默认值
uint16_t CalTemp[TIPMAX][4] = {TEMP200, TEMP280, TEMP360, TEMPCHP};
char TipName[TIPMAX][TIPNAMELENGTH] = {TIPNAME};
uint8_t CurrentTip = 0;
uint8_t NumberOfTips = 1;

// Variables for pin change interrupt 引脚更改中断的变量
volatile uint8_t a0, b0, c0, d0;
volatile bool ab0;
volatile int count, countMin, countMax, countStep;
volatile bool handleMoved;

// Variables for temperature control 温度控制变量
uint16_t SetTemp, ShowTemp, gap, Step;
double Input, Output, Setpoint, RawTemp, CurrentTemp, ChipTemp;

// Variables for voltage readings 电压读数变量
uint16_t Vcc, Vin;

// State variables 状态变量
bool inLockMode = true;
bool inSleepMode = false;
bool inOffMode = false;
bool inBoostMode = false;
bool inCalibMode = false;
bool isWorky = true;
bool beepIfWorky = true;
bool TipIsPresent = true;
bool OledClear;

// Timing variables 时间变量
uint32_t sleepmillis;
uint32_t boostmillis;
uint32_t buttonmillis;
uint32_t goneMinutes;
uint32_t goneSeconds;
uint8_t SensorCounter = 255;

// Specify variable pointers and initial PID tuning parameters
// 指定变量指针和初始PID调优参数
PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, REVERSE);

// Setup u8g object depending on OLED controller
// 根据OLED控制器设置u8g对象
#if defined(SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21);
#elif defined(SH1107)
U8G2_SH1107_64X128_F_HW_I2C u8g2(U8G2_R1, 7);
#else
#error Wrong OLED controller type!
#endif

// Buffer for drawUTF8
char F_Buffer[20];

float lastSENSORTmp = 0;
float newSENSORTmp = 0;
uint8_t SENSORTmpTime = 0;

// ADC Calibrate
uint16_t vref_adc0, vref_adc1;
ESP32AnalogRead adc_sensor;
ESP32AnalogRead adc_vin;

// Language
uint8_t language = 0;

// MSC Firmware
FirmwareMSC MSC_Update;
bool MSC_Updating_Flag = false;

void setup()
{
  pinMode(14, INPUT);
  pinMode(13, INPUT);
  // QC.set12V();
  Serial.begin(115200);
  // delay(5000);

  //  analogSetAttenuation(ADC_11db);
  //  vref_adc0 = calibrate_adc(ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_5);
  //  vref_adc1 = calibrate_adc(ADC_UNIT_2, (adc_atten_t)ADC2_CHANNEL_9);
  adc_sensor.attach(SENSOR_PIN);
  adc_vin.attach(VIN_PIN);

  /*#if defined(MPU)
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);*/

  // #endif

  // set the pin modes 设置引脚模式
  pinMode(SENSOR_PIN, INPUT);
  pinMode(VIN_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CONTROL_PIN, OUTPUT);
  pinMode(BUTTON_P_PIN, INPUT_PULLUP);
  pinMode(BUTTON_N_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  analogWrite(CONTROL_PIN, HEATER_OFF); // this shuts off the heater这是用来关闭加热器的
  // digitalWrite(BUZZER_PIN, LOW);        // must be LOW when buzzer not in use当蜂鸣器不使用时，必须是低电平

  // get default values from EEPROM 从EEPROM获取默认值
  //  if (!EEPROM.begin(EEPROM_SIZE))
  //  {
  //    Serial.println("failed to initialise EEPROM");
  //    delay(100);
  //  }
  init_EEPROM();
  if (digitalRead(BUTTON_P_PIN) == LOW && digitalRead(BUTTON_N_PIN) == LOW && digitalRead(BUTTON_PIN) == HIGH)
  {
    write_default_EEPROM();
  }
  getEEPROM();

  pinMode(PD_CFG_0, OUTPUT);
  pinMode(PD_CFG_1, OUTPUT);
  pinMode(PD_CFG_2, OUTPUT);

  if(QCEnable){
    switch (VoltageValue)
    {
    case 0:
      QC.set9V();
      break;
    case 1:
      QC.set12V();
      break;
    case 2:
      QC.set12V();
      break;
    case 3:
      QC.set20V();
      break;
    default:
      break;
    }
  }

  PD_Update();

  // read supply voltages in mV 以mV为单位读取电源电压
  delay(100);
  Vin = getVIN();

  // read and set current iron temperature 读取和设置当前的烙铁头温度
  SetTemp = DefaultTemp;
  RawTemp = denoiseAnalog(SENSOR_PIN);

  calculateTemp();

  // turn on heater if iron temperature is well below setpoint
  // 如果烙铁头温度远低于设定值，则打开加热器
  if (((CurrentTemp + 20) < DefaultTemp) && !inLockMode)
    analogWrite(CONTROL_PIN, HEATER_ON);

  // set PID output range and start the PID
  // 设置PID输出范围，启动PID
  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(AUTOMATIC);

  // set initial rotary encoder values 设置旋转编码器的初始值
  // a0 = PINB & 1; b0 = PIND >> 7 & 1; ab0 = (a0 == b0);
  a0 = 0;
  b0 = 0;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);

  // reset sleep timer 睡眠定时器重置
  sleepmillis = millis();

  // long beep for setup completion 安装完成时长哔哔声
  beep();
  beep();
  //  delay(2000);
  Serial.println("Soldering Pen");
  // #elif defined(LIS)
  //   u8g2.setBusClock(100000);
  Wire.begin();
  if (accel.begin() == false)
  {
    delay(500);
    Serial.println("Accelerometer not detected.");
  }
  ChipTemp = getChipTemp();
  lastSENSORTmp = getMPUTemp();
  u8g2.initDisplay();
  u8g2.begin();
  u8g2.enableUTF8Print();

  OledClear = 1;
}

void loop()
{
  ROTARYCheck(); // check rotary encoder (temp/boost setting, enter setup menu) 检查旋转编码器(温度/升压设置，进入设置菜单)
  SLEEPCheck();  // check and activate/deactivate sleep modes 检查和激活/关闭睡眠模式
  SENSORCheck(); // reads temperature and vibration switch of the iron 读取烙铁头的温度和振动开关
  Thermostat();  // heater control 加热器控制
  MainScreen();  // updates the main page on the OLED 刷新OLED主界面
}

// check rotary encoder; set temperature, toggle boost mode, enter setup menu accordingly
// 检查旋转编码器;设置温度，切换升压模式，进入设置菜单相应
void ROTARYCheck()
{
  // set working temperature according to rotary encoder value
  // 根据旋转编码器值设定工作温度
  SetTemp = getRotary();

  // check rotary encoder switch 检查旋转编码器开关
  uint8_t c = digitalRead(BUTTON_PIN);
  if (!c && c0)
  {
    beep();
    buttonmillis = millis();
    while ((!digitalRead(BUTTON_PIN)) && ((millis() - buttonmillis) < 500))
      ;
    if ((millis() - buttonmillis) >= 500)
    {
      SetupScreen();
    }
    else
    {
      if (inLockMode)
      {
        inLockMode = false;
      }
      else
      {
        if (inOffMode)
        {
          inOffMode = false;
        }
        else
        {
          inBoostMode = !inBoostMode;
          if (inBoostMode)
          {
            boostmillis = millis();
          }
          handleMoved = true;
        }
      }
    }
  }
  c0 = c;

  // check timer when in boost mode 在升温模式时检查计时器
  if (inBoostMode && timeOfBoost)
  {
    goneSeconds = (millis() - boostmillis) / 1000;
    if (goneSeconds >= timeOfBoost)
    {
      inBoostMode = false; // stop boost mode 停止升温模式
      beep();              // beep if boost mode is over 如果升温模式结束，会发出蜂鸣声
      beepIfWorky = true;  // beep again when working temperature is reached 当达到工作温度，会发出蜂鸣声
    }
  }
}

// check and activate/deactivate sleep modes 检查和激活/关闭睡眠模式
void SLEEPCheck()
{
  if (inLockMode)
  {
    ;
  }
  else
  {
    if (handleMoved)
    { // if handle was moved 如果手柄被移动
      u8g2.setPowerSave(0);
      if (inSleepMode)
      {                                        // in sleep or off mode? 在睡眠模式还是关机模式?
        if ((CurrentTemp + 20) < SetTemp)      // if temp is well below setpoint 如果温度远低于设定值
          analogWrite(CONTROL_PIN, HEATER_ON); // then start the heater right now 那现在就启动加热器
        beep();                                // beep on wake-up
        beepIfWorky = true;                    // beep again when working temperature is reached 当达到工作温度，会发出蜂鸣声
      }
      handleMoved = false; // reset handleMoved flag
      inSleepMode = false; // reset sleep flag
      //      inOffMode = false;      // reset off flag
      sleepmillis = millis(); // reset sleep timer
    }

    // check time passed since the handle was moved 检查把手被移动后经过的时间
    goneSeconds = (millis() - sleepmillis) / 1000;
    if ((!inSleepMode) && (time2sleep > 0) && (goneSeconds >= time2sleep))
    {
      inSleepMode = true;
      beep();
    }
    else if ((!inOffMode) && (time2off > 0) && ((goneSeconds / 60) >= time2off))
    {
      inOffMode = true;
      u8g2.setPowerSave(1);
      beep();
    }
  }
}

// reads temperature, vibration switch and supply voltages 读取温度，振动开关和电源电压
void SENSORCheck()
{
  /*#if defined(MPU)
    mpu6050.update();
    if (abs(mpu6050.getGyroX() - gx) > WAKEUP_THRESHOLD || abs(mpu6050.getGyroY() - gy) > WAKEUP_THRESHOLD || abs(mpu6050.getGyroZ() - gz) > WAKEUP_THRESHOLD)
    {
      gx = mpu6050.getGyroX();
      gy = mpu6050.getGyroY();
      gz = mpu6050.getGyroZ();
      handleMoved = true;
      Serial.println("进入工作状态!");
    }*/
  // #if defined(LIS)
  if (abs(accel.getX() - gx) > WAKEUP_THRESHOLD || abs(accel.getY() - gy) > WAKEUP_THRESHOLD || abs(accel.getZ() - gz) > WAKEUP_THRESHOLD)
  {
    gx = accel.getX();
    gy = accel.getY();
    gz = accel.getZ();
    handleMoved = true;
    //    Serial.println("进入工作状态!");
  }
  // #endif

  analogWrite(CONTROL_PIN, HEATER_OFF);    // shut off heater in order to measure temperature 关闭加热器以测量温度
  delayMicroseconds(TIME2SETTLE);          // wait for voltage to settle 等待电压稳定
  double temp = denoiseAnalog(SENSOR_PIN); // 读取ADC值的温度

  if (!SensorCounter--)
    Vin = getVIN(); // get Vin every now and then 时不时去获取VIN电压

  if (!inLockMode)
  {
    analogWrite(CONTROL_PIN, HEATER_PWM); // turn on again heater 再次打开加热器
  }

  RawTemp += (temp - RawTemp) * SMOOTHIE; // stabilize ADC temperature reading 稳定ADC温度读数
  calculateTemp();                        // calculate real temperature value 计算实际温度值

  // stabilize displayed temperature when around setpoint 稳定显示温度时，周围的设定值
  if ((ShowTemp != Setpoint) || (abs(ShowTemp - CurrentTemp) > 5))
    ShowTemp = CurrentTemp;
  if (abs(ShowTemp - Setpoint) <= 1)
    ShowTemp = Setpoint;
  if (inLockMode)
  {
    ShowTemp = 0;
  }

  // set state variable if temperature is in working range; beep if working temperature was just reached
  // 温度在工作范围内可设置状态变量;当工作温度刚刚达到时，会发出蜂鸣声
  gap = abs(SetTemp - CurrentTemp);
  if (gap < 5)
  {
    if (!isWorky && beepIfWorky)
      beep();
    isWorky = true;
    beepIfWorky = false;
  }
  else
    isWorky = false;

  // checks if tip is present or currently inserted 检查烙铁头是否存在或当前已插入
  if (ShowTemp > 500)
    TipIsPresent = false; // tip removed ? 烙铁头移除？
  if (!TipIsPresent && (ShowTemp < 500))
  {                                                    // new tip inserted ? 新的烙铁头插入？
    analogWrite(CONTROL_PIN, HEATER_OFF);              // shut off heater 关闭加热器
    beep();                                            // beep for info
    TipIsPresent = true;                               // tip is present now 烙铁头已经存在
    ChangeTipScreen();                                 // show tip selection screen 显示烙铁头选择屏幕
    updateEEPROM();                                    // update setting in EEPROM EEPROM的更新设置
    handleMoved = true;                                // reset all timers 重置所有计时器
    RawTemp = denoiseAnalog(SENSOR_PIN);               // restart temp smooth algorithm 重启临时平滑算法
    c0 = LOW;                                          // switch must be released 必须松开开关
    setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp); // reset rotary encoder 重置旋转编码器
  }
}

// calculates real temperature value according to ADC reading and calibration values
// 根据ADC读数和校准值，计算出真实的温度值
void calculateTemp()
{
  if (RawTemp < 200)
    CurrentTemp = map(RawTemp, 0, 200, 15, CalTemp[CurrentTip][0]);
  else if (RawTemp < 280)
    CurrentTemp = map(RawTemp, 200, 280, CalTemp[CurrentTip][0], CalTemp[CurrentTip][1]);
  else
    CurrentTemp = map(RawTemp, 280, 360, CalTemp[CurrentTip][1], CalTemp[CurrentTip][2]);
}

// controls the heater 控制加热器
void Thermostat()
{
  // define Setpoint acoording to current working mode 根据当前工作模式定义设定值
  if (inOffMode || inLockMode)
    Setpoint = 0;
  else if (inSleepMode)
    Setpoint = SleepTemp;
  else if (inBoostMode)
  {
    Setpoint = constrain(SetTemp + BoostTemp, 0, 450);
  }
  else
    Setpoint = SetTemp;

  if (SetTemp != DefaultTemp)
  {
    DefaultTemp = SetTemp; // 把设置里面的默认温度也修改了
    update_default_temp_EEPROM();
  }

  // control the heater (PID or direct) 控制加热器(PID或直接)
  gap = abs(Setpoint - CurrentTemp);
  if (PIDenable)
  {
    Input = CurrentTemp;
    if (gap < 30)
      ctrl.SetTunings(consKp, consKi, consKd);
    else
      ctrl.SetTunings(aggKp, aggKi, aggKd);
    ctrl.Compute();
  }
  else
  {
    // turn on heater if current temperature is below setpoint
    // 如果当前温度低于设定值，则打开加热器
    if ((CurrentTemp + 0.5) < Setpoint)
      Output = 0;
    else
      Output = 255;
  }
  analogWrite(CONTROL_PIN, HEATER_PWM); // set heater PWM 设置加热器PWM
}

// creates a short beep on the buzzer 在蜂鸣器上创建一个短的哔哔声
void beep()
{
  if (beepEnable)
  {
    for (uint8_t i = 0; i < 255; i++)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(125);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(125);
    }
  }
}

// sets start values for rotary encoder 设置旋转编码器的起始值
void setRotary(int rmin, int rmax, int rstep, int rvalue)
{
  countMin = rmin << ROTARY_TYPE;
  countMax = rmax << ROTARY_TYPE;
  countStep = rstep;
  count = rvalue << ROTARY_TYPE;
}

// reads current rotary encoder value 读取当前旋转编码器值
int getRotary()
{
  Button_loop();
  return (count >> ROTARY_TYPE);
}

// reads user settings from EEPROM; if EEPROM values are invalid, write defaults
// 从EEPROM读取用户设置;如果EEPROM值无效，则写入默认值
void getEEPROM()
{
  read_EEPROM();
}

// writes user settings to EEPROM using updade function to minimize write cycles
// 使用升级功能将用户设置写入EEPROM，以最小化写入周期
void updateEEPROM()
{
  update_EEPROM();
}

// draws the main screen 绘制主屏幕
void MainScreen()
{
  u8g2.firstPage();
  do
  {
    // u8g2.setCursor(0, 0);
    // u8g2.print(F("nihao"));
    //  draw setpoint temperature
    u8g2.setFont(u8g2_font_unifont_t_chinese3);
    u8g2.setFontPosTop();
    //    u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, "设温:");
    u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_set_temp[language]);
    u8g2.setCursor(40, 0 + SCREEN_OFFSET);
    u8g2.print(Setpoint, 0);

    // draw status of heater 绘制加热器状态
    u8g2.setCursor(96, 0 + SCREEN_OFFSET);
    if (ShowTemp > 500)
      u8g2.print(txt_error[language]);
    else if (inOffMode || inLockMode)
      u8g2.print(txt_off[language]);
    else if (inSleepMode)
      u8g2.print(txt_sleep[language]);
    else if (inBoostMode)
      u8g2.print(txt_boost[language]);
    else if (isWorky)
      u8g2.print(txt_worky[language]);
    else if (Output < 180)
      u8g2.print(txt_on[language]);
    else
      u8g2.print(txt_hold[language]);

    // rest depending on main screen type 休息取决于主屏幕类型
    if (MainScrType)
    {
      // draw current tip and input voltage 绘制当前烙铁头及输入电压
      float fVin = (float)Vin / 1000; // convert mv in V
      newSENSORTmp = newSENSORTmp + 0.01 * getMPUTemp();
      SENSORTmpTime++;
      if (SENSORTmpTime >= 100)
      {
        lastSENSORTmp = newSENSORTmp;
        newSENSORTmp = 0;
        SENSORTmpTime = 0;
      }
      u8g2.setCursor(0, 50);
      u8g2.print(lastSENSORTmp, 1);
      u8g2.print(F("C"));
      u8g2.setCursor(83, 50);
      u8g2.print(fVin, 1);
      u8g2.print(F("V"));
      // draw current temperature 绘制当前温度
      u8g2.setFont(u8g2_font_freedoomr25_tn);
      u8g2.setFontPosTop();
      u8g2.setCursor(37, 18);
      if (ShowTemp > 500)
        u8g2.print(F("000"));
      else
        u8g2.printf("%03d", ShowTemp);
    }
    else
    {
      // draw current temperature in big figures 用大数字绘制当前温度
      u8g2.setFont(u8g2_font_fub42_tn);
      u8g2.setFontPosTop();
      u8g2.setCursor(15, 20);
      if (ShowTemp > 500)
        u8g2.print(F("000"));
      else
        u8g2.printf("%03d", ShowTemp);
    }
  } while (u8g2.nextPage());
}

// setup screen 设置屏幕
void SetupScreen()
{
  analogWrite(CONTROL_PIN, HEATER_OFF); // shut off heater
  beep();
  uint16_t SaveSetTemp = SetTemp;
  uint8_t selection = 0;
  bool repeat = true;

  while (repeat)
  {
    selection = MenuScreen(SetupItems, sizeof(SetupItems), selection);
    switch (selection)
    {
    case 0:
    {
      TipScreen();
      repeat = false;
    }
    break;
    case 1:
    {
      TempScreen();
    }
    break;
    case 2:
    {
      TimerScreen();
    }
    break;
      //      case 3:
      //        PIDenable = MenuScreen(ControlTypeItems, sizeof(ControlTypeItems), PIDenable);
      //        break;
    case 3:
    {
      MainScrType = MenuScreen(MainScreenItems, sizeof(MainScreenItems), MainScrType);
    }
    break;
    case 4:
    {
      InfoScreen();
    }
    break;
    case 5:
      VoltageValue = MenuScreen(VoltageItems, sizeof(VoltageItems), VoltageValue);
      PD_Update();
      break;
    case 6:
      QCEnable = MenuScreen(QCItems, sizeof(QCItems), QCEnable);
      break;
    case 7:
      beepEnable = MenuScreen(BuzzerItems, sizeof(BuzzerItems), beepEnable);
      break;
    case 8:
    {
      restore_default_config = MenuScreen(DefaultItems, sizeof(DefaultItems), restore_default_config);
      if (restore_default_config)
      {
        restore_default_config = false;
        write_default_EEPROM();
        read_EEPROM();
      }
    }
    break;
    case 9:
    {
      bool lastbutton = (!digitalRead(BUTTON_PIN));
      u8g2.clearBuffer();                      // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);      // choose a suitable font
      u8g2.drawStr(0, 10, "MSC Update"); // write something to the internal memory
      u8g2.sendBuffer();                       // transfer internal memory to the display
      delay(1000);
      do
      {
        MSC_Update.onEvent(usbEventCallback);
        MSC_Update.begin();
        if (lastbutton && digitalRead(BUTTON_PIN))
        {
          delay(10);
          lastbutton = false;
        }
      } while (digitalRead(BUTTON_PIN) || lastbutton);

      MSC_Update.end();
    }
    break;
    case 10:
    {
      Serial.println(language);
      language = MenuScreen(LanguagesItems, sizeof(LanguagesItems), language);
      Serial.println(language);
      repeat = false;
    }
    break;
    default:
      repeat = false;
      break;
    }
  }
  updateEEPROM();
  handleMoved = true;
  SetTemp = SaveSetTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}

// tip settings screen 烙铁头设置屏幕
void TipScreen()
{
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat)
  {
    selection = MenuScreen(TipItems, sizeof(TipItems), selection);
    switch (selection)
    {
    case 0:
      ChangeTipScreen();
      break;
    case 1:
      CalibrationScreen();
      break;
    case 2:
      InputNameScreen();
      break;
    case 3:
      DeleteTipScreen();
      break;
    case 4:
      AddTipScreen();
      break;
    default:
      repeat = false;
      break;
    }
  }
}

// temperature settings screen 温度设置屏幕
void TempScreen()
{
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat)
  {
    selection = MenuScreen(TempItems, sizeof(TempItems), selection);
    switch (selection)
    {
    case 0:
      setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
      DefaultTemp = InputScreen(DefaultTempItems);
      break;
    case 1:
      setRotary(50, TEMP_MAX, TEMP_STEP, SleepTemp);
      SleepTemp = InputScreen(SleepTempItems);
      break;
    case 2:
      setRotary(10, 100, TEMP_STEP, BoostTemp);
      BoostTemp = InputScreen(BoostTempItems);
      break;
    default:
      repeat = false;
      break;
    }
  }
}

// timer settings screen 定时器设置屏幕
void TimerScreen()
{
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat)
  {
    selection = MenuScreen(TimerItems, sizeof(TimerItems), selection);
    switch (selection)
    {
    case 0:
      setRotary(0, 600, 10, time2sleep);
      time2sleep = InputScreen(SleepTimerItems);
      break;
    case 1:
      setRotary(0, 60, 1, time2off);
      time2off = InputScreen(OffTimerItems);
      break;
    case 2:
      setRotary(0, 180, 10, timeOfBoost);
      timeOfBoost = InputScreen(BoostTimerItems);
      break;
    case 3:
      setRotary(0, 50, 5, WAKEUPthreshold);
      WAKEUPthreshold = InputScreen(WAKEUPthresholdItems);
      break;
    default:
      repeat = false;
      break;
    }
  }
}

// menu screen 菜单屏幕
uint8_t MenuScreen(const char *Items[][language_types], uint8_t numberOfItems, uint8_t selected)
{
  Serial.println(numberOfItems);
  bool isTipScreen = ((strcmp(Items[0][language], "烙铁头:") == 0) || (strcmp(Items[0][language], "Tip:") == 0) || (strcmp(Items[0][language], "烙鐵頭:") == 0));
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected)
    arrow = 1;
  numberOfItems = numberOfItems / language_types;
  numberOfItems >>= 2;

  // 根据OLED控制器设置选择方向
#if defined(SSD1306)
  setRotary(0, numberOfItems + 3, 1, selected);
#elif defined(SH1107)
  setRotary(0, numberOfItems - 2, 1, selected);
#else
#error Wrong OLED controller type!
#endif

  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do
  {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do
    {
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      if (isTipScreen)
        u8g2.drawUTF8(54, 0 + SCREEN_OFFSET, TipName[CurrentTip]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++)
      {
        uint8_t drawnumber = selected + i + 1 - arrow;
        if (drawnumber < numberOfItems)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET, Items[selected + i + 1 - arrow][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return selected;
}

void MessageScreen(const char *Items[][language_types], uint8_t numberOfItems)
{
  numberOfItems = numberOfItems / language_types;
  bool lastbutton = (!digitalRead(BUTTON_PIN));
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_unifont_t_chinese3);
    u8g2.setFontPosTop();
    for (uint8_t i = 0; i < numberOfItems; i++)
      u8g2.drawUTF8(0, i * 16, Items[i][language]);
  } while (u8g2.nextPage());
  do
  {
    if (lastbutton && digitalRead(BUTTON_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);
  beep();
}

// input value screen 输入值屏幕
uint16_t InputScreen(const char *Items[][language_types])
{
  uint16_t value;
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do
  {
    value = getRotary();
    u8g2.firstPage();
    do
    {
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      u8g2.setCursor(0, 32);
      u8g2.print(">");
      u8g2.setCursor(10, 32);
      if (value == 0)
        u8g2.print(txt_Deactivated[language]);
      else
      {
        u8g2.print(value);
        u8g2.print(" ");
        u8g2.print(Items[1][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return value;
}

// information display screen 信息显示屏幕
void InfoScreen()
{
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do
  {
    Vin = getVIN();                 // read supply voltage
    float fVin = (float)Vin / 1000; // convert mv in V
    float fTmp = getChipTemp();     // read cold junction temperature
    u8g2.firstPage();
    do
    {
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.setFontPosTop();
      u8g2.setCursor(0, 0 + SCREEN_OFFSET);
      u8g2.print(txt_temp[language]);
      u8g2.print(fTmp, 1);
      u8g2.print(F(" C"));
      u8g2.setCursor(0, 16 + SCREEN_OFFSET);
      u8g2.print(txt_voltage[language]);
      u8g2.print(fVin, 1);
      u8g2.print(F(" V"));
      u8g2.setCursor(0, 16 * 2 + SCREEN_OFFSET);
      u8g2.print(txt_Version[language]);
      u8g2.print(VERSION);
      // u8g2.setCursor(0, 48); u8g2.print(F("IMU:  ")); u8g2.print(accelerometer[1], DEC); u8g2.print(F(""));
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
}

// change tip screen 改变烙铁头屏幕
void ChangeTipScreen()
{
  uint8_t selected = CurrentTip;
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected)
    arrow = 1;
  setRotary(0, NumberOfTips - 1, 1, selected);
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  Serial.print("selected: ");
  Serial.println(selected);
  Serial.print("lastselected: \n");
  Serial.println(lastselected);
  Serial.print("NumberOfTips: \n");
  Serial.println(NumberOfTips);

  do
  {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do
    {
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.setFontPosTop();
      //      strcpy_P(F_Buffer, PSTR("选择烙铁头"));
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_select_tip[language]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++)
      {
        uint8_t drawnumber = selected + i - arrow;
        if (drawnumber < NumberOfTips)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET, TipName[selected + i - arrow]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN))
    {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  CurrentTip = selected;
}

// temperature calibration screen 温度校准屏幕
void CalibrationScreen()
{
  uint16_t CalTempNew[4];
  uint16_t tempSetTemp = SetTemp;
  for (uint8_t CalStep = 0; CalStep < 3; CalStep++)
  {
    SetTemp = CalTemp[CurrentTip][CalStep];
    Serial.print("SetTemp: ");
    Serial.println(SetTemp);
    setRotary(100, 500, 1, SetTemp);
    beepIfWorky = true;
    bool lastbutton = (!digitalRead(BUTTON_PIN));

    do
    {
      SENSORCheck(); // reads temperature and vibration switch of the iron 读取烙铁头的温度和振动开关
      Thermostat();  // heater control

      u8g2.firstPage();
      do
      {
        u8g2.setFont(u8g2_font_unifont_t_chinese3);
        u8g2.setFontPosTop();
        //        strcpy_P(F_Buffer, PSTR("校准"));
        u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_calibrate[language]);
        u8g2.setCursor(0, 16 + SCREEN_OFFSET);
        u8g2.print(txt_step[language]);
        u8g2.print(CalStep + 1);
        u8g2.print(" of 3");
        if (isWorky)
        {
          u8g2.setCursor(0, 32 + SCREEN_OFFSET);
          u8g2.print(txt_set_measured[language]);
          u8g2.setCursor(0, 48 + SCREEN_OFFSET);
          u8g2.print(txt_s_temp[language]);
          u8g2.print(getRotary());
        }
        else
        {
          u8g2.setCursor(0, 32 + SCREEN_OFFSET);
          u8g2.print(txt_temp_2[language]);
          u8g2.print(uint16_t(RawTemp));
          u8g2.setCursor(0, 48 + SCREEN_OFFSET);
          u8g2.print(txt_wait_pls[language]);
        }
      } while (u8g2.nextPage());
      if (lastbutton && digitalRead(BUTTON_PIN))
      {
        delay(10);
        lastbutton = false;
      }
    } while (digitalRead(BUTTON_PIN) || lastbutton);

    CalTempNew[CalStep] = getRotary();
    beep();
    delay(10);
  }

  analogWrite(CONTROL_PIN, HEATER_OFF); // shut off heater 关闭加热器
  delayMicroseconds(TIME2SETTLE);       // wait for voltage to settle 等待电压稳定
  CalTempNew[3] = getChipTemp();        // read chip temperature 读芯片温度
  if ((CalTempNew[0] + 10 < CalTempNew[1]) && (CalTempNew[1] + 10 < CalTempNew[2]))
  {
    if (MenuScreen(StoreItems, sizeof(StoreItems), 0))
    {
      for (uint8_t i = 0; i < 4; i++)
        CalTemp[CurrentTip][i] = CalTempNew[i];
    }
  }

  SetTemp = tempSetTemp;
  update_EEPROM();
}

// input tip name screen 输入烙铁头名字屏幕
void InputNameScreen()
{
  uint8_t value;

  for (uint8_t digit = 0; digit < (TIPNAMELENGTH - 1); digit++)
  {
    bool lastbutton = (!digitalRead(BUTTON_PIN));
    setRotary(31, 96, 1, 65);
    do
    {
      value = getRotary();
      if (value == 31)
      {
        value = 95;
        setRotary(31, 96, 1, 95);
      }
      if (value == 96)
      {
        value = 32;
        setRotary(31, 96, 1, 32);
      }
      u8g2.firstPage();
      do
      {
        u8g2.setFont(u8g2_font_unifont_t_chinese3);
        u8g2.setFontPosTop();
        u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_enter_tip_name[language]);
        u8g2.setCursor(12 * digit, 48 + SCREEN_OFFSET);
        u8g2.print(char(94));
        u8g2.setCursor(0, 32 + SCREEN_OFFSET);
        for (uint8_t i = 0; i < digit; i++)
          u8g2.print(TipName[CurrentTip][i]);
        u8g2.setCursor(12 * digit, 32 + SCREEN_OFFSET);
        u8g2.print(char(value));
      } while (u8g2.nextPage());
      if (lastbutton && digitalRead(BUTTON_PIN))
      {
        delay(10);
        lastbutton = false;
      }
    } while (digitalRead(BUTTON_PIN) || lastbutton);
    TipName[CurrentTip][digit] = value;
    beep();
    delay(10);
  }
  TipName[CurrentTip][TIPNAMELENGTH - 1] = 0;
  return;
}

// delete tip screen 删除烙铁头屏幕
void DeleteTipScreen()
{
  if (NumberOfTips == 1)
  {
    MessageScreen(DeleteMessage, sizeof(DeleteMessage));
  }
  else if (MenuScreen(SureItems, sizeof(SureItems), 0))
  {
    if (CurrentTip == (NumberOfTips - 1))
    {
      CurrentTip--;
    }
    else
    {
      for (uint8_t i = CurrentTip; i < (NumberOfTips - 1); i++)
      {
        for (uint8_t j = 0; j < TIPNAMELENGTH; j++)
          TipName[i][j] = TipName[i + 1][j];
        for (uint8_t j = 0; j < 4; j++)
          CalTemp[i][j] = CalTemp[i + 1][j];
      }
    }
    NumberOfTips--;
  }
}

// add new tip screen 添加新的烙铁头屏幕
void AddTipScreen()
{
  if (NumberOfTips < TIPMAX)
  {
    CurrentTip = NumberOfTips++;
    InputNameScreen();
    CalTemp[CurrentTip][0] = TEMP200;
    CalTemp[CurrentTip][1] = TEMP280;
    CalTemp[CurrentTip][2] = TEMP360;
    CalTemp[CurrentTip][3] = TEMPCHP;
  }
  else
    MessageScreen(MaxTipMessage, sizeof(MaxTipMessage));
}

// 对32个ADC读数进行平均以降噪
//  VP+_Ru = 100k, Rd_GND = 1K
uint16_t denoiseAnalog(byte port)
{

  uint32_t result = 0;

  for (uint8_t i = 0; i < 32; i++)
  { // get 32 readings 得到32个读数
    float value, raw_adc;
    //    if (analogRead(SENSOR_PIN) < 1300)
    //    {
    //      value = analogRead(SENSOR_PIN) * 1390 / 4095 * 0.4;   //原0.4
    //    }
    //    else if (1300 <= analogRead(SENSOR_PIN) && analogRead(SENSOR_PIN) < 2300)
    //    {
    //      value = analogRead(SENSOR_PIN) * 1390 / 4095 * 0.36;  //原0.357
    //    }
    //    else if (2300 <= analogRead(SENSOR_PIN) && analogRead(SENSOR_PIN) < 4095)
    //      value = analogRead(SENSOR_PIN) * 1390 / 4095 * 0.31;   //原0.34
    raw_adc = adc_sensor.readMiliVolts();
    //    value = constrain(0.6*raw_adc-40, 50, 1000);
    value = constrain(0.4432 * raw_adc + 29.665, 20, 1000);
    //    Serial.println(raw_adc);
    result += value; // add them up 把它们加起来（结果与返回值）
    //    result+=adc_sensor.readMiliVolts();
  }
  //  Serial.printf("raw_val: %d", adc_sensor.readMiliVolts());
  //  Serial.println();
  Serial.printf("val: %d", result / 32);
  Serial.println();
  return (result >> 5); // devide by 32 and return value 除以32并返回值
}

// 读取SENSOR内部温度
double getChipTemp()
{
#if defined(MPU)
  mpu6050.update();
  int16_t Temp = mpu6050.getTemp();
#elif defined(LIS)
  int16_t Temp = accel.getTemperature();
#endif

  return Temp;
}

// get LIS/MPU temperature 获取LIS/MPU的温度
float getMPUTemp()
{
#if defined(MPU)
  mpu6050.update();
  int16_t Temp = mpu6050.getTemp();
#elif defined(LIS)
  int16_t Temp = accel.getTemperature();
#endif

  return Temp;
}

// get supply voltage in mV 得到以mV为单位的电源电压
uint16_t getVIN()
{
  long value;
  long voltage;
  long result = 0;

  for (uint8_t i = 0; i < 32; i++)
  { // get 32 readings 得到32个读数
    //    long val = analogRead(VIN_PIN);
    long val = adc_vin.readMiliVolts();

    result += val; // add them up 把它们加起来
  }

  value = (result >> 5);

  //  // VIN_Ru = 100k, Rd_GND = 3.3K
  //  if (value < 500)
  //  {
  //    voltage = value * 1390 * 31.3 / 4095 * 1.35;
  //  }
  //  else if (500 <= value && value < 1000)
  //  {
  //    voltage = value * 1390 * 31.3 / 4095 * 1.135;
  //  }
  //  else if (1000 <= value && value < 1500)
  //  {
  //    voltage = value * 1390 * 31.3 / 4095 * 1.071;
  //  }
  //  else if (1500 <= value && value < 2000)
  //  {
  //    voltage = value * 1390 * 31.3 / 4095;
  //  }
  //  else if (2000 <= value && value < 3000)
  //  {
  //    voltage = value * 1390 * 31.3 / 4095;
  //  }
  //  else
  //    voltage = value * 1390 * 31.3 / 4095;

  voltage = value * 31.3;

  return voltage;
  // return value;
}

int32_t variance(int16_t a[])
{
  // Compute mean (average of elements)计算平均值(元素的平均值)
  int32_t sum = 0;

  for (int i = 0; i < 32; i++)
    sum += a[i];
  int16_t mean = (int32_t)sum / 32;
  // Compute sum squared differences with mean.计算和平方差的平均值
  int32_t sqDiff = 0;
  for (int i = 0; i < 32; i++)
    sqDiff += (a[i] - mean) * (a[i] - mean);
  return (int32_t)sqDiff / 32;
}

unsigned int Button_Time1 = 0, Button_Time2 = 0;

void Button_loop()
{
  if (!digitalRead(BUTTON_N_PIN) && a0 == 1)
  {
    delay(BUTTON_DELAY);
    if (!digitalRead(BUTTON_N_PIN))
    {
      count = constrain(count + countStep, countMin, countMax);
      a0 = 0;
    }
  }
  else if (!digitalRead(BUTTON_N_PIN) && a0 == 0)
  {
    delay(BUTTON_DELAY);
    if (Button_Time1 > 10) // 这里的数越大，需要长按时间更长
      count = constrain(count + countStep, countMin, countMax);
    else
      Button_Time1++;
  }
  else if (digitalRead(BUTTON_N_PIN))
  {
    Button_Time1 = 0;
    a0 = 1;
  }

  if (!digitalRead(BUTTON_P_PIN) && b0 == 1)
  {
    delay(BUTTON_DELAY);
    if (!digitalRead(BUTTON_P_PIN))
    {
      count = constrain(count - countStep, countMin, countMax);
      b0 = 0;
    }
  }
  else if (!digitalRead(BUTTON_P_PIN) && b0 == 0)
  {
    delay(BUTTON_DELAY);
    if (Button_Time2 > 10) // 这里的数越大，需要长按时间更长
      count = constrain(count - countStep, countMin, countMax);
    else
      Button_Time2++;
  }
  else if (digitalRead(BUTTON_P_PIN))
  {
    Button_Time2 = 0;
    b0 = 1;
  }
}

void PD_Update()
{
  switch (VoltageValue)
  {
  case 0:
  {
    digitalWrite(PD_CFG_0, LOW);
    digitalWrite(PD_CFG_1, LOW);
    digitalWrite(PD_CFG_2, LOW);
  }
  break;
  case 1:
  {
    digitalWrite(PD_CFG_0, LOW);
    digitalWrite(PD_CFG_1, LOW);
    digitalWrite(PD_CFG_2, HIGH);
  }
  break;
  case 2:
  {
    digitalWrite(PD_CFG_0, LOW);
    digitalWrite(PD_CFG_1, HIGH);
    digitalWrite(PD_CFG_2, HIGH);
  }
  break;
  case 3:
  {
    digitalWrite(PD_CFG_0, LOW);
    digitalWrite(PD_CFG_1, HIGH);
    digitalWrite(PD_CFG_2, LOW);
  }
  break;
  default:
    break;
  }
}

static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == ARDUINO_USB_EVENTS)
  {
    // arduino_usb_event_data_t* data = (arduino_usb_event_data_t*)event_data;
    switch (event_id)
    {
    case ARDUINO_USB_STARTED_EVENT:
      // HWSerial.println("USB PLUGGED");
      u8g2.clearBuffer();                 // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
      u8g2.drawStr(0, 10, "USB PLUGGED"); // write something to the internal memory
      u8g2.sendBuffer();                  // transfer internal memory to the display
      break;
    case ARDUINO_USB_STOPPED_EVENT:
      // HWSerial.println("USB UNPLUGGED");
      u8g2.clearBuffer();                   // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
      u8g2.drawStr(0, 10, "USB UNPLUGGED"); // write something to the internal memory
      u8g2.sendBuffer();                    // transfer internal memory to the display
      break;
    case ARDUINO_USB_SUSPEND_EVENT:
      // HWSerial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en);
      u8g2.clearBuffer();                   // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
      u8g2.drawStr(0, 10, "USB SUSPENDED"); // write something to the internal memory
      u8g2.sendBuffer();                    // transfer internal memory to the display
      break;
    case ARDUINO_USB_RESUME_EVENT:
      // HWSerial.println("USB RESUMED");
      u8g2.clearBuffer();                 // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
      u8g2.drawStr(0, 10, "USB RESUMED"); // write something to the internal memory
      u8g2.sendBuffer();                  // transfer internal memory to the display
      break;

    default:
      break;
    }
  }
  else if (event_base == ARDUINO_FIRMWARE_MSC_EVENTS)
  {
    // arduino_firmware_msc_event_data_t* data = (arduino_firmware_msc_event_data_t*)event_data;
    switch (event_id)
    {
    case ARDUINO_FIRMWARE_MSC_START_EVENT:
      // HWSerial.println("MSC Update Start");
      u8g2.clearBuffer();                      // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);      // choose a suitable font
      u8g2.drawStr(0, 10, "MSC Update Start"); // write something to the internal memory
      u8g2.sendBuffer();                       // transfer internal memory to the display
      break;
    case ARDUINO_FIRMWARE_MSC_WRITE_EVENT:
      // HWSerial.printf("MSC Update Write %u bytes at offset %u\n", data->write.size, data->write.offset);
      //  HWSerial.print(".");
      u8g2.clearBuffer();                  // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
      u8g2.drawStr(0, 10, "MSC Updating"); // write something to the internal memory
      u8g2.sendBuffer();                   // transfer internal memory to the display
      break;
    case ARDUINO_FIRMWARE_MSC_END_EVENT:
      // HWSerial.printf("\nMSC Update End: %u bytes\n", data->end.size);
      u8g2.clearBuffer();                    // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);    // choose a suitable font
      u8g2.drawStr(0, 10, "MSC Update End"); // write something to the internal memory
      u8g2.sendBuffer();                     // transfer internal memory to the display
      break;
    case ARDUINO_FIRMWARE_MSC_ERROR_EVENT:
      // HWSerial.printf("MSC Update ERROR! Progress: %u bytes\n", data->error.size);
      u8g2.clearBuffer();                       // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);       // choose a suitable font
      u8g2.drawStr(0, 10, "MSC Update ERROR!"); // write something to the internal memory
      u8g2.sendBuffer();                        // transfer internal memory to the display
      break;
    case ARDUINO_FIRMWARE_MSC_POWER_EVENT:
      // HWSerial.printf("MSC Update Power: power: %u, start: %u, eject: %u", data->power.power_condition, data->power.start, data->power.load_eject);
      u8g2.clearBuffer();                      // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);      // choose a suitable font
      u8g2.drawStr(0, 10, "MSC Update Power"); // write something to the internal memory
      u8g2.sendBuffer();                       // transfer internal memory to the display
      break;

    default:
      break;
    }
  }
}

// uint16_t calibrate_adc(adc_unit_t adc, adc_atten_t channel) {
//   uint16_t vref;
//   esp_adc_cal_characteristics_t adc_chars;
//   esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)adc, (adc_atten_t)channel, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
//   //Check type of calibration value used to characterize ADC
//   if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
//     Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
//     Serial.println();
//     vref = adc_chars.vref;
//   } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
//     Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
//     Serial.println();
//   } else {
//     Serial.println("Default Vref: 1100mV");
//   }
//   return vref;
// }
