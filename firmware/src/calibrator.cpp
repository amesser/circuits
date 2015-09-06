/*
 * calibrator.cpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#include <ecpp/String.hpp>
#include <string.h>

#include "calibrator_app.hpp"

using namespace ecpp;

typedef FlashVariable<char, 16> LcdStringType;

struct LcdStrings
{
  const LcdStringType StatusReady;
  const LcdStringType MenuitemMeasure;
  const LcdStringType MenuitemHumidity;
  const LcdStringType MenuitemLight;
  const LcdStringType MenuitemTemp;
  const LcdStringType MenuitemReset;
  const LcdStringType MenuitemCalibrate;
  const LcdStringType StatusMeasuring;
  const LcdStringType StatusRead;
  const LcdStringType StatusErasing;
  const LcdStringType StatusCalibrating;
  const LcdStringType StatusInitTransfer;
  const LcdStringType StatusTransfer;
  const LcdStringType DisplayDataUncalibrated;
  const LcdStringType DisplayDataCalibrated;
};

static constexpr struct LcdStrings s_LcdStrings PROGMEM =
{
  .StatusReady             = "Bereit          ",
  .MenuitemMeasure         = "Sensor auslesen ",
  .MenuitemHumidity        = "Bodenfeuchte    ",
  .MenuitemLight           = "Helligkeit      ",
  .MenuitemTemp            = "Temperatur      ",
  .MenuitemReset           = "Sensor zur" "\xf5" "cksetzen",
  .MenuitemCalibrate       = "Sensor kalibrieren",
  .StatusMeasuring         = "Lese Sensor     ",
  .StatusRead              = "Sensor gelesen  ",
  .StatusErasing           = "L" "\xef" "sche Sensor   ",
  .StatusCalibrating       = "Kalibriere Sensor",
  .StatusInitTransfer      = "Initialisiere...",
  .StatusTransfer          = "\xf5" "bertrage  XX/24",
  .DisplayDataUncalibrated = "Preview    XXXXX",
  .DisplayDataCalibrated   = "Sensor     XXXXX",
};

struct StateHandler
{
  const FlashVariable<CalibratorApp::Handler> leaveState;
  const FlashVariable<CalibratorApp::Handler> enterState;
  const FlashVariable<CalibratorApp::Handler> pollState;
};

/*
APP_STATE_POWERUP = 0,
APP_STATE_IDLE,
APP_STATE_READSENSOR,
APP_STATE_WRITESENSOR,
APP_STATE_MAX,
*/


static constexpr FlashVariable<CalibratorApp::Handler>
s_CalibratorAppLeaveStateHandler[CalibratorApp::APP_STATE_MAX] PROGMEM =
{
  &CalibratorApp::leaveStatePowerUp,
  0,
  &CalibratorApp::leaveStateReadSensor,
  0,
};

static constexpr FlashVariable<CalibratorApp::Handler>
s_CalibratorAppEnterStateHandler[CalibratorApp::APP_STATE_MAX] PROGMEM =
{
  0,
  &CalibratorApp::enterStateIdle,
  &CalibratorApp::enterStateReadSensor,
  &CalibratorApp::enterStateWriteSensor,
};

static constexpr FlashVariable<CalibratorApp::Handler>
s_CalibratorAppPollStateHandler[CalibratorApp::APP_STATE_MAX] PROGMEM =
{
  0,
  &CalibratorApp::pollStateIdle,
  &CalibratorApp::pollStateReadSensor,
  &CalibratorApp::pollStateWriteSensor,
};

static EEVariable<calibration_statistics> s_CalStatistics EEMEM;


void CalibratorApp::changeState(enum AppState NextState)
{
  m_MenuState = MENU_STATE_IDLE;

  invokeFromList(s_CalibratorAppLeaveStateHandler, m_State, 0);
  invokeFromList(s_CalibratorAppEnterStateHandler,NextState, 0);

  m_State = NextState;
}


static constexpr FlashVariable<uint8_t> s_MenuLcdStringOffset[CalibratorApp::MENU_STATE_IDLE] PROGMEM =
{
  offsetof(struct LcdStrings, MenuitemMeasure),
  offsetof(struct LcdStrings, MenuitemHumidity),
  offsetof(struct LcdStrings, MenuitemLight),
  offsetof(struct LcdStrings, MenuitemTemp),
  offsetof(struct LcdStrings, MenuitemReset),
};

static constexpr FlashVariable<CalibratorApp::Handler>
s_MenuHandler[CalibratorApp::MENU_STATE_IDLE] PROGMEM =
{
  0,
  &CalibratorApp::displayHumidity,
  &CalibratorApp::displayLight,
  &CalibratorApp::displayTemperature,
  0,
};

void
CalibratorApp::changeMenuState(enum MenuState NextState)
{
  if(NextState < MENU_STATE_IDLE)
  {
    auto & bsp = CalibratorBsp::getBsp();
    auto & lcd = bsp.getLCD();
    auto & buffer = lcd.getBuffer();

    const uint8_t * p      = reinterpret_cast<const uint8_t*>(&s_LcdStrings);
    const LcdStringType *pString = reinterpret_cast<const LcdStringType*>(
        p + s_MenuLcdStringOffset[NextState]);

    pString->read(buffer);

    lcd.moveCursor(0);
    lcd.writeTextBuffer(16);

    memset(&buffer, 0x00, sizeof(buffer));
    invokeFromList(s_MenuHandler, NextState, 0);

    lcd.moveCursor(0x40);
    lcd.writeTextBuffer(16);
  }

  m_MenuState = NextState;

}

static const FlashVariable<char, 16> s_FormatHex PROGMEM    = "0123456789ABCDEF";
static const FlashVariable<struct calibration_param> s_ResetCalibrationParam PROGMEM =
{
  {0x10000UL, 0x0000, 0xFFFF},
};

void
CalibratorApp::handleKeys()
{
  auto & bsp    = CalibratorBsp::getBsp();

  const auto laststate = m_KeyState;
  const auto newstate  = bsp.getKeyState();
  const auto keydown   = (newstate ^ laststate) & newstate;

  if (keydown & 0x01)
  { /* next/abort pressed */
    if(m_State == APP_STATE_IDLE)
    {
      auto nextstate = m_MenuState + 1;

      if(nextstate >= MENU_STATE_IDLE)
      {
        nextstate = 0;
      }

      changeMenuState(static_cast<enum MenuState>(nextstate));
    }
  }
  else if (keydown & 0x02)
  { /* ok pressed */
    if (m_MenuState == MENU_STATE_READSENSOR)
    {
       changeState(APP_STATE_READSENSOR);
    }
    else if (m_MenuState == MENU_STATE_RESETSENSOR)
    {
       auto & data = bsp.getVoltageModulator().getBuffer<calibration_data>();
       auto & lcd = bsp.getLCD();
       auto & buffer = lcd.getBuffer();

       s_ResetCalibrationParam.read(data.Humidity);
       s_ResetCalibrationParam.read(data.Light);
       s_ResetCalibrationParam.read(data.Temperature);

       s_LcdStrings.StatusErasing.read(buffer);
       lcd.moveCursor(0);
       lcd.writeTextBuffer(16);

       changeState(APP_STATE_WRITESENSOR);
    }
    else if (m_MenuState == MENU_STATE_HUMIDITY)
    {
      auto & uart    = bsp.getUartHandler();
      auto & ubuffer = uart.getBuffer();

      measurement_data * pMeasData = reinterpret_cast<measurement_data *>(&ubuffer);

      /* statistic data can only be generated from uncalibrated sensor data
       * output */
      if(0xC0 == pMeasData->sync && 0x00 == pMeasData->type)
      {
        collectMinMaxStatistics(m_CalStatistics.Humidity, pMeasData->humidity_counts, true);
      }
    }
    else if (m_MenuState == MENU_STATE_LIGHT)
    {
      auto & uart    = bsp.getUartHandler();
      auto & ubuffer = uart.getBuffer();

      measurement_data * pMeasData = reinterpret_cast<measurement_data *>(&ubuffer);

      /* statistic data can only be generated from uncalibrated sensor data
       * output */
      if(0xC0 == pMeasData->sync && 0x00 == pMeasData->type)
      {
        collectMinMaxStatistics(m_CalStatistics.Light, pMeasData->led_counts, true);
      }
    }
    else if (m_MenuState == MENU_STATE_TEMP)
    {
      auto & uart    = bsp.getUartHandler();
      auto & ubuffer = uart.getBuffer();

      measurement_data * pMeasData = reinterpret_cast<measurement_data *>(&ubuffer);

      /* statistic data can only be generated from uncalibrated sensor data
       * output */
      if(0xC0 == pMeasData->sync && 0x00 == pMeasData->type)
      {
        collectTempStatistics(m_CalStatistics.Temperature, pMeasData->temp, 263);
      }
    }
  }

  m_KeyState = newstate;
}


void
CalibratorApp::handleState()
{
  uint_fast8_t State     = m_State;

  if(State == APP_STATE_POWERUP)
  {
    changeState(APP_STATE_IDLE);
  }
  else
  {
    invokeFromList(s_CalibratorAppPollStateHandler, State, 0);
  }
}

void
CalibratorApp::readSensor()
{
}

void CalibratorApp::leaveStatePowerUp(uint_fast16_t Argument)
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();


  s_LcdStrings.StatusReady.read(buffer);
  lcd.moveCursor(0);
  lcd.writeTextBuffer(16);

  memset(buffer, ' ', sizeof(buffer));
  lcd.moveCursor(0x40);
  lcd.writeTextBuffer(16);

}

void CalibratorApp::enterStateIdle(uint_fast16_t Argument)
{
  auto & bsp = CalibratorBsp::getBsp();

  bsp.setSensorVoltage(bsp.SUPPLY_OFF);
}

void CalibratorApp::pollStateIdle(uint_fast16_t Argument)
{
}

void CalibratorApp::leaveStateReadSensor(uint_fast16_t Argument)
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & uart = bsp.getUartHandler();

  s_LcdStrings.StatusRead.read(buffer);
  lcd.moveCursor(0);
  lcd.writeTextBuffer(16);
}

void CalibratorApp::enterStateReadSensor(uint_fast16_t Argument)
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & uart = bsp.getUartHandler();

  s_CalStatistics.read(m_CalStatistics);

  uart.activate();
  bsp.setSensorVoltage(bsp.SUPPLY_5V);

  s_LcdStrings.StatusMeasuring.read(buffer);
  lcd.moveCursor(0);
  lcd.writeTextBuffer(16);
}

void
CalibratorApp::pollStateReadSensor(uint_fast16_t Argument)
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & uart = bsp.getUartHandler();

  uart.handleCyclic();

  memset(buffer, ' ', sizeof(buffer));

  if(1)
  {
    auto & ubuffer = uart.getBuffer();
    uint_fast8_t Idx;

    for(Idx = 0; Idx < sizeof(ubuffer); ++Idx)
    {
      buffer[Idx * 2]     = (ubuffer[Idx] >> 4) & 0x0F;
      buffer[Idx * 2 + 1] = (ubuffer[Idx]) & 0x0F;
    }

    for(Idx = 0; Idx < sizeof(buffer); ++Idx)
    {
      uint8_t c = buffer[Idx];

      if (c < 16)
        buffer[Idx] = s_FormatHex[buffer[Idx] % 16];
    }
  }
  lcd.moveCursor(0x40);
  lcd.writeTextBuffer(16);

  if(uart.finished())
  {
    calculateCalibration();

    changeState(APP_STATE_IDLE);

    s_LcdStrings.StatusRead.read(buffer);
    lcd.moveCursor(0);
    lcd.writeTextBuffer(16);
  }
}

void CalibratorApp::enterStateWriteSensor(uint_fast16_t Argument)
{
  auto & g   = Globals::getGlobals();
  auto & bsp = CalibratorBsp::getBsp();
  auto & timer  = g.getVoltageModulatorTimer();
  auto & vmod   = bsp.getVoltageModulator();

  timer.start(10000);
  bsp.setSensorVoltage(bsp.SUPPLY_3V);
  vmod.startTransmission();
}

void CalibratorApp::pollStateWriteSensor(uint_fast16_t Argument)
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & vmod   = bsp.getVoltageModulator();

  vmod.handleTransmission();

  memset(buffer, ' ', sizeof(buffer));

  if(vmod.getTransferring() == 0)
  {/* transfer has not started yet */
    s_LcdStrings.StatusInitTransfer.read(buffer);
  }
  else
  {
    s_LcdStrings.StatusTransfer.read(buffer);
    String::formatDecimal(&(buffer[16-5]), 2, vmod.getNumTransferred(), '0');
  }

  lcd.moveCursor(0x40);
  lcd.writeTextBuffer(16);

  if(vmod.hasFinished())
  {
    changeState(APP_STATE_READSENSOR);
  }
}

void CalibratorApp::calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm)
{
  prm.Offset = -stat.Min;
  prm.Max    =  stat.Max - stat.Min;
  prm.Mult   = (0xFFFF * 0x10000ULL) / prm.Max;
}


void CalibratorApp::calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm)
{
  /* y = Mult * (x + Offset) / 65536 */

  uint_fast8_t NumPoints;
  int_fast32_t Numerator, Denominator, SumX, SumY;

  NumPoints = sum(stat.NumPoints);

  SumX      = sum(stat.SumX);
  SumY      = sum(stat.SumY);

  Numerator   = sum(stat.SumXY) - ((SumX * SumY) + NumPoints / 2) / NumPoints;
  Denominator = sum(stat.SumXX) - ((SumX * SumX) + NumPoints / 2) / NumPoints;

  prm.Mult   = (((int_fast64_t)Numerator) * 65536ULL) / Denominator ;
  prm.Offset =    (SumY + NumPoints / 2) * 65536ULL / NumPoints / prm.Mult
                - (SumX + NumPoints / 2) / NumPoints;
  prm.Max    = 0xFFFF0000 / prm.Mult;
}

#define MAX_MINMAX_JUMPWIDTH (25)

void
CalibratorApp::collectMinMaxStatistics(struct calibration_minmax &stat, uint_fast16_t Counts, bool ForceCollect)
{
  const auto MaxJumpwidth = ForceCollect ? TypeProperties<decltype(stat.Max)>::MaxUnsigned : MAX_MINMAX_JUMPWIDTH;

  if(stat.Min > stat.Max)
  { /* First value */
    stat.Min = stat.Max = Counts;
  }
  else if(Counts > stat.Max && (Counts - stat.Max) <= MaxJumpwidth)
  {
    stat.Max = Counts;
  }
  else if(Counts < stat.Min && (stat.Min - Counts) <= MaxJumpwidth)
  {
    stat.Min = Counts;
  }
}

void
CalibratorApp::collectTempStatistics(struct calibration_linearregression &stat, uint_fast16_t TempCounts, uint_fast16_t Temp)
{
  /* Temperature statistics is kept in seperate bins of 2 degrees.
   * This is used to improve the temperature calibration distribution */

  /* Bins are available from [-10, +30 [ */
  if(Temp >= (273 - 10) && Temp < (273 + 30))
  {
    uint_fast16_t UTemp = Temp - (273 - 10);
    uint_fast8_t Idx = static_cast<uint_fast8_t>(UTemp / 2);

    if(stat.NumPoints[Idx] < (min(stat.NumPoints) + 2))
    {
      stat.NumPoints[Idx] += 1;
      stat.SumX[Idx]      += TempCounts;
      stat.SumX[Idx]      += TempCounts * TempCounts;
      stat.SumY[Idx]      += Temp;
      stat.SumXY[Idx]     += Temp * TempCounts;
    }
  }
}


/** Collect calibration statistics from sensor data.
 *
 * This function will update the calibration statistics
 * with the data read out from a sensor.
 */
void CalibratorApp::collectStatistics()
{
  auto & bsp    = CalibratorBsp::getBsp();
  auto & uart    = bsp.getUartHandler();
  auto & ubuffer = uart.getBuffer();

  measurement_data * pMeasData = reinterpret_cast<measurement_data *>(&ubuffer);

  /* statistic data can only be generated from uncalibrated sensor data
   * output */
  if(0xC0 == pMeasData->sync && 0x00 == pMeasData->type)
  {
    auto & CalStat = m_CalStatistics;

    collectMinMaxStatistics(CalStat.Humidity, pMeasData->humidity_counts);
    collectMinMaxStatistics(CalStat.Light,    pMeasData->led_counts);
    collectTempStatistics(CalStat.Temperature, pMeasData->temp, 263);
  }
}

void CalibratorApp::calculateCalibration()
{
  auto & Stat  = m_CalStatistics;
  auto & Param = m_Scratch.CalPrm;

  calculateMinMaxCal(Stat.Humidity, Param.Humidity);
  calculateMinMaxCal(Stat.Light, Param.Light);
  calculateLinRegr(Stat.Temperature, Param.Temperature);
}

uint16_t CalibratorApp::getValue(uint_fast8_t idx, uint_fast16_t input)
{
  auto & bsp    = CalibratorBsp::getBsp();
  auto & uart    = bsp.getUartHandler();
  auto & ubuffer = uart.getBuffer();
  measurement_data * pMeasData = reinterpret_cast<measurement_data *>(&ubuffer);
  Evaluator * pe = reinterpret_cast<Evaluator*>(&(m_Scratch.CalPrm));
  uint16_t *pInput = &(pMeasData->humidity_counts);

  return pe[idx].scale(pInput[idx]);
}

void CalibratorApp::displayValue(uint_fast8_t idx)
{
  auto & bsp    = CalibratorBsp::getBsp();
  auto & lcd    = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & uart    = bsp.getUartHandler();
  auto & ubuffer = uart.getBuffer();
  measurement_data * pMeasData = reinterpret_cast<measurement_data *>(&ubuffer);

  if(0xC0 == pMeasData->sync)
  {
    uint16_t *pInput = &(pMeasData->humidity_counts);

    if(0x00 == pMeasData->type)
    { /* uncalibrated sensor data
       * calculate possible result */
      uint16_t value   = getValue(idx, pInput[idx]);
      s_LcdStrings.DisplayDataUncalibrated.read(buffer);
      String::formatDecimal(&buffer[11], 5, value);
    }
    else if (0x01 == pMeasData->type)
    {
      /* calibrated sensor data */
      uint16_t value = pInput[idx];
      s_LcdStrings.DisplayDataCalibrated.read(buffer);
      String::formatDecimal(&buffer[11], 5, value);
    }
  }
}

void CalibratorApp::displayHumidity(uint_fast16_t Argument)
{
  displayValue(0);
}

void CalibratorApp::displayLight(uint_fast16_t Argument)
{
  displayValue(1);
}

void CalibratorApp::displayTemperature(uint_fast16_t Argument)
{
  displayValue(2);
}

static CalibratorApp app;

int main(void) __attribute__ ((OS_main));
int main(void)
{
  auto & bsp = CalibratorBsp::getBsp();

  bsp.init();
  while(1)
  {
    bsp.cycle();
    app.handleKeys();
    app.handleState();

    {
    }
    /* lcd.moveCursor(0);
     * lcd.writeData(abTest, 8); */
  }
}


