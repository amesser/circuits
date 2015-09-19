/*
 * calibrator.cpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#include <ecpp/String.hpp>
#include <ecpp/Byteorder.hpp>
#include <string.h>

#include "calibrator_app.hpp"

using namespace ecpp;

typedef FlashVariable<char, 16> LcdStringType;

/*  ö  = " "\xef" " */
static constexpr LcdStringType s_UiLcdStrings[] PROGMEM =
{
  "Sensor lesen    ", /* UI_STATE_READSENSOR */
  "Lese Sensor...  ", /* UI_STATE_READINGSENSOR */
  "Messwerte Sensor", /* UI_STATE_READSENSORRESULTS */
  "Rohdaten Sensor ", /* UI_STATE_READSENSORRAW */
  "Rohdaten Feuchte", /* UI_STATE_READSENSORHUMIDITY */
  "Rohdaten Licht  ", /* UI_STATE_READSENSORLIGHT */
  "Rohdaten Temp.  ", /* UI_STATE_READSENSORTEMPERATURE */
  "Fehler          ", /* UI_STATE_SENSORERROR */
  "Sensor l" "\xef" "schen  ", /* UI_STATE_ERASESENSOR */
  "L" "\xef" "sche Sensor...", /* UI_STATE_ERASINGSENSOR */
  "Sensor kalib.   ", /* UI_STATE_CALIBRATESENSOR */
  "Kalibriere S....", /* UI_STATE_CALIBRATINGSENSOR */
  "Stat. l" "\xef" "schen   ", /* UI_STATE_RESETSTATISTICS */
  "Automatikmodus  ", /* UI_STATE_AUTOCALIBRATION */
  "A Messinterval  ", /* UI_STATE_AUTOSELECTINTERVAL */
  "A Lese Sensor...", /* UI_STATE_AUTOREADINGSENSOR */
  "A Warte         ", /* UI_STATE_AUTOWAIT */
  "Bereit          ", /* UI_STATE_STARTUP */
};

static constexpr LcdStringType s_UiLcdNextOK PROGMEM =
  "[Weiter]    [OK]";

static constexpr LcdStringType s_UiLcdBaud   PROGMEM =
  "Lese        Baud";

static constexpr LcdStringType s_UiLcdWait PROGMEM =
  "Warte           ";

static constexpr LcdStringType s_UiLcdWriteProgress PROGMEM =
  "Schreibe   XX/24";

static constexpr LcdStringType s_UiLcdRaw PROGMEM =
  "[I]          [S]";

static constexpr FlashVariable<uint_least8_t> s_UiStateNext[] PROGMEM =
{
  CalibratorApp::UI_STATE_ERASESENSOR,           /* UI_STATE_READSENSOR */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_READINGSENSOR */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_READSENSORRESULTS */
  CalibratorApp::UI_STATE_READSENSORHUMIDITY,    /* UI_STATE_READSENSORRAW */
  CalibratorApp::UI_STATE_READSENSORLIGHT,       /* UI_STATE_READSENSORHUMIDITY */
  CalibratorApp::UI_STATE_READSENSORTEMPERATURE, /* UI_STATE_READSENSORLIGHT */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_READSENSORTEMPERATURE */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_SENSORERROR */
  CalibratorApp::UI_STATE_CALIBRATESENSOR,       /* UI_STATE_ERASESENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_ERASINGSENSOR */
  CalibratorApp::UI_STATE_RESETSTATISTICS,       /* UI_STATE_CALIBRATESENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_CALIBRATINGSENSOR */
  CalibratorApp::UI_STATE_AUTOCALIBRATION,       /* UI_STATE_RESETSTATISTICS */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_AUTOCALIBRATION */
  CalibratorApp::UI_STATE_AUTOSELECTINTERVAL,    /* UI_STATE_AUTOSELECTINTERVAL */
  CalibratorApp::UI_STATE_AUTOCALIBRATION,       /* UI_STATE_AUTOREADINGSENSOR */
  CalibratorApp::UI_STATE_AUTOCALIBRATION,       /* UI_STATE_AUTOWAIT */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_STARTUP */
};

static constexpr FlashVariable<uint_least8_t> s_UiStateOK[] PROGMEM =
{
  CalibratorApp::UI_STATE_READINGSENSOR,         /* UI_STATE_READSENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_READINGSENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_READSENSORRESULTS */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_READSENSORRAW */
  CalibratorApp::UI_STATE_READSENSORLIGHT,       /* UI_STATE_READSENSORHUMIDITY */
  CalibratorApp::UI_STATE_READSENSORTEMPERATURE, /* UI_STATE_READSENSORLIGHT */
  CalibratorApp::UI_STATE_READSENSOR,            /* UI_STATE_READSENSORTEMPERATURE */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_SENSORERROR */
  CalibratorApp::UI_STATE_ERASINGSENSOR,         /* UI_STATE_ERASESENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_ERASINGSENSOR */
  CalibratorApp::UI_STATE_CALIBRATINGSENSOR,     /* UI_STATE_CALIBRATESENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_CALIBRATINGSENSOR */
  CalibratorApp::UI_STATE_RESETSTATISTICS,       /* UI_STATE_RESETSTATISTICS */
  CalibratorApp::UI_STATE_AUTOSELECTINTERVAL,    /* UI_STATE_AUTOCALIBRATION */
  CalibratorApp::UI_STATE_AUTOREADINGSENSOR,     /* UI_STATE_AUTOSELECTINTERVAL */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_AUTOREADINGSENSOR */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_AUTOWAIT */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_STARTUP */
};


static EEVariable<calibration_statistics> s_CalStatistics EEMEM;
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

  enum UiState NextState    = UI_STATE_STARTUP;
  enum UiState CurrentState = static_cast<enum UiState>(m_MenuState);

  if (keydown & 0x01)
  { /* key next pressed */
    NextState = static_cast<enum UiState>((uint8_t)s_UiStateNext[CurrentState]);

    switch(CurrentState)
    {
    case UI_STATE_READSENSORTEMPERATURE:
      {
        s_CalStatistics = m_CalStatistics;
      }
      break;
    default:
      break;
    }
  }
  else if (keydown & 0x02)
  { /* ok pressed */
    NextState = static_cast<enum UiState>((uint8_t)s_UiStateOK[CurrentState]);

    switch(CurrentState)
    {
    case UI_STATE_READSENSORHUMIDITY:
      {
        auto & bsp  = CalibratorBsp::getBsp();
        auto & uart = bsp.getUartHandler();
        auto & data = uart.getBufferAs<measurement_data>();

        collectMinMaxStatistics(m_CalStatistics.Humidity, data.humidity_counts, true);
      }
      break;
    case UI_STATE_READSENSORLIGHT:
      {
        auto & bsp  = CalibratorBsp::getBsp();
        auto & uart = bsp.getUartHandler();
        auto & data = uart.getBufferAs<measurement_data>();

        collectMinMaxStatistics(m_CalStatistics.Light, data.led_counts, true);
      }
      break;
    case UI_STATE_READSENSORTEMPERATURE:
      {
        auto & bsp  = CalibratorBsp::getBsp();
        auto & uart = bsp.getUartHandler();
        auto & data = uart.getBufferAs<measurement_data>();

        collectTempStatistics(m_CalStatistics.Temperature, data.temp, 263);
        s_CalStatistics = m_CalStatistics;
      }
      break;
    case UI_STATE_RESETSTATISTICS:
      {
        auto & Statistics = m_CalStatistics;
        memset(&Statistics, 0x00, sizeof(Statistics));

        Statistics.Humidity.Min = 0xFFFF;
        Statistics.Light.Min    = 0xFFFF;

        s_CalStatistics = Statistics;
      }
      break;
    case UI_STATE_READSENSOR:
      changeState(APP_STATE_READSENSOR);
      break;
    case UI_STATE_ERASESENSOR:
      {
        auto & data = bsp.getVoltageModulator().getBuffer<calibration_data>();

        s_ResetCalibrationParam.read(data.Humidity);
        s_ResetCalibrationParam.read(data.Light);
        s_ResetCalibrationParam.read(data.Temperature);

        changeState(APP_STATE_WRITESENSOR);
      }
      break;
    case UI_STATE_CALIBRATESENSOR:
      {
        auto & data = bsp.getVoltageModulator().getBuffer<calibration_data>();
        s_CalStatistics.read(m_CalStatistics);
        calculateCalibration();
        memcpy(&data, &m_Scratch.CalPrm, sizeof(data));
        changeState(APP_STATE_WRITESENSOR);
        break;
      }
    default:
      break;
    }
  }


  if(NextState != UI_STATE_STARTUP)
  {
    changeUiState(NextState);
  }

  m_KeyState = newstate;
}

void CalibratorApp::formatMinMax(struct calibration_minmax & cal, uint16_t value)
{
  s_UiLcdRaw.read(m_LcdScratch);

  if(value > cal.Max)
  {
    m_LcdScratch[4] = '>';
    m_LcdScratch[5] = '>';
    String::formatUnsigned(m_LcdScratch + 7, 5, value);
  }
  else if(value < cal.Min)
  {
    m_LcdScratch[4] = '<';
    m_LcdScratch[5] = '<';
    String::formatUnsigned(m_LcdScratch + 7, 5, value);
  }
  else
  {
    Evaluator * pe = reinterpret_cast<Evaluator*>(&cal);

    m_LcdScratch[4]  = '[';
    m_LcdScratch[10] = ']';
    String::formatUnsigned(m_LcdScratch + 5, 5, pe->scale(value));
  }
}

void CalibratorApp::formatLinRegr(struct calibration_linearregression & cal, uint16_t value, uint16_t ref)
{
  s_UiLcdRaw.read(m_LcdScratch);

  String::formatSigned(m_LcdScratch + 4, 3, static_cast<int16_t>(ref)   - 273);
  String::formatSigned(m_LcdScratch + 9, 3, static_cast<int16_t>(value) - 273);
}

void CalibratorApp::changeUiState(enum UiState NextState)
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();

  { /* setup first display line */
    auto & str = s_UiLcdStrings[NextState];
    lcd.displayString(lcd.Location(0,0), str.begin(), str.end());
  }

  /* enter state handling */
  switch(NextState)
  {
  case UI_STATE_READSENSOR:
  case UI_STATE_ERASESENSOR:
  case UI_STATE_CALIBRATESENSOR:
  case UI_STATE_RESETSTATISTICS:
  case UI_STATE_AUTOCALIBRATION:
    lcd.displayString(lcd.Location(0,1), s_UiLcdNextOK.begin(), s_UiLcdNextOK.end());
    break;
  case UI_STATE_READSENSORHUMIDITY:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      formatMinMax(m_CalStatistics.Humidity, data.humidity_counts);
      lcd.displayString(lcd.Location(0,1), m_LcdScratch, m_LcdScratch + 16);
    }
    break;
  case UI_STATE_READSENSORLIGHT:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      formatMinMax(m_CalStatistics.Light, data.led_counts);
      lcd.displayString(lcd.Location(0,1), m_LcdScratch, m_LcdScratch + 16);
    }
    break;
  case UI_STATE_READSENSORTEMPERATURE:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      formatLinRegr(m_CalStatistics.Temperature, data.temp, 263);
      lcd.displayString(lcd.Location(0,1), m_LcdScratch, m_LcdScratch + 16);
    }
    break;
  case UI_STATE_READSENSORRESULTS:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      memset(m_LcdScratch, ' ', sizeof(m_LcdScratch));

      String::formatUnsigned(m_LcdScratch + 0,  5, data.humidity_counts);
      String::formatUnsigned(m_LcdScratch + 6,  5, data.led_counts);
      String::formatSigned(m_LcdScratch  + 12, 3, static_cast<int16_t>(data.temp)   - 273);

      lcd.displayString(lcd.Location(0,1), m_LcdScratch, m_LcdScratch + 16);
    }
    break;
  default:
    break;
  }

  m_MenuState = NextState;
}

void CalibratorApp::handleUiState()
{
  enum UiState CurrentState = static_cast<enum UiState>(m_MenuState);
  auto & bsp  = CalibratorBsp::getBsp();

  switch(CurrentState)
  {
  case UI_STATE_READINGSENSOR:
    if (m_State == APP_STATE_IDLE)
    {
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      if(0xC0 == data.sync && 0x00 == data.type)
      { /* raw data received from sensor */
        s_CalStatistics.read(m_CalStatistics);
        calculateCalibration();

        changeUiState(UI_STATE_READSENSORRAW);
      }
      else if(0xC0 == data.sync && 0x01 == data.type)
      { /* measurement data received from sensor */
        changeUiState(UI_STATE_READSENSORRESULTS);
      }
      else
      {
        changeUiState(UI_STATE_SENSORERROR);
      }
    }
    break;
  case UI_STATE_ERASINGSENSOR:
  case UI_STATE_CALIBRATINGSENSOR:
    if (m_State == APP_STATE_IDLE)
    {
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      if(0xC0 == data.sync && 0x00 == data.type && UI_STATE_ERASINGSENSOR == CurrentState)
      { /* erase was succesfull*/
        changeUiState(UI_STATE_READSENSOR);
      }
      else if(0xC0 == data.sync && 0x01 == data.type && UI_STATE_CALIBRATINGSENSOR == CurrentState)
      { /* calibrate was succesfull */
        changeUiState(UI_STATE_READSENSOR);
      }
      else
      {
        changeUiState(UI_STATE_SENSORERROR);
      }
    }
    break;
  default:
    break;
  }

  if(m_State == APP_STATE_READSENSOR)
  {
    auto & lcd = bsp.getLCD();

    uint_least16_t BaudDivisor = (UBRRH << 8) | UBRRL;
    uint_least32_t BaudRate    = (F_CPU / 16) / (BaudDivisor + 1);

    s_UiLcdBaud.read(m_LcdScratch);
    String::formatUnsigned(m_LcdScratch + 16 - 5 - 5, 5, (uint16_t)BaudRate);
    lcd.displayString(lcd.Location(0,1), m_LcdScratch, m_LcdScratch + 16);
  }
  else if (m_State == APP_STATE_WRITESENSOR)
  {
    auto & vmod   = bsp.getVoltageModulator();
    auto & lcd = bsp.getLCD();

    if(vmod.getTransferring() == 0)
    {/* transfer has not started yet */
      auto & timer = (Globals::getGlobals()).getVoltageModulatorTimer();

      s_UiLcdWait.read(m_LcdScratch);
      String::formatUnsigned(m_LcdScratch + 16 - 2, 2, (timer.getRemainingTime() + 999) / 1000, '0');
    }
    else
    {
      s_UiLcdWriteProgress.read(m_LcdScratch);
      String::formatUnsigned(m_LcdScratch + 16 - 2 - 1 - 2, 2, vmod.getNumTransferred(), '0');
    }

    lcd.displayString(lcd.Location(0,1), m_LcdScratch, m_LcdScratch + 16);
  }
}

void CalibratorApp::changeState(enum AppState NextState)
{
  enum AppState CurrentState = static_cast<enum AppState>(m_State);

  if(CurrentState != NextState)
  {
    auto & bsp = CalibratorBsp::getBsp();
    auto & uart = bsp.getUartHandler();

    if(CurrentState == APP_STATE_POWERUP)
    {
      changeUiState(UI_STATE_STARTUP);
    }

    switch(NextState)
    {
    case APP_STATE_IDLE:
      {
        bsp.setSensorVoltage(bsp.SUPPLY_OFF);
      }
      break;
    case APP_STATE_READSENSOR:
      {
        uart.activate();
        bsp.setSensorVoltage(bsp.SUPPLY_5V);
      }
      break;
    case APP_STATE_WRITESENSOR:
      {
        auto & g      = Globals::getGlobals();
        auto & timer  = g.getVoltageModulatorTimer();
        auto & vmod   = bsp.getVoltageModulator();

        timer.start(10000);
        bsp.setSensorVoltage(bsp.SUPPLY_3V);
        vmod.startTransmission();
      }
      break;
    case APP_STATE_POWERUP:
      break;
    }

    m_State = NextState;
  }
}
void
CalibratorApp::handleState()
{
  enum AppState CurrentState = static_cast<enum AppState>(m_State);
  enum AppState NextState    = CurrentState;

  { /* poll for state changes */
    auto & bsp = CalibratorBsp::getBsp();
    auto & uart = bsp.getUartHandler();

    switch(CurrentState)
    {
    case APP_STATE_READSENSOR:
      {
        uart.handleCyclic();

        if(uart.finished())
        {
          auto & data = uart.getBufferAs<measurement_data>();

          data.humidity_counts = ntoh16(data.humidity_counts);
          data.led_counts      = ntoh16(data.led_counts);
          data.temp            = ntoh16(data.temp);

          NextState = APP_STATE_IDLE;
        }
      }
      break;
    case APP_STATE_WRITESENSOR:
      {
        auto & vmod   = bsp.getVoltageModulator();
        vmod.handleTransmission();

        if(vmod.hasFinished())
        {
          NextState = APP_STATE_READSENSOR;
        }
      }
      break;
    case APP_STATE_IDLE:
      break;
    case APP_STATE_POWERUP:
      NextState = APP_STATE_IDLE;
      break;
    }
  }

  if(NextState != CurrentState)
  {
    changeState(NextState);
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
      //s_LcdStrings.DisplayDataUncalibrated.read(buffer);
      String::formatUnsigned(&buffer[11], 5, value);
    }
    else if (0x01 == pMeasData->type)
    {
      /* calibrated sensor data */
      uint16_t value = pInput[idx];
      //s_LcdStrings.DisplayDataCalibrated.read(buffer);
      String::formatUnsigned(&buffer[11], 5, value);
    }
  }
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
    app.handleUiState();
  }
}


