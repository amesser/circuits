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

#define MAX_HUMJUMP   100
#define MAX_LIGHTJUMP 200

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

static constexpr LcdStringType s_UiLcdTemp PROGMEM =
  "   XXX% Rh XXX" "\xdf" "C";

static constexpr LcdStringType s_UiLcdAutoWait PROGMEM =
  "W XXX S XX XXX" "\xdf" "C";

static constexpr LcdStringType s_UiLcdInterval PROGMEM =
  "[S] XXX Min [OK]";

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
  CalibratorApp::UI_STATE_AUTOCALIBRATION,       /* UI_STATE_AUTOREADINGSENSOR */
  CalibratorApp::UI_STATE_AUTOCALIBRATION,       /* UI_STATE_AUTOWAIT */
  CalibratorApp::UI_STATE_STARTUP,               /* UI_STATE_STARTUP */
};

static constexpr FlashVariable<uint_least16_t> s_AutoIntervals[] PROGMEM =
{
  5*60,
  10*60,
  15*60,
  30*60,
  60*60,
  90*60,
  120*60,
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
    case UI_STATE_AUTOSELECTINTERVAL:
      {
        if (m_AutoIntervalSelector < (ElementCount(s_AutoIntervals) - 1))
        {
          m_AutoIntervalSelector += 1;
        }
        else
        {
          m_AutoIntervalSelector  = 0;
        }
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

        CalibratorFit::collectMinMaxStatistics(m_CalStatistics.Humidity, data.humidity_counts);
      }
      break;
    case UI_STATE_READSENSORLIGHT:
      {
        auto & bsp  = CalibratorBsp::getBsp();
        auto & uart = bsp.getUartHandler();
        auto & data = uart.getBufferAs<measurement_data>();

        CalibratorFit::collectMinMaxStatistics(m_CalStatistics.Light, data.led_counts);
      }
      break;
    case UI_STATE_READSENSORTEMPERATURE:
      {
        auto & bsp  = CalibratorBsp::getBsp();
        auto & uart = bsp.getUartHandler();
        auto & data = uart.getBufferAs<measurement_data>();

        CalibratorFit::collectTempStatistics(m_CalStatistics.Temperature, data.temp, m_AbsTemp);
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
    case UI_STATE_AUTOCALIBRATION:
      {
        m_AutoIntervalSelector = 0;
      }
      break;
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

void CalibratorApp::formatMinMax(LcdRowBufferType & Buffer, struct CalibratorFit::calibration_minmax & cal, uint16_t value)
{
  s_UiLcdRaw.read(Buffer);

  if(value > cal.Max)
  {
    Buffer[4] = '>';
    Buffer[5] = '>';
    String::formatUnsigned(Buffer + 7, 5, value);
  }
  else if(value < cal.Min)
  {
    Buffer[4] = '<';
    Buffer[5] = '<';
    String::formatUnsigned(Buffer + 7, 5, value);
  }
  else
  {
    Evaluator * pe = reinterpret_cast<Evaluator*>(&cal);

    Buffer[4]  = '[';
    Buffer[10] = ']';
    String::formatUnsigned(Buffer + 5, 5, pe->scale(value));
  }
}

void CalibratorApp::formatLinRegr(LcdRowBufferType & Buffer, struct CalibratorFit::calibration_linearregression & cal, uint16_t value, uint16_t ref)
{
  s_UiLcdRaw.read(Buffer);

  String::formatSigned(Buffer + 4, 3, static_cast<int16_t>(ref)   - 273);
  String::formatSigned(Buffer + 9, 3, static_cast<int16_t>(value) - 273);
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

      formatMinMax(bsp.getDisplayRow(1), m_CalStatistics.Humidity, data.humidity_counts);
    }
    break;
  case UI_STATE_READSENSORLIGHT:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      formatMinMax(bsp.getDisplayRow(1),m_CalStatistics.Light, data.led_counts);
    }
    break;
  case UI_STATE_READSENSORTEMPERATURE:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      formatLinRegr(bsp.getDisplayRow(1),m_CalStatistics.Temperature, data.temp, m_AbsTemp);
    }
    break;
  case UI_STATE_READSENSORRESULTS:
    {
      auto & bsp  = CalibratorBsp::getBsp();
      auto & uart = bsp.getUartHandler();
      auto & data = uart.getBufferAs<measurement_data>();

      auto & Buffer = bsp.getDisplayRow(1);

      memset(Buffer, ' ', sizeof(Buffer));

      String::formatUnsigned(Buffer + 0,  5, data.humidity_counts);
      String::formatUnsigned(Buffer + 6,  5, data.led_counts);
      String::formatSigned(Buffer  + 12, 3, static_cast<int16_t>(data.temp)   - 273);
    }
    break;
  case UI_STATE_AUTOREADINGSENSOR:
    {
      uint16_t Timeout = s_AutoIntervals[m_AutoIntervalSelector];

      changeState(APP_STATE_READSENSOR);
      Globals::getGlobals().getAutomaticTimer().startSeconds(Timeout);
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
  case UI_STATE_AUTOREADINGSENSOR:
    {
      if (m_State == APP_STATE_IDLE)
      {
        auto & uart = bsp.getUartHandler();
        auto & data = uart.getBufferAs<measurement_data>();

        if(0xC0 == data.sync && 0x00 == data.type)
        { /* raw data received from sensor */
          s_CalStatistics.read(m_CalStatistics);

          if(data.humidity_counts > m_LastHum && (data.humidity_counts - m_LastHum) < MAX_HUMJUMP)
          {
            CalibratorFit::collectMinMaxStatistics(m_CalStatistics.Humidity, data.humidity_counts);
          }
          else if(data.humidity_counts < m_LastHum && (m_LastHum - data.humidity_counts) < MAX_HUMJUMP)
          {
            CalibratorFit::collectMinMaxStatistics(m_CalStatistics.Humidity, data.humidity_counts);
          }

          m_LastHum = data.humidity_counts;

          if(data.led_counts > m_LastLight && (data.led_counts - m_LastLight) < MAX_LIGHTJUMP)
          {
            CalibratorFit::collectMinMaxStatistics(m_CalStatistics.Light, data.led_counts);
          }
          else if(data.led_counts < m_LastLight && (m_LastLight - data.led_counts) < MAX_LIGHTJUMP)
          {
            CalibratorFit::collectMinMaxStatistics(m_CalStatistics.Light, data.led_counts);
          }

          m_LastLight = data.led_counts;

          if(m_StableCnt > (12 * 5))
          { /* temperature should be stable for at least 5 minutes */
              CalibratorFit::collectTempStatistics(m_CalStatistics.Temperature, data.temp, m_AbsTemp);
          }

          s_CalStatistics = m_CalStatistics;

          changeUiState(UI_STATE_AUTOWAIT);
        }
        else if(0xC0 == data.sync && 0x01 == data.type)
        { /* measurement data received from sensor */
          changeUiState(UI_STATE_SENSORERROR);
        }
        else
        { /* failure reading the senosr, ignore */
          changeUiState(UI_STATE_AUTOWAIT);
        }
      }
    }
    break;
  case UI_STATE_AUTOWAIT:
    {
      if (m_State == APP_STATE_IDLE)
      {
        if(Globals::getGlobals().getAutomaticTimer().hasTimedOut())
        {
          changeUiState(UI_STATE_AUTOREADINGSENSOR);
        }
      }
    }
    break;
  default:
    break;
  }

  if(m_State == APP_STATE_READSENSOR)
  {
    auto & Buffer = bsp.getDisplayRow(1);

    uint_least16_t BaudDivisor = (UBRRH << 8) | UBRRL;
    uint_least32_t BaudRate    = (F_CPU / 16) / (BaudDivisor + 1);

    s_UiLcdBaud.read(Buffer);

    if(1)
    {
      auto & uart = bsp.getUartHandler();
      String::formatSigned(Buffer   + 5,         2, uart.getLastDelta());
      String::formatUnsigned(Buffer + 5 + 3,     3, uart.getSequence());
      String::formatUnsigned(Buffer + 5 + 3 + 4, 3, BaudDivisor);
    }
    else
    {
      String::formatUnsigned(Buffer + 16 - 5 - 5, 5, (uint16_t)BaudRate);
    }
  }
  else if (m_State == APP_STATE_WRITESENSOR)
  {
    auto & vmod   = bsp.getVoltageModulator();
    auto & Buffer = bsp.getDisplayRow(1);

    if(vmod.getTransferring() == 0)
    {/* transfer has not started yet */
      auto & timer = (Globals::getGlobals()).getVoltageModulatorTimer();

      s_UiLcdWait.read(Buffer);
      String::formatUnsigned(Buffer + 16 - 2, 2, (timer.getRemainingMilliseconds() + 999) / 1000, '0');
    }
    else
    {
      s_UiLcdWriteProgress.read(Buffer);
      String::formatUnsigned(Buffer + 16 - 2 - 1 - 2, 2, vmod.getNumTransferred(), '0');
    }
  }
  else if (CurrentState == UI_STATE_AUTOSELECTINTERVAL)
  {
    auto & Buffer = bsp.getDisplayRow(1);

    s_UiLcdInterval.read(Buffer);
    uint16_t Minutes = s_AutoIntervals[m_AutoIntervalSelector] / 60;
    String::formatUnsigned(Buffer + 4, 3, Minutes);
  }

}

void CalibratorApp::changeState(enum AppState NextState)
{
  enum AppState CurrentState = static_cast<enum AppState>(m_State);

  if(CurrentState != NextState)
  {
    auto & bsp = CalibratorBsp::getBsp();
    auto & uart = bsp.getUartHandler();

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

        /* trigger a refresh of the temperatur */
        Globals::getGlobals().getTemperatureTimer().stop();
      }
      break;
    case APP_STATE_WRITESENSOR:
      {
        auto & g      = Globals::getGlobals();
        auto & timer  = g.getVoltageModulatorTimer();
        auto & vmod   = bsp.getVoltageModulator();

        timer.startMilliseconds(10000);
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
    case APP_STATE_POWERUP:
      {
        auto & Buffer = bsp.getDisplayRow(1);
        memset(Buffer, '0', sizeof(Buffer));
      }

      NextState = APP_STATE_IDLE;
      changeUiState(UI_STATE_STARTUP);
      break;
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
    }
  }

  if(NextState != CurrentState)
  {
    changeState(NextState);
  }
}

void CalibratorApp::calculateCalibration()
{
  auto & Stat  = m_CalStatistics;
  auto & Param = m_Scratch.CalPrm;

  CalibratorFit::calculateMinMaxCal(Stat.Humidity, Param.Humidity);
  CalibratorFit::calculateMinMaxCal(Stat.Light, Param.Light);
  CalibratorFit:: calculateLinRegr(Stat.Temperature, Param.Temperature);
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

#define HDC1000_ADDRESS (0x43)

const char HexChars[] = "0123456789abcdef";

void CalibratorApp::changeTempState(enum TempState NextState)
{
  auto & bsp    = CalibratorBsp::getBsp();
  auto & twi    = bsp.getTWIHandler();

  switch(NextState)
  {
  case TEMP_STATE_POWERON:
    {
      twi.close();
      twi.sendStop();
    }
    break;
  case TEMP_STATE_WSETUP:
    {
      auto & buffer = twi.getBuffer();
      buffer[0] = 0x02; /* register 2: configuration */
      buffer[1] = 0x10;
      buffer[2] = 0x00;

      twi.sendStartAndWrite(HDC1000_ADDRESS, 3);
      Globals::getGlobals().getTemperatureTimer().startSeconds(5);
    }
    break;
  case TEMP_STATE_WTRIGGER:
    {
      auto & buffer = twi.getBuffer();
      buffer[0] = 0x00; /* register 0: trigger*/
      twi.sendStartAndWrite(HDC1000_ADDRESS, 1);
      Globals::getGlobals().getTemperatureTimer().startSeconds(5);
    }
    break;
  case TEMP_STATE_WMEASUREMENT:
    {
      twi.sendStartAndRead(HDC1000_ADDRESS, 4);
    }
    break;
  case TEMP_STATE_WTIMEOUT:
    {
      twi.sendStop();

    }
    break;
  }

  if(TEMP_STATE_WTIMEOUT == NextState)
  {
    if(m_MenuState == UI_STATE_STARTUP ||
       m_MenuState == UI_STATE_AUTOWAIT)
    {
      auto & Buffer = bsp.getDisplayRow(1);

      if(m_MenuState == UI_STATE_AUTOWAIT)
      {
        s_UiLcdAutoWait.read(Buffer);

        String::formatUnsigned(Buffer + 2, 3, Globals::getGlobals().getAutomaticTimer().getRemainingSeconds<uint16_t>() / 60);
        String::formatUnsigned(Buffer + 8, 2, m_StableCnt / 12);
      }
      else
      {
        s_UiLcdTemp.read(Buffer);
        String::formatUnsigned(Buffer + 3, 3, m_Rh);
      }
      String::formatSigned  (Buffer + 16 - 5, 3, m_AbsTemp - 273);
    }
  }
  else
  {
    if(m_MenuState == UI_STATE_STARTUP)
    {
      auto & Buffer = bsp.getDisplayRow(1);

      if(Buffer[NextState] < '9')
      {
        Buffer[NextState] += 1;
      }

      Buffer[15] = '0' + NextState;

      uint8_t status = twi.getStatus();
      Buffer[12] = HexChars[(status >> 4) & 0xF];
      Buffer[13] = HexChars[(status >> 0) & 0xF];
    }
  }

  m_TempState = NextState;
}

void CalibratorApp::handleTempState()
{
  auto & bsp    = CalibratorBsp::getBsp();
  auto & twi    = bsp.getTWIHandler();

  enum TempState state = static_cast<enum TempState>(m_TempState);

  switch(state)
  {
  case TEMP_STATE_POWERON:
    {
      m_StableCnt = 0;
      changeTempState(TEMP_STATE_WSETUP);
    }
    break;
  case TEMP_STATE_WSETUP:
    {
      if(twi.hasFinished())
      {
        if (twi.getError())
        {
          changeTempState(TEMP_STATE_POWERON);
        }
        else
        {
          changeTempState(TEMP_STATE_WTRIGGER);
        }
      }
      else if(Globals::getGlobals().getTemperatureTimer().hasTimedOut())
      {
        changeTempState(TEMP_STATE_POWERON);
      }
    }
    break;
  case TEMP_STATE_WTRIGGER:
    {
      if(twi.hasFinished())
      {
        if (twi.getError())
        {
          changeTempState(TEMP_STATE_POWERON);
        }
        else
        {
          changeTempState(TEMP_STATE_WMEASUREMENT);
        }
      }
      else if(Globals::getGlobals().getTemperatureTimer().hasTimedOut())
      {
        changeTempState(TEMP_STATE_POWERON);
      }

    }
    break;
  case TEMP_STATE_WMEASUREMENT:
    {
      if(twi.hasFinished())
      {
        if (twi.hasNack())
        {
          twi.sendStopStartAndRead(HDC1000_ADDRESS, 4);
        }
        else if (twi.getError())
        {
          changeTempState(TEMP_STATE_POWERON);
        }
        else
        {
          changeTempState(TEMP_STATE_WTIMEOUT);

          auto & buffer = twi.getBuffer();

          {
            uint32_t Calc32;
            uint16_t  Temp;

            Calc32 = (uint16_t)buffer[0] << 8 | buffer[1];
            Calc32 = Calc32 * 165 + 0x8000;
            Calc32 = (Calc32 & 0xFFFF0000UL) >> 16;

            Temp = (Calc32 & 0xFF) - 40 + 273;

            if(Temp != m_AbsTemp)
            {
              m_AbsTemp      = Temp;
              m_StableCnt = 0;
            }
            else
            {
              if(m_StableCnt < 255)
              {
                m_StableCnt += 1;
              }
            }

            Calc32 = (uint16_t)buffer[2] << 8 | buffer[3];
            Calc32 = Calc32 * 100 + 0x8000;
            Calc32 = (Calc32 & 0xFFFF0000UL) >> 16;

            m_Rh = (Calc32 & 0xFF);
          }
        }
      }
      else if(Globals::getGlobals().getTemperatureTimer().hasTimedOut())
      {
        changeTempState(TEMP_STATE_POWERON);
      }
    }
    break;
  case TEMP_STATE_WTIMEOUT:
    {
      if(Globals::getGlobals().getTemperatureTimer().hasTimedOut())
      {
        changeTempState(TEMP_STATE_WTRIGGER);
      }
      break;
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
    app.handleKeys();
    app.handleState();
    app.handleUiState();
    app.handleTempState();
    bsp.cycle();
  }
}

