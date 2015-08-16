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
  const LcdStringType MenuitemReset;
  const LcdStringType MenuitemCalibrate;
  const LcdStringType StatusMeasuring;
  const LcdStringType StatusRead;
  const LcdStringType StatusErasing;
  const LcdStringType StatusCalibrating;
  const LcdStringType StatusInitTransfer;
  const LcdStringType StatusTransfer;
};

static constexpr struct LcdStrings s_LcdStrings PROGMEM =
{
  .StatusReady        = "Bereit          ",
  .MenuitemMeasure    = "Sensor auslesen ",
  .MenuitemReset      = "Sensor zur" "\xf5" "cksetzen",
  .MenuitemCalibrate  = "Sensor kalibrieren",
  .StatusMeasuring    = "Lese Sensor     ",
  .StatusRead         = "Sensor gelesen  ",
  .StatusErasing      = "L" "\xef" "sche Sensor   ",
  .StatusCalibrating  = "Kalibriere Sensor",
  .StatusInitTransfer = "Initialisiere...",
  .StatusTransfer     = "\xf5" "bertrage  XX/24",
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
static constexpr struct StateHandler s_CalibratorAppStateHandlers[CalibratorApp::APP_STATE_MAX] PROGMEM =
{
  {&CalibratorApp::leaveStatePowerUp},
  {0,                                    &CalibratorApp::enterStateIdle},
  {&CalibratorApp::leaveStateReadSensor, &CalibratorApp::enterStateReadSensor,  &CalibratorApp::pollStateReadSensor},
  {0,                                    &CalibratorApp::enterStateWriteSensor, &CalibratorApp::pollStateWriteSensor},
};

void CalibratorApp::changeState(enum AppState NextState)
{
  m_MenuState = MENU_STATE_IDLE;

  invoke(s_CalibratorAppStateHandlers[m_State].leaveState);
  invoke(s_CalibratorAppStateHandlers[NextState].enterState);

  m_State = NextState;
}


struct MenuStateHandler
{
  const FlashVariable<uint8_t>                LcdStringOffset;
  const FlashVariable<CalibratorApp::Handler> updateMenu;
};

static constexpr struct MenuStateHandler
s_CalibratorMenuStateHandlers[CalibratorApp::MENU_STATE_IDLE] PROGMEM =
{
  {offsetof(struct LcdStrings, MenuitemMeasure), 0},
  {offsetof(struct LcdStrings, MenuitemReset),   0},
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
        p + s_CalibratorMenuStateHandlers[NextState].LcdStringOffset);

    pString->read(buffer);
    lcd.moveCursor(0);
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
  }

  m_KeyState = newstate;
}


void
CalibratorApp::handleState()
{
  if(m_State == APP_STATE_POWERUP)
  {
    changeState(APP_STATE_IDLE);
  }
  else
  {
    invoke(s_CalibratorAppStateHandlers[m_State].pollState);
  }
}

void
CalibratorApp::readSensor()
{
  auto & bsp = CalibratorBsp::getBsp();

}

void CalibratorApp::leaveStatePowerUp()
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

void CalibratorApp::enterStateIdle()
{
  auto & bsp = CalibratorBsp::getBsp();

  bsp.setSensorVoltage(bsp.SUPPLY_OFF);
}

void CalibratorApp::leaveStateReadSensor()
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & uart = bsp.getUartHandler();

  s_LcdStrings.StatusRead.read(buffer);
  lcd.moveCursor(0);
  lcd.writeTextBuffer(16);
}

void CalibratorApp::enterStateReadSensor()
{
  auto & bsp = CalibratorBsp::getBsp();
  auto & lcd = bsp.getLCD();
  auto & buffer = lcd.getBuffer();
  auto & uart = bsp.getUartHandler();

  uart.activate();
  bsp.setSensorVoltage(bsp.SUPPLY_5V);

  s_LcdStrings.StatusMeasuring.read(buffer);
  lcd.moveCursor(0);
  lcd.writeTextBuffer(16);
}

void
CalibratorApp::pollStateReadSensor()
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
    changeState(APP_STATE_IDLE);

    s_LcdStrings.StatusRead.read(buffer);
    lcd.moveCursor(0);
    lcd.writeTextBuffer(16);
  }
}

void CalibratorApp::enterStateWriteSensor()
{
  auto & g   = Globals::getGlobals();
  auto & bsp = CalibratorBsp::getBsp();
  auto & timer  = g.getVoltageModulatorTimer();
  auto & vmod   = bsp.getVoltageModulator();

  timer.start(10000);
  bsp.setSensorVoltage(bsp.SUPPLY_3V);
  vmod.startTransmission();
}

void CalibratorApp::pollStateWriteSensor()
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


