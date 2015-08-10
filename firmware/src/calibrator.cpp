/*
 * calibrator.cpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#include <ecpp/String.hpp>
#include <string.h>

#include "calibrator_app.hpp"


static const FlashVariable<char, 16> s_TextRead PROGMEM     = "Reading Sensor  ";
static const FlashVariable<char, 16> s_TextReadDone PROGMEM = "Read Sensor     ";
static const FlashVariable<char, 16> s_FormatHex PROGMEM    = "0123456789ABCDEF";

void
CalibratorApp::handleState()
{
  auto & bsp = CalibratorBsp::getBsp();
  auto State = m_State;

  if(State == STATE_POWERUP)
  {
    auto & uart = bsp.getUartHandler();

    uart.activate();
    bsp.setSensorVoltage(bsp.SUPPLY_5V);

    State = STATE_READSENSOR;

    auto & lcd = bsp.getLCD();
    auto & buffer = lcd.getBuffer();

    lcd.moveCursor(0);
    s_TextRead.read(buffer);
    lcd.writeTextBuffer(sizeof(s_TextRead));

  }
  else if(State == STATE_READSENSOR)
  {
    auto & lcd = bsp.getLCD();
    auto & buffer = lcd.getBuffer();

    auto & uart = bsp.getUartHandler();

    uart.handleCyclic();

    if(uart.finished())
    {
      State = STATE_READDONE;

      bsp.setSensorVoltage(bsp.SUPPLY_OFF);

      lcd.moveCursor(0);
      s_TextReadDone.read(buffer);
      lcd.writeTextBuffer(sizeof(s_TextReadDone));
    }

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
    else
    {
      String::formatDecimal(&(buffer[11]), 5, Globals::getGlobals().getUartTimer().getRemainingTime());
    }
    lcd.moveCursor(0x40);
    lcd.writeTextBuffer(16);
  }
  else if(State == STATE_READDONE)
  {

  }

  m_State = State;
}

void
CalibratorApp::readSensor()
{
  auto & bsp = CalibratorBsp::getBsp();

}

static CalibratorApp app;

int main(void) __attribute__ ((OS_main));
int main(void)
{
  auto & bsp = CalibratorBsp::getBsp();

  bsp.init();
  while(1)
  {
    app.handleState();
    bsp.handle();

    {
    }
    /* lcd.moveCursor(0);
     * lcd.writeData(abTest, 8); */
  }
}


