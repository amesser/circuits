/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor Calibrator firmware.
 *
 *  This software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *  */
#include "bsp.hpp"
#include "globals.hpp"

CalibratorBsp CalibratorBsp::s_Instance;

/** LCD Display Initialization command sequence */
const FlashVariable<HD44780_CMD, 5> CalibratorBsp::LcdBsp::InitSequence PROGMEM =
{
   HD44780_CMD_FUNCTIONSET_4BIT | HD44780_CMD_FUNCTIONSET_2LINE | HD44780_CMD_FUNCTIONSET_5x7FONT,
   HD44780_CMD_DISPLAYCONTROL_DISPLAYON,
   HD44780_CMD_ENTRYMODE_INCREMENT | HD44780_CMD_ENTRYMODE_NOSHIFT,
   HD44780_CMD_HOME,
   HD44780_CMD_CLEAR,
};

void CalibratorBsp::init()
{
  /* initialize IO Ports */
  PortLcdControl = PinE.OutHigh  | PinRw.OutHigh | PinRs.OutHigh;
  PortNibble     = PinTXA.OutLow | PinTXB.OutLow | PinTXC.OutLow;
  PortB          = PinLcdContrast.OutLow;

  /* Setup timer 0 to count us */
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 1000;

  TCCR1A = 0;
  TCCR1B = _BV(CS11); /* divide by 8 */

  /* enable interrupts */
  TIMSK = _BV(OCIE1A);

  /* configure adc */
  ADMUX  = _BV(MUX1);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);


  Sys_AVR8::enableInterrupts();

  /* Setup LCD */
  getBsp().getLCD().init();
  getBsp().getUartHandler().init();
  getBsp().getTWIHandler().init(20000);
}

static const uint16_t LEVEL_KEY_A = (uint16_t)(1023. * (0.2/ 5.0 + (1*68. + 68./2) / (3300+1*68)));
static const uint16_t LEVEL_KEY_B = (uint16_t)(1023. * (0.2/ 5.0 + (2*68. + 68./2) / (3300+2*68)));
static const uint16_t LEVEL_KEY_C = (uint16_t)(1023. * (0.2/ 5.0 + (3*68. + 68./2) / (3300+3*68)));

void CalibratorBsp::cycle()
{
  Globals & g = Globals::getGlobals();

  { /* Update all timers, hopefully we get called every 256 ms */
    uint16_t Ticks;
    uint8_t  Milliseconds;

    Sys_AVR8::disableInterupts();

    Ticks = m_IsrTicks;
    Milliseconds = (Ticks & 0xFF) - m_HandledMillisecondTicks;

    if(Milliseconds)
    {
      uint8_t Seconds;

      Ticks        = Ticks - m_HandledSecondTicks;

      if( Ticks >= 1000)
      {
        m_HandledSecondTicks += 1000;
        Seconds = 1;
      }
      else
      {
        Seconds = 0;
      }

      m_HandledMillisecondTicks += Milliseconds;
      g.handleTimers(Milliseconds, Seconds);
    }

    Sys_AVR8::enableInterrupts();
  }

  /* handle key presses */
  if (0 == (ADCSRA & _BV(ADSC)))
  {
    auto & timer = g.getKeyTimer();

    uint_fast8_t  newstate = m_KeyState & 0x0F;
    uint_fast16_t readout  = ADC;

    if(readout < LEVEL_KEY_A)
    {
      newstate |= 0x10;
    }
    else if(readout < LEVEL_KEY_B)
    {
      newstate |= 0x20;
    }
    else if(readout < LEVEL_KEY_C)
    {
      newstate |= 0x40;
    }

    if (newstate != m_KeyState)
    {
      timer.startMilliseconds(50);
      m_KeyState = newstate;
    }
    else
    {
      if(timer.hasTimedOut())
      {
        timer.startMilliseconds(20);

        m_KeyState = (newstate & 0xF0) | ((newstate >> 4) & 0x0F);
        ADCSRA |= _BV(ADSC);
      }
    }

    if((newstate & 0x0F) != ((newstate >> 4) & 0x0F))
    {
      ADCSRA |= _BV(ADSC);
    }


  }

  { /* handle drivers */
    m_TWIMaster.handleCyclic();
    m_Lcd.poll();
  }
}

/**
 *
 * OCR1A will be invoked with 1ms clock
 */
ISR(TIMER1_COMPA_vect)
{
  auto & bsp = CalibratorBsp::getBsp();

  OCR1A  = OCR1A + 1000;
  bsp.m_IsrTicks += 1;
}

/** Store received character and status */
ISR(USART_RXC_vect)
{
  auto & bsp = CalibratorBsp::getBsp();
  bsp.getUartHandler().recvIrq();
}

/** Store received character and status */
ISR(TWI_vect)
{
}

void
CalibratorBsp::setSensorVoltage(uint8_t state)
{
  switch(state)
  {
  case SUPPLY_5V:
    PinTXA = 1;
    PinTXB = 1;
    PinTXC = 1;
    break;
  case SUPPLY_4V:
    PinTXA = 0;
    PinTXB = 1;
    PinTXC = 1;
    break;
  case SUPPLY_3V:
    PinTXA = 0;
    PinTXB = 0;
    PinTXC = 1;
    break;
  default:
    PinTXA = 0;
    PinTXB = 0;
    PinTXC = 0;
    break;
  }
}
