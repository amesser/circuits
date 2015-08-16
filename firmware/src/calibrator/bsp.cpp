/*
 * calibrator_bsp.cpp
 *
 *  Created on: 28.07.2015
 *      Author: andi
 */

#include "bsp.hpp"
#include "globals.hpp"

CalibratorBsp CalibratorBsp::s_Instance;

void  CalibratorBsp::LcdBsp::delay(uint_fast16_t us)
{
  while(us--) _delay_us(1);
}


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
  PortLcdControl = PinE.OutHigh  | PinRw.OutHigh | PinRs.OutHigh | Pin1k8.OutLow | Pin8k2.OutLow;
  PortB          = Pin3k3.OutLow | PinKeyA.InPullUp | PinKeyB.InPullUp;

  /* Setup timer 0 to count us */
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 1000;

  TCCR1A = 0;
  TCCR1B = _BV(CS11); /* divide by 8 */

  /* enable interrupts */
  TIMSK = _BV(OCIE1A);

  Sys_AVR8::enableInterrupts();

  /* Setup LCD */
  getBsp().getLCD().init();
}

void CalibratorBsp::cycle()
{
  Globals & g = Globals::getGlobals();

  { /* Update all timers, hopefully we get called every 256 ms */
    const auto Ticks1Ms = m_IsrTicks1ms - m_HandledTicks1ms;

    if(Ticks1Ms)
    {
      m_HandledTicks1ms += Ticks1Ms;

      Sys_AVR8::disableInterupts();
      g.handleTimers(Ticks1Ms);
      Sys_AVR8::enableInterrupts();
    }
  }

  { /* handle key presses */
    auto & timer = g.getKeyTimer();

    uint_fast8_t newstate = 0;

    if (!PinKeyA)
    {
      newstate |= 0x10;
    }

    if (!PinKeyB)
    {
      newstate |= 0x20;
    }

    if ((m_KeyState & 0xF0) != newstate)
    {
      timer.start(50);
      m_KeyState = newstate | (m_KeyState & 0x0F);
    }
    else
    {
      if(timer.hasTimedOut())
      {
        m_KeyState = newstate | ((newstate >> 4) & 0x0F);
      }
    }
  }
}


/**
 *
 * OCR1A will be invoked with 1ms clock
 */
ISR(TIMER1_COMPA_vect)
{
  auto & bsp = CalibratorBsp::getBsp();

  OCR1A = OCR1A + 1000;
  bsp.m_IsrTicks1ms += 1;
}

/** Store received character and status */
ISR(USART_RXC_vect)
{
  auto & bsp = CalibratorBsp::getBsp();
  bsp.getUartHandler().recvIrq();
}



void
CalibratorBsp::setSensorVoltage(uint8_t state)
{
  switch(state)
  {
  case SUPPLY_5V:
    Pin1k8 = 1;
    Pin3k3.disableOutput();
    Pin8k2.disableOutput();
    break;
  case SUPPLY_4V:
    Pin1k8 = 1;
    Pin3k3.disableOutput();
    Pin3k3.enableOutput();
    break;
  case SUPPLY_3V:
    Pin1k8 = 1;
    Pin3k3.enableOutput();
    Pin8k2.disableOutput();
    break;
  default:
    Pin1k8 = 0;
    Pin3k3.disableOutput();
    Pin8k2.disableOutput();
    break;
  }
}
