/*
 * calibrator_bsp.cpp
 *
 *  Created on: 28.07.2015
 *      Author: andi
 */

#include "calibrator_bsp.hpp"

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

class HumidityUartReceiver
{
private:
  static uint8_t  State;
  static uint8_t* Ptr;
public:
  static void startReceive();
};


void CalibratorBsp::init()
{
  /* initialize IO Ports */
  PortLcdControl = PinE.OutHigh | PinRw.OutHigh | PinRs.OutHigh;

  /* Setup timer 0 to count us */
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 1000;

  TCCR1A = 0;
  TCCR1B = _BV(CS10);

  /* enable interrupts */
  TIMSK = _BV(OCIE1A);

  Sys_AVR8::enableInterrupts();

  /* Setup LCD */
  LCDType().init();
}

void CalibratorBsp::handleTimers()
{ /* hopefully we get called every 256 ms */
  const uint8_t Ticks1Ms = m_IsrTicks1ms - m_HandledTicks1ms;
  m_HandledTicks1ms += Ticks1Ms;

  auto & Globals = m_Globals;

  for(uint_fast8_t i = 0; i < ElementCount(Globals.m_Timers); ++i)
  {
    Globals.m_Timers[i].update(Ticks1Ms);
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
  uint8_t status, resh, resl;

  status = UCSRA & (_BV(PE) | _BV(FE)); /* parity & frame error bits */
  resh   = UCSRB & _BV(RXB8); /* 9th bit */
  resl   = UDR;

  /* combine error bits and 9th bit */
  status = (status | resh) >> 1;

  if(bsp.m_UartState == bsp.UART_STATE_RECV)
  {
    auto offset = bsp.m_UartRecvLen;

    if(offset == 0)
    { /* store error bits & 9th bit of first byte*/
      bsp.m_UartStatus0 = status;
    }

    if(offset < sizeof(*(bsp.m_pUartBuffer)))
    {
      /* store error bits in status */
      bsp.m_UartStatus0 |= status << 4;

      (*(bsp.m_pUartBuffer))[offset] = resl;
    }

    offset += 1;

    if(offset == 0)
      offset--;

    bsp.m_UartRecvLen = offset;
    bsp.getUARTTimer().start(20);
  }
}

/** Handle UART Status */
void CalibratorBsp::handleUART()
{
  auto & Timer = getUARTTimer();

  uint8_t state = m_UartState;
  uint8_t  len  = m_UartRecvLen;

  if(len > 0 and UART_STATE_RECV == state and Timer.hasTimedOut())
  {
    /* the first byte transmitted by the sensor is a sync byte
     * with value 0x0C0. (9 bits). Together with the parity bit
     * and the two stop bits this can be used to figure out if
     * we need to adapt the baudrate
     */

    /* prevent further receiving */
    UCSRB &= ~_BV(RXEN);

    uint8_t sync = (*m_pUartBuffer)[0];
    int8_t delta = 0;

    if (len >= 1)
    {
      const uint8_t mask_parity = _BV(PE) >> 1;
      const uint8_t mask_fe     = _BV(FE) >> 1;

      const uint8_t status = m_UartStatus0;

      /* the received sync byte will be evaluated bit by bit
       * according the transmission direction. The farer
       * we get away from the start bit with the receive error
       * the closer we get to the correct divisor
       *
       * Parity and 9th bit is always zero, so the pattern to match is
       *
       * 0b0 0000 0011 00 11
       * Bit counting starts with 0 (including the start bit)
       */
      if(sync & 0x20)
      { /* Bit 7 received as Bit 6 -> too slow */
        delta = -1;
      }
      else if (0 == (sync & 0x40))
      { /* Bit 6 received as Bit 7 -> too fast */
        delta = +1;
      }
      else if(0 == (sync & 0x80))
      { /* bit 9 received as bit 8 -> too slow */
        delta = -1;
      }
      else if (status & 0x01)
      { /* bit 8 received as bit 9 -> too fast */
        delta = +1;
      }
      else if (status & mask_parity)
      { /* bit 11 (sb 0) received as bit 10 (Parity) -> too slow */
        delta = -1;
      }
      else if (status & mask_fe)
      { /* bit 10 (Parity) received as bit 11 (sb0) -> too fast */
        delta = +1;
      }

      if (delta || (status & (mask_parity << 4)))
      { /* we must adapt baud rate or a parity error occured in
         * any received char therefore repeat */
        len = 0;
      }
    }

    /* we should receive at least two characters */
    if (len < 2)
    {
      /* restart receiver with possibly new baud rate divisor */

      if(delta)
      { /* update baud rate divisor */
        uint16_t divisor = (UBRRH << 8) | UBRRL;

        if (divisor > 356 and delta < 0)
        {
          divisor -= 1;
        }
        else if (divisor < 499 and delta > 0)
        {
          divisor += 1;
        }

        UBRRH  = (divisor >> 8) & 0x7F;
        UBRRL  = divisor & 0xFF;
      }

      m_UartRecvLen = 0;
      /* reenable uart */
      UCSRB |= _BV(RXEN);
    }
    else
    { /*receiving finished successfully */
      m_UartState = UART_STATE_DONE;
    }
  }
}


