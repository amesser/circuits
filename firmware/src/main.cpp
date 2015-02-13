/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  ATTiny2313 RS232 Infrared Receiver
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General  *  Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  */
#include <target/attiny2313.hpp>
#include <generic/buffer.hpp>

using Platform::Buffer::Ringbuffer;
using Platform::Target::AVR8::PowerManager;

Ringbuffer<uint8_t, 64> s_UartTransmittBuffer;

ISR(TIMER1_CAPT_vect)
{
  PowerManager::preventSleep();

  if((GPIOR0 & 0x01) == 0)
  {
    GPIOR0  |= 0x01;
    GPIOR2   = GPIOR1;

    TCNT1 = 0;
    GPIOR1  = 0;
  }
}

ISR(TIMER1_OVF_vect)
{
  PowerManager::preventSleep();

  uint8_t bValue = GPIOR1 + 1;

  if(bValue < 0x80)
    GPIOR1 = bValue;
  else
    GPIOR0  |= 0x02;
}


ISR(USART_TX_vect)
{
  PowerManager::preventSleep();
}

static void
updateEdge()
{
  /* configure capture edge according current pin state */
  if(PIND & _BV(PIN6))
  {
    TCCR1B &= ~_BV(ICES1);
  }
  else
  {
    TCCR1B |=  _BV(ICES1);
  }
}

int main()
{
  PowerManager::selectSleepMode(PowerManager::SLEEPMODE_IDLE);

  DDRD  = _BV(PIN1);
  PORTD = _BV(PIN6);

  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  UCSRC = _BV(UCSZ1) | _BV(UCSZ0) | _BV(UPM1);
  UCSRB = _BV(TXCIE) | _BV(TXEN);
  UBRRH =  0;
  UBRRL = 12;

  TIFR    = 0xFF;
  TIMSK   = _BV(TOIE1) | _BV(ICIE1);
  TCCR1B  = _BV(ICNC1) | _BV(CS11);

  updateEdge();

  while(true)
  {
    PowerManager::prepareSleep();

    if(GPIOR0 & 0x01)
    { /* toggle occured */
      if((s_UartTransmittBuffer.getSize() - s_UartTransmittBuffer.getCount()) >= 3)
      {
        uint8_t bValue;

        /* set msb according state */
        if(TCCR1B & _BV(ICES1))
        {
          bValue = 0x00;
        }
        else
        {
          bValue = 0x80;
        }

        bValue |= GPIOR2;
        s_UartTransmittBuffer.pushForced(bValue);


        if(GPIOR0 & 0x02)
        {
          s_UartTransmittBuffer.pushForced(0xFF);
          s_UartTransmittBuffer.pushForced(0xFF);
        }
        else
        {
          uint16_t usValue = ICR1;
          s_UartTransmittBuffer.pushForced((usValue >> 8) & 0xFF);
          s_UartTransmittBuffer.pushForced( usValue     & 0xFF);
        }
      }

      GPIOR0 = 0;

      updateEdge();
    }

    if((UCSRA & _BV(UDRE)) && (s_UartTransmittBuffer.getCount() > 0))
    {
      UDR = s_UartTransmittBuffer.popForced();
      PowerManager::preventSleep();
    }

    sei();

    PowerManager::enterSleep();
  }
}
