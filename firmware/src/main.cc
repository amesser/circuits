/*
 *  Copyright 2014 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of USB Display firmware.
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
#include <target/attiny461.hpp>
#include <util/datatypes.hpp>

using namespace Platform::Target::AVR8;
using namespace Platform::Architecture::AVR8;
using namespace Platform::Util::Datatypes;

static const EEPROMBuffer<4*2> DigitMask EEMEM
{{
    (uint8_t)(~0x10), (uint8_t)(~0x00),
    (uint8_t)(~0x04), (uint8_t)(~0x00),
    (uint8_t)(~0x00), (uint8_t)(~0x04),
    (uint8_t)(~0x00), (uint8_t)(~0x01),
}};

static const EEPROMBuffer<10 * 2> SegmentMask EEMEM
{{
  (uint8_t)(~0x62), (uint8_t)(~0xE0),
  (uint8_t)(~0x00), (uint8_t)(~0x60),
  (uint8_t)(~0xA2), (uint8_t)(~0xC0),
  (uint8_t)(~0x82), (uint8_t)(~0xE0),
  (uint8_t)(~0xC0), (uint8_t)(~0x60),
  (uint8_t)(~0xC2), (uint8_t)(~0xA0),
  (uint8_t)(~0xE2), (uint8_t)(~0xA0),
  (uint8_t)(~0x40), (uint8_t)(~0xE0),
  (uint8_t)(~0xE2), (uint8_t)(~0xE0),
  (uint8_t)(~0xC2), (uint8_t)(~0xE0),
}};

/* digit to display, counts from left to right */
static volatile uint8_t NextDigit;
static uint8_t CurrentDigit;

uint8_t Digits[4];

USISPIMaster<100000> usi;

int main() __attribute__((noreturn));

int main()
{
  /* set clock divisor to 1 */
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  usi.initialize(USI::SPI_MODE3);

  /* timer 0 ctc to generate 400 hz irq */
  OCR0A = 78;

  TCCR0A = _BV(CTC0);
  TCCR0B = _BV(CS02);

  TIMSK  = _BV(OCIE0A);

  PORTB  = _BV(PB1) | _BV(PB2) | _BV(PB3);
  DDRB   = _BV(PB1) | _BV(PB2) | _BV(PB3);

  ADMUX  = _BV(REFS1) | _BV(REFS0) | 0x05;
  ADCSRA = _BV(ADEN)  | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  ADCSRB = _BV(REFS2);

  sei();

  while(1)
  {
    if(CurrentDigit == 3)
    {
      uint16_t readout = (ADCW + 2) / 4;

      uint8_t digit = ' ';

      if (readout >= 100)
      {
        digit = '0';

        while(readout >= 100)
        {
            readout -= 100;
          digit += 1;
        }

        digit = '0';
      }

      Digits[0] = digit;

      if (readout >= 10)
      {
        digit = '0';

        while(readout >= 10)
        {
            readout -= 10;
          digit += 1;
        }
      }

      Digits[1] = digit;

      Digits[2] = '0' + readout;

      ADCSRA |= _BV(ADSC);
    }

    if(CurrentDigit != NextDigit)
    {
      CurrentDigit = NextDigit;

      uint8_t index = Digits[CurrentDigit] - '0';
      uint8_t value;

      value = DigitMask[CurrentDigit * 2];

      if(index < 10)
        value &= SegmentMask[index * 2];

      usi.transferByte(value);

      value = DigitMask[CurrentDigit * 2 + 1];

      if(index < 10)
        value &= SegmentMask[index * 2 + 1];

      usi.transferByte(value);

      /* pulse latch line */

      PORTB |= _BV(PB3);
      _delay_us(50);
      PORTB &= ~(_BV(PB3));
    }

    PowerManager::selectSleepMode(PowerManager::SLEEPMODE_IDLE);
    PowerManager::enterSleepMode();
  }
}

ISR(TIMER0_COMPA_vect)
{
  NextDigit = (uint8_t)(CurrentDigit + 1) % (uint8_t)4;
}

ISR(ADC_vect)
{

}
