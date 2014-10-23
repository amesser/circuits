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

#include <string.h>
using namespace Platform::Target::AVR8;
using namespace Platform::Architecture::AVR8;
using namespace Platform::Util::Datatypes;

enum segment_id
{
  segment_a  = 0,
  segment_b  = 1,
  segment_c  = 2,
  segment_d  = 3,
  segment_e  = 4,
  segment_f  = 5,
  segment_g  = 6,
  segment_dp = 7,
};

enum digit_id
{
  digit_1    = 0,
  digit_2    = 1,
  digit_3    = 2,
  digit_4    = 3,
  digit_5    = 4,
  digit_6    = 5,
  digit_7    = 6,
  digit_8    = 7,
};

enum segment_mask
{
  segment_mask_a  = (0x1 << 0),
  segment_mask_b  = (0x1 << 1),
  segment_mask_c  = (0x1 << 2),
  segment_mask_d  = (0x1 << 3),
  segment_mask_e  = (0x1 << 4),
  segment_mask_f  = (0x1 << 5),
  segment_mask_g  = (0x1 << 6),
  segment_mask_dp = (0x1 << 7),
};

struct eeprom_data
{
  EEPROMBuffer<8, uint8_t> segment_bits;

  EEPROMBuffer<8, uint8_t> digit_bits;

  EEPROMBuffer<10, uint8_t> digit_segments;
};

static const struct eeprom_data eeprom EEMEM =
{
   {{7, 6, 5, 9, 13, 14, 15, 2}},

   {{12, 10, 2, 0, 0xff, 0xff, 0xff, 0xff}},

   {{
     segment_mask_a | segment_mask_b | segment_mask_c | segment_mask_d | segment_mask_e | segment_mask_f,
                      segment_mask_b | segment_mask_c,
     segment_mask_a | segment_mask_b                  | segment_mask_d | segment_mask_e                  | segment_mask_g,
     segment_mask_a | segment_mask_b | segment_mask_c | segment_mask_d                                   | segment_mask_g,
                      segment_mask_b | segment_mask_c                                   | segment_mask_f | segment_mask_g,
     segment_mask_a                  | segment_mask_c | segment_mask_d                  | segment_mask_f | segment_mask_g,
     segment_mask_a                  | segment_mask_c | segment_mask_d | segment_mask_e | segment_mask_f | segment_mask_g,
     segment_mask_a | segment_mask_b | segment_mask_c                                   | segment_mask_f,
     segment_mask_a | segment_mask_b | segment_mask_c | segment_mask_d | segment_mask_e | segment_mask_f | segment_mask_g,
     segment_mask_a | segment_mask_b | segment_mask_c | segment_mask_d                  | segment_mask_f | segment_mask_g,
   }}
};

template<class HW>
class panelmeter_app : public HW
{
private:
  uint8_t          num_digits;
  uint16_t         digit_masks[8];
  uint16_t         number_masks[10];

  volatile uint8_t update_digit;
  volatile uint8_t display_digit;

  uint8_t         digits[8];

  constexpr uint16_t genMask(uint8_t bit) {return ((uint16_t)0x1 << bit);}

public:
  template<typename CFG>
  void initialize(const CFG & config)
  {
    HW::initialize();

    uint8_t seg, seg_mask, index;

    memset(number_masks, 0xFF, sizeof(number_masks));
    memset(digits, ' ', sizeof(digits));

    for(seg = 0, seg_mask = 0x01; seg < 8; seg++, seg_mask <<= 1)
    {
      const uint16_t mask = ~(genMask(eeprom.segment_bits[seg]));

      for (index = 0; index < 10; ++index)
      {
        if(eeprom.digit_segments[index] & seg_mask)
          number_masks[index] &= mask;
      }
    }

    for(num_digits = 0; num_digits < 8; num_digits++)
    {
      const uint8_t bit = eeprom.digit_bits[num_digits];

      if(bit < 16)
        digit_masks[num_digits] = ~(genMask(bit));
      else
        break;
    }

    update_digit  = num_digits;
    display_digit = num_digits;
  }

  void
  update_display()
  {
    uint8_t digit = update_digit;

    if(digit != display_digit)
    {
      display_digit = digit;

      uint8_t  index = digits[digit] - '0';
      uint16_t value;

      value = digit_masks[digit];

      if(index < 10)
        value &= number_masks[index];

      HW::setBitmask(value);
    }
  }

  void show_value(uint16_t value)
  {
    uint8_t index = 0;
    uint8_t digit = ' ';

    if (value >= 1000)
    {
      digit = '0';

      while(value >= 1000)
      {
        value -= 1000;
        digit += 1;
      }
    }

    digits[index++] = digit;

    if (value >= 100 || digit != ' ')
    {
      digit = '0';

      while(value >= 100)
      {
        value -= 100;
        digit += 1;
      }
    }

    digits[index++] = digit;

    if (value >= 10 || digit != ' ')
    {
      digit = '0';

      while(value >= 10)
      {
        value -= 10;
        digit += 1;
      }
    }

    digits[index++] = digit;
    digits[index++] = '0' + value;
  }

  void timeout()
  {
    uint8_t digit = update_digit + 1;

    if (digit < num_digits)
      update_digit = digit;
    else
      update_digit = 0;
  }

  void run()
  {
    while(1)
    {
      update_display();

      if(display_digit == num_digits - 1)
      {
        uint16_t value = HW::getValue(0);
        show_value(value * 2 + value / 2);
      }

      HW::sleep();
    }
  }
};

class Hardware_rev1
{
private:
  typedef USISPIMaster<100000> usi_type;

public:
  void initialize()
  {
    /* set clock divisor to 1 */
    CLKPR = _BV(CLKPCE);
    CLKPR = 0;

    usi_type::initialize(USI::SPI_MODE3);

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
  }

  static void
  setBitmask(uint16_t value)
  {
    /* shift mask into shift registers */
    usi_type::transferBytes((value >> 8) & 0xFF, value & 0xFF);

    /* pulse latch line */

    PORTB |= _BV(PB3);
    _delay_us(50);
    PORTB &= ~(_BV(PB3));
  }

  static uint16_t
  getValue(uint8_t channel)
  {
    uint16_t value = ADCW;

    ADCSRA |= _BV(ADSC);

    return value;
  }

  static void
  sleep()
  {
    PowerManager::selectSleepMode(PowerManager::SLEEPMODE_IDLE);
    PowerManager::enterSleepMode();
  }
};

static panelmeter_app<Hardware_rev1> app;

int main() __attribute__((noreturn));

int main()
{
  app.initialize(eeprom);
  app.run();
}

ISR(TIMER0_COMPA_vect)
{
  app.timeout();
}

ISR(ADC_vect)
{

}
