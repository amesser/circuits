/*
 *  Copyright 2013 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of SDCard Data Logger firmware.
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
#include <target/attiny2313.hpp>
#include <generic/buffer.hpp>
#include <devices/mcp3204.hpp>
#include <devices/sdcard.hpp>
#include <util/datatypes.hpp>

#include "interface.hpp"

using namespace Platform::Architecture::AVR8;
using namespace Platform::Target::AVR8;
using namespace Platform::Buffer;
using namespace Platform::Devices;
using namespace Platform::Util::Datatypes;

class SPISlaveA
{
public:
  __attribute__((always_inline)) static void enablePowerSupply()  {PORTB &= ~_BV(PIN2); delayPowerSupply();}
  __attribute__((always_inline)) static void disablePowerSupply() {PORTB |= _BV(PIN2);  delayPowerSupply();}
  __attribute__((always_inline)) static void selectChip()         {DDRD  |= _BV(PIN3);}
  __attribute__((always_inline)) static void deselectChip()       {DDRD  &= ~_BV(PIN3);}
  __attribute__((noinline))      static void delayPowerSupply()   {_delay_ms(200);}

  USISPIMaster<400000>   getSlowTransport()                { return USISPIMaster<400000>(); }
  USISPIMaster<400000> getTransport(const unsigned long) { return USISPIMaster<400000>(); }
};

class SPISlaveB
{
public:
  __attribute__((always_inline)) static void selectChip()         {PORTD &= ~_BV(PIN2);}
  __attribute__((always_inline)) static void deselectChip()       {PORTD |=  _BV(PIN2);}

  USISPIMaster<400000> getTransport(const unsigned long) { return USISPIMaster<400000>(); }
};


uint8_t readEEPROM(const uint8_t *ptr) {
  EEAR = static_cast<uint8_t>(reinterpret_cast<uint16_t>(ptr));
  EECR |= _BV(EERE);
  return EEDR;
}

class RegisterConfig {
public:
  uint8_t _u8RegNr;
  uint8_t _u8RegValue;

  constexpr RegisterConfig(volatile uint8_t &reg, const uint8_t value) :
       _u8RegNr{static_cast<uint8_t>(_SFR_IO_ADDR(reg))}, _u8RegValue{value} {}

  void writeRegister() const{
    const uint8_t *addr = reinterpret_cast<const uint8_t*>(this);
    const uint8_t ioreg = readEEPROM(addr);
    const uint8_t value = readEEPROM(addr+1);

    _SFR_IO8(ioreg) = value;
  }
};


constexpr uint8_t  timer0_max = ((F_CPU / 8) / 20000);
constexpr uint16_t timer1_max = static_cast<uint16_t>(0.001 * (F_CPU / 256.));

constexpr struct RegisterConfig RegisterSettings[] EEMEM = {

  /* enable output port pins */
  {PORTD,_BV(PIN2) | _BV(PIN4) | _BV(PIN6)},
  {DDRD, _BV(PIN2) | _BV(PIN4) | _BV(PIN5) | _BV(PIN6)},

  {PORTB, _BV(PIN2) | _BV(PIN5) | _BV(PIN6) | _BV(PIN7)},
  {DDRB,  _BV(PIN2) | _BV(PIN6) | _BV(PIN7)},

  /* configure timer 0 to generate 20khz square */
  {OCR0A, timer0_max},
  {OCR0B, timer0_max / 2},

  {TCCR0A, _BV(COM0B1) | _BV(WGM00) | _BV(WGM01)},
  {TCCR0B, _BV(CS01) | _BV(WGM02)},

  /* activate watchdog timer (used for led timing) */
  // WDTCSR = _BV(WDIE);

  /* use timer1 for 1ms clock */
  {TCNT1H, 0},
  {TCNT1L, 0},
  {OCR1AH, timer1_max / 256},
  {OCR1AL, timer1_max % 256},
  {TIMSK, _BV(OCIE1A)},
  {TCCR1B, _BV(WGM12) | _BV(CS12)},
};

constexpr uint8_t CalibrationData [] EEMEM = {
    Interface::BLOCK_TYPE_CALIBRATION,
    0x02,
    (3300) % 256,
    (3300) / 256,
    0xFF,
};

ISR(TIMER1_COMPA_vect);

class Application {
private:

  union {
    volatile uint32_t       _u32Clock1ms;
    volatile uint16_t       _u16Clock1ms;
    volatile uint8_t        _u8Clock1ms;

    uint32_t                _u32Clock1msEvent; /* interrupt routine actually changes this var
                                                  unfortunately gcc is still very stupid, see irq handler below */
    uint16_t                _u16Clock1msEvent; /* interrupt routine actually changes this var
                                                  unfortunately gcc is still very stupid, see irq handler below */
  };

  uint16_t                _u16Timeout1ms;
  uint8_t                 _u8LedSequence;

  uint8_t                 _channels;

  struct {
    Interface::BlockConfigA a;
    Interface::BlockConfigB b[Interface::CHANNEL_MAX];
  } __attribute__((packed)) _config;

  SDCard<SPISlaveA>       _sdcard;
  MCP3204<SPISlaveB>      _adc;

  Ringbuffer<uint8_t, 64> _buf;

  enum ApplicationState {
    APP_STATE_NOSDCARD = 0,
    APP_STATE_ERR_CFG,
    APP_STATE_ERR_WR,
    APP_STATE_ERR_RD,
    APP_STATE_RUN,
    APP_STATE_FULL,
  };

  static void inititalize() {
    /* set clockdivider to 1 
     * this must be done within four cycles */
    CLKPR = _BV(CLKPCE);
    CLKPR =  0;

    for(uint8_t index = 0; index < ElementCount(RegisterSettings); index++)
      RegisterSettings[index].writeRegister();

    USISPIMaster<0>::initialize(0);
  }

  __attribute__((always_inline))
  inline int8_t readConfiguration() {
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&(this->_config));
    uint8_t index;

    auto blk = reinterpret_cast<Interface::BlockHead*>(ptr);

    if (0 != _sdcard.startMultipleRead(0))
      return -1;

    for(index = 0; index < SizeOf(*blk);index++)
      ptr[index] = _sdcard.readByte();

    if (_sdcard.getError())
      return -1;

    if (blk->getType() != Interface::BLOCK_TYPE_CONFIG)
      return -2;

    const auto contensize = blk->getLen();

    if(contensize < SizeOf(this->_config.a))
      return -2;

    if(contensize > SizeOf(this->_config))
      return -2;

    this->_channels = (contensize - SizeOf(this->_config.a)) / SizeOf(this->_config.b[0]);

    for(index = 0; index < contensize;index++)
      ptr[index] = _sdcard.readByte();

    if (_sdcard.getError())
      return -1;

    _sdcard.stopMultipleRead();

    if (this->_config.a.getMeasureInterval() > 60000)
      return -2;

    return 0;
  }

  __attribute__((always_inline))
  inline int8_t measure() {
    const uint8_t contentsize  = sizeof(Interface::BlockDataA) +
                                 sizeof(Interface::BlockDataB) * _channels;

    const uint8_t blksize = contentsize +
                             sizeof(Interface::BlockHead) +
                             sizeof(Interface::BlockEnd);

    const uint8_t spacerb = _buf.getSize() - _buf.getCount();

    if (spacerb < blksize)
      return -1;

    _buf.pushForced(Interface::BLOCK_TYPE_DATA);
    _buf.pushForced(contentsize);

    uint32_t clock = this->u32getClock1ms();

    _buf.pushForced(static_cast<uint8_t>((clock >> 0) & 0xFF));
    _buf.pushForced(static_cast<uint8_t>((clock >> 8) & 0xFF));
    _buf.pushForced(static_cast<uint8_t>((clock >> 16) & 0xFF));
    _buf.pushForced(static_cast<uint8_t>((clock >> 24) & 0xFF));

    for(decltype(_channels) index = 0; index < _channels; index++) {
        const auto channel = _config.b[index].getChannel();
        uint16_t value = 0xFFFF;

        if(channel < Interface::CHANNEL_DI) {
            value = _adc.sample(channel);
        } else if (channel == Interface::CHANNEL_DI) {
            const uint8_t input = PINB;
            value = (input & 0x3) | ((input >> 1) & 0xC);
        }

        value |= (this->_u8Clock1ms) << 12;

        _buf.pushForced(static_cast<uint8_t>((value >> 0) & 0xFF));
        _buf.pushForced(static_cast<uint8_t>((value >> 8) & 0xFF));
    }

    _buf.pushForced(0xFF);
    return 0;
  }

  int8_t store() {
    auto len = _sdcard.writeByte(_buf.front());

    if (len < 0)
      return -1;

    if (len == 1)
      _buf.popForced();

    return 0;
  }

  bool checkTimeout() {
    const uint16_t u16Shift1ms = 62000;
    const uint16_t u16Delta1ms = u16Shift1ms + this->u16getClock1ms() - this->_u16Timeout1ms;

    return (u16Delta1ms > u16Shift1ms);
  }

  bool checkTimeoutNoLock() {
    const uint16_t u16Shift1ms = 62000;
    const uint16_t u16Delta1ms = u16Shift1ms + this->_u16Clock1ms - this->_u16Timeout1ms;

    return (u16Delta1ms > u16Shift1ms);
  }

  void setTimeoutDelay(const uint16_t u16Delay1ms) {
    this->_u16Timeout1ms = this->u16getClock1ms() + u16Delay1ms;
  }

  void setTimeoutInc(const uint16_t u16Interval1ms) {
    this->_u16Timeout1ms += u16Interval1ms;
  }


  void setLedState(const enum ApplicationState state) {
    static constexpr uint8_t sequences[] PROGMEM = {
        0x0F /* APP_STATE_NOSDCARD */,
        0x30 /* APP_STATE_ERR_CFG  */,
        0x05 /* APP_STATE_ERR_WR   */,
        0x03 /* APP_STATE_ERR_RD   */,
        0x00 /* APP_STATE_RUN      */,
        0x11 /* APP_STATE_FULL     */,
    };

    uint8_t sequence;

    asm volatile (
        "lpm %0, %a1"
        : "=&r" (sequence)
        : "b"   (&sequences[state]));

    _u8LedSequence = sequence;
  }

  __attribute__((always_inline)) static void enableLed0()  {PORTD &= ~(_BV(PIN4));}
  __attribute__((always_inline)) static void disableLed0() {PORTD |=  (_BV(PIN4));}

  __attribute__((always_inline)) static void enableLed1()  {PORTD &= ~(_BV(PIN6));}
  __attribute__((always_inline)) static void disableLed1() {PORTD |=  (_BV(PIN6));}

public:

  __attribute__((noreturn))
  void main() {
    enum ApplicationState state = APP_STATE_NOSDCARD;

    inititalize();

    sei();
    while(1) {
        this->setLedState(state);

        switch(state)
        {
        case APP_STATE_NOSDCARD:
          {
            int8_t result;

            if (0 != _sdcard.initCard(*this)) {
                state = APP_STATE_NOSDCARD;
            } else if ( 0 != (result = this->readConfiguration())) {
                if (result == -1)
                  state = APP_STATE_ERR_RD;
                else
                  state = APP_STATE_ERR_CFG;
            } else if ( 0 != _sdcard.startMultipleWrite(_sdcard.getBlockSize())) {
                state = APP_STATE_ERR_WR;
            } else {
                _buf.reset();

                for(uint8_t index = 0; index < ElementCount(CalibrationData); index++)
                  _buf.pushForced(readEEPROM(CalibrationData + index));

                state = APP_STATE_RUN;
                this->setTimeoutDelay(0);
                continue;
            }
            break;
          }
        case APP_STATE_RUN:
          {
            if (this->checkTimeout()) {
                this->setTimeoutInc(_config.a.getMeasureInterval());
                this->measure();
            }

            if (this->_buf.getCount() > 0)
            {
              if(this->store() < 0)
              {
                this->setTimeoutDelay(4096);
                state = APP_STATE_FULL;
              }

              continue; /* jump to beginning of while */
            }
            break;
          }
        case APP_STATE_ERR_CFG:
        case APP_STATE_ERR_RD:
        case APP_STATE_ERR_WR:
        case APP_STATE_FULL:
          if (this->checkTimeout()) {
              this->setTimeoutInc(4096);

              if ( 0 != _sdcard.initCard(*this)) {
                state = APP_STATE_NOSDCARD;
                continue;
              }
          }
          break;
        }

        cli();
        if (this->checkTimeoutNoLock() == false) {
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
    }
  }


  inline void eventClockTimer() {
    _u32Clock1msEvent = _u32Clock1msEvent + 1;
    updateLed();
  }

  inline void updateLed() const {
    uint16_t u16Clock1ms = _u16Clock1ms;

    if ((u16Clock1ms % 256) == 0) {
      const uint8_t shift = ((u16Clock1ms / 256) % 4);
      const uint8_t mask  = 0x11;
      const uint8_t state = _u8LedSequence & (mask << shift);

      if(state & 0x0F)
        enableLed0();
      else
        disableLed0();

      if(state & 0xF0)
        enableLed1();
      else
        disableLed1();
    }
  }

  uint32_t u32getClock1ms() const {
    CriticalSectionGuard guard;
    return this->_u32Clock1ms;
  }

  uint16_t u16getClock1ms() const {
    CriticalSectionGuard guard;
    return this->_u16Clock1ms;
  }

  uint8_t u8getClock1ms() const {
    CriticalSectionGuard guard;
    return this->_u8Clock1ms;
  }

  friend void TIMER1_COMPA_vect ();
};


static Application app;

ISR(TIMER1_COMPA_vect) {
  /* gcc is very stupid in optimizing this
   * actually the irq handler takes 9 bytes of the stack
   * and uses a lot of 4byte instructions.
   * Therefore, use assembler here */
  // app.eventClockTimer();

  uint16_t u16;
  asm volatile (
      "ld  %A0, %a1"          "\n\t"
      "ldd %B0, %a1+1"        "\n\t"
      "adiw %A0, 0x1"         "\n\t"
      "st  %a1, %A0"          "\n\t"
      "std  %a1+1, %B0"        "\n\t"
      "ldd  %A0, %a1+2"        "\n\t"
      "ldd  %B0, %a1+3"        "\n\t"
      "adc %A0,__zero_reg__"  "\n\t"
      "adc %B0,__zero_reg__"  "\n\t"
      "std  %a1+2, %A0"        "\n\t"
      "std  %a1+3, %B0"          "\n\t"
      "ld  %A0, %a1"             "\n\t"
      "cpse %A0, __zero_reg__"   "\n\t"
      "rjmp 1f"                  "\n\t"
      "ldd  %A0, %a1+1"          "\n\t"
      "andi %A0, 0x03"           "\n\t"
      "ldi  %B0, 0x11"           "\n\t"
      "rjmp 3f"                  "\n\t"
      "2:"                       "\n\t"
      "add %B0, %B0"             "\n\t"
      "3:"                       "\n\t"
      "dec %A0"                  "\n\t"
      "brpl 2b"                  "\n\t"
      "ldd %A0, %a1+%2"          "\n\t"
      "and %A0, %B0"             "\n\t"
      "mov %B0, %A0"             "\n\t"
      "andi %B0, 0x0F"           "\n\t"
      "breq 4f"                  "\n\t"
      "cbi  0x12, 4"             "\n\t"
      "rjmp 5f"                  "\n\t"
      "4:"                       "\n\t"
      "sbi  0x12, 4"             "\n\t"
      "5:"                       "\n\t"
      "andi %A0, 0xF0"           "\n\t"
      "breq 6f"                  "\n\t"
      "cbi  0x12, 6"             "\n\t"
      "rjmp 7f"                  "\n\t"
      "6:"                       "\n\t"
      "sbi  0x12, 6"             "\n\t"
      "7:"                       "\n\t"
      "1:"
      : "=&w" (u16)
      : "b" (&app._u32Clock1ms) , "I" (&app._u8LedSequence - reinterpret_cast<uint8_t*>(&app))
      : "memory"
      );
}

int main(void) {
  app.main();
}




