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
  __attribute__((always_inline)) static void selectChip()         {DDRD  |= _BV(PIN2);}
  __attribute__((always_inline)) static void deselectChip()       {DDRD  &= ~_BV(PIN2);}

  USISPIMaster<400000> getTransport(const unsigned long) { return USISPIMaster<400000>(); }
};

#if 0
  static constexpr uint8_t _MaxConfigSize = sizeof(Interface::BlockHead) +
                                         sizeof(Interface::BlockConfigA) +
                                         sizeof(Interface::BlockConfigB) * Interface::CHANNEL_MAX +
                                         sizeof(Interface::BlockEnd);
  inline void checkTimeout();
  inline void sleep();

  inline void ClockTimerTick();
  inline void LedTimerTick();

  inline void startTimer(uint16_t u16Interval1ms);
  inline void stopTimer();


  inline void     main() __attribute__((noreturn)) ;

  template<typename SIZET>
  inline uint8_t* readConfiguration(const SIZET len);
  template<typename SIZET>
  inline bool     canWrite(const SIZET len);
  template<typename TYPE>
  inline void     writeData(const TYPE& data) __attribute__((allways_inline));
#endif

class Application {
private:

  union {
    volatile uint32_t       _u32Clock1ms;
    volatile uint16_t       _u16Clock1ms;
    volatile uint8_t        _u8Clock1ms;

    uint32_t                _u32Clock1msEvent; /* interrupt routine actually changes this var
                                                  unfortunately gcc is still very stupid, see irq handler below */
  };

  union {
    uint32_t                _u32Timeout1ms;
    uint16_t                _u16Timeout1ms;
  };

  uint8_t                 _u8LedTimeout1ms;
  uint8_t                 _u8LedSequence;

  uint8_t                 _channels;

  struct {
    Interface::BlockConfigA a;
    Interface::BlockConfigB b[Interface::CHANNEL_MAX];
  } __attribute__((packed)) _config;

  SDCard<SPISlaveA>       _sdcard;
  MCP3204<SPISlaveB>      _adc;

  Ringbuffer<uint8_t, 64> _buf;

  enum LedState {
    LED_SEQUENCE_NOSDCARD     = 0x0F,
    LED_SEQUENCE_SDCARD_RDERR = 0x03,
    LED_SEQUENCE_SDCARD_WRERR = 0x05,
    LED_SEQUENCE_SDCARD_FULL  = 0x11,
    LED_SEQUENCE_CONFIG_ERR   = 0x30,
    LED_SEQUENCE_NOERR        = 0x00,
  };

  void setLedState(const enum LedState state) {
    _u8LedSequence = state;
  }

  static void inititalize() {
    /* set clockdivider to 1 */
    CLKPR = _BV(CLKPCE);
    CLKPR = 0;

    /* enable output port pins */
    PORTD = _BV(PIN4) | _BV(PIN6);
    DDRD  = _BV(PIN4) | _BV(PIN5) | _BV(PIN6);

    PORTB = _BV(PIN2) | _BV(PIN5) | _BV(PIN6) | _BV(PIN7);
    DDRB  = _BV(PIN2) | _BV(PIN6) | _BV(PIN7);

    /* configure timer 0 to generate 20khz square */
    const uint8_t timer0_max = ((F_CPU / 8) / 20000);

    OCR0A  = timer0_max;
    OCR0B  = timer0_max/2;

    TCCR0A = _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B = _BV(CS01) | _BV(WGM02);

    /* activate watchdog timer (used for led timing) */
    WDTCSR = _BV(WDIE);

    /* use timer1 for 1ms clock */
    TCNT1  = 0;
    OCR1A  = 0.001 * (F_CPU / 256.);
    TIMSK  = _BV(OCIE1A);
    TCCR1B = _BV(WGM12) | _BV(CS12);

    USISPIMaster<0>::initialize(0);
  }

  __attribute__((always_inline))
  inline int8_t readConfiguration() {
    uint8_t *ptr = reinterpret_cast<uint8_t*>(&(this->_config));

    auto blk = reinterpret_cast<Interface::BlockHead*>(ptr);

    if (0 != _sdcard.startMultipleRead(0))
      return -1;

    if(SizeOf(*blk) != _sdcard.read(ptr,SizeOf(*blk)))
      return -1;

    if (blk->getType() != Interface::BLOCK_TYPE_CONFIG)
      return -2;

    const auto contensize = blk->getLen();

    if(contensize < SizeOf(this->_config.a))
      return -2;

    if(contensize > SizeOf(this->_config))
      return -2;

    this->_channels = (contensize - SizeOf(this->_config.a)) / SizeOf(this->_config.b[0]);

    if(contensize != _sdcard.read(ptr,contensize))
      return -1;

    _sdcard.stopMultipleRead();

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

  __attribute__((always_inline)) static void enableLed0()  {PORTD &= ~(_BV(PIN4));}
  __attribute__((always_inline)) static void disableLed0() {PORTD |=  (_BV(PIN4));}

  __attribute__((always_inline)) static void enableLed1()  {PORTD &= ~(_BV(PIN6));}
  __attribute__((always_inline)) static void disableLed1() {PORTD |=  (_BV(PIN6));}

public:
  void eventLedTimer() {
    const uint16_t u16Clock1ms = this->_u16Clock1ms;

    const uint8_t shift = ((u16Clock1ms / 256) % 4);
    const uint8_t mask  = 0x11;
    const uint8_t state = _u8LedSequence & (mask << shift);

    if(state & 0x0F)
      this->enableLed0();
    else
      this->disableLed0();

    if(state & 0xF0)
      this->enableLed1();
    else
      this->disableLed1();

  }

  __attribute__((noreturn))
  void main() {
    enum LedState state;

    inititalize();

    state = LED_SEQUENCE_NOSDCARD;

    sei();
    while(1) {
        this->setLedState(state);

        if (state == LED_SEQUENCE_NOERR) {
            while(state == LED_SEQUENCE_NOERR) {
                cli();
                if (static_cast<int32_t>(this->_u32Clock1ms - this->_u32Timeout1ms) >= 0) {
                    sei();
                    this->_u32Timeout1ms += _config.a.getMeasureInterval();
                    this->measure();
                } else if (this->_buf.getCount() > 0) {
                    sei();

                    if(this->store() < 0)
                    {
                      this->_u32Timeout1ms = this->u32getClock1ms() + 1024;
                      state = LED_SEQUENCE_SDCARD_FULL;
                    }
                } else {
                    sleep_enable();
                    sei();
                    sleep_cpu();
                    sleep_disable();
                }
            }
        } else if (state == LED_SEQUENCE_SDCARD_FULL) {
            cli();
            if (static_cast<int32_t>(this->_u16Clock1ms - this->_u16Timeout1ms) >= 0) {
                sei();
                this->_u32Timeout1ms += 1024;

                if ( 0 != _sdcard.initCard(*this))
                  state = LED_SEQUENCE_NOSDCARD;
            } else {
                sleep_enable();
                sei();
                sleep_cpu();
                sleep_disable();
            }
        } else {
          int8_t result;

          if (0 != _sdcard.initCard(*this)) {
              state = LED_SEQUENCE_NOSDCARD;
          } else if ( 0 != (result = this->readConfiguration())) {
              if (result == -1)
                state = LED_SEQUENCE_SDCARD_RDERR;
              else
                state = LED_SEQUENCE_CONFIG_ERR;
          } else if ( 0 != _sdcard.startMultipleWrite(_sdcard.getBlockSize())) {
              state = LED_SEQUENCE_SDCARD_WRERR;
          } else {
              _buf.reset();
              state = LED_SEQUENCE_NOERR;
              this->_u32Timeout1ms = this->u32getClock1ms();
          }
        }
    }
  }

  void eventClockTimer() {
    _u32Clock1msEvent += 1;
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
};


static Application app;

ISR(TIMER1_COMPA_vect) {
  /* gcc is very stupid in optimizing this
   * actually the irq handler takes 9 bytes of the stack
   */
  app.eventClockTimer();
}

ISR(WDT_OVERFLOW_vect) {
  app.eventLedTimer();
}

int main(void) {
  app.main();
}




