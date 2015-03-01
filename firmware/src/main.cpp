/*
 * main.cpp
 *
 *  Created on: 10.02.2015
 *      Author: andi
 */
#include "target/attiny461.hpp"
#include "util/guard.hpp"
#include "ECPP/Byteorder.hpp"
#include "ECPP/Datatypes.hpp"
#include "ECPP/Arch/AVR8.hpp"

using namespace ECPP;
using namespace ECPP::Datatypes;

using namespace Platform::Architecture::AVR8;
using namespace Platform::Util::Datatypes;
using namespace Platform::Guards;

typedef USISPIMaster<1000000> SPIDriverType;


const FlashBuffer<25,RegisterConfig> g_RegisterInit PROGMEM =
{{
  {DDRA, 0},
  {DDRB, 0},

  {GPIOR0, 0},
  {GPIOR1, 0},
  /* enable pll */
  {PLLCSR, _BV(PLLE)},

  /* configure adc */
  {DIDR0,  _BV(AREFD) | _BV(ADC3D) | _BV(ADC4D) | _BV(ADC5D) | _BV(ADC6D)},
  {DIDR1, _BV(ADC9D)},
  {ADCSRB, _BV(ADTS2) | _BV(ADTS1)}, /* we want to be triggered by Timer 1 Overflow */
  {ADCSRA, _BV(ADEN)  | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADIE) | _BV(ADATE)},

  /* configure tick timer */
  {TCCR0A, _BV(WGM00)},
  {OCR0A,  125 - 1},
  {TCCR0B, _BV(CS01) | _BV(CS00) | _BV(PSR0)},
  {TCNT0L,  0},

  /* preconfigure pwm timer */
  {OCR1C ,  0x7F},
  {TCCR1A, _BV(PWM1A)| _BV(PWM1B)},
  {TCCR1D, _BV(WGM10)},
  {TCCR1B, _BV(CS10)},

  /* configure ports */
  {PORTA,  0},
  {PORTB, _BV(PB5)},
  {DDRA,  _BV(PA1) | _BV(PA2)},
  {DDRB,  _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB5)},
  {USIPP, _BV(USIPOS)},
  SPIDriverType::RegisterInit<SPIDriverType::SPI_MODE3>(),

  /* configure timer interrupts */
  {TIMSK, _BV(OCIE0A)},
  /* configure sleep mode idle */
  {MCUCR, 0},
}};

class CanDevice : public SPIDriverType
{
public:
  static void assertChipSelect() __attribute__((always_inline))
  {
    PORTB &= ~ _BV(PB5);
  }

  static void deassertChipSelect() __attribute__((always_inline))
  {
    PORTB |= _BV(PB5);
  }
};

template<class DEVICE>
class MCP2515Driver : public DEVICE
{
private:
  struct __attribute__((packed)) TransmitBuffer
  {
    uint8_t  cmd;

    uint8_t  sidh;

    uint8_t  sidl;

    uint8_t  eid8;

    uint8_t  eid0;

    uint8_t  dlc;

    uint8_t  data[8];
  };

  struct __attribute__((packed)) ReadRegisters
  {
    uint8_t  cmd;

    uint8_t  reg;

    uint8_t  val[8];
  };

  struct __attribute__((packed)) WriteRegisters
  {
    uint8_t  cmd;

    uint8_t  reg;

    uint8_t  val[8];
  };

  struct __attribute__((packed)) ModifyRegister
  {
    uint8_t  cmd;

    uint8_t  reg;

    uint8_t  mask;

    uint8_t  val;
  };

  union {
    TransmitBuffer   tx;
    ReadRegisters    rr;
    WriteRegisters   wr;
    ModifyRegister   mr;

    uint8_t        bytes[];
  } _scratch;

  typedef ChipSelectGuard<MCP2515Driver> ChipSelectGuardType;

  enum CommandCodes
  {
    CMD_WRITE_REGISTER      = 0x02,
    CMD_READ_REGISTER       = 0x03,
    CMD_BITMODIFY_REGISTER  = 0x05,
    CMD_LOAD_TXBUFFER       = 0x40,
  };

public:
  enum RegisterAddresses
  {
    REG_BFPCTRL   = 0x0C,
    REG_TXRTSCTRL = 0x0D,

    REG_CANCTRL   = 0x0F,

    REG_CNF3      = 0x28,
    REG_CNF2      = 0x29,
    REG_CNF1      = 0x2A,

    REG_TXB0CTRL  = 0x30,

    REG_TXB0DLC   = 0x35,
    REG_TXB0D0    = 0x36,
  };

  enum
  {
    MODE_NORMAL   = 0,
    MODE_SLEEP    = 1,
    MODE_LOOPBACK = 2,
    MODE_LISTEN   = 3,
    MODE_CONFIG   = 4,
  };

  enum
  {
    MSK_TXREQ   = 0x08,
    MSK_REQOP   = 0xE0,
    MSK_BTLMODE = 0x80,
    MSK_PHSEG1  = 0x34,
    MSK_PRSEG   = 0x07,
    MSK_PHSEG2  = 0x07,
    MSK_B0BFE   = 0x04,
    MSK_B1BFE   = 0x08,
    MSK_B0BFS   = 0x10,
    MSK_B1BFS   = 0x20,
    MSK_OSM     = 0x08,
  };

  enum
  {
    SRT_REQOP  = 5,
    SRT_SJW    = 6,
    SRT_PRSEG  = 0,
    SRT_PHSEG1 = 3,
    SRT_PHSEG2 = 0,
    SRT_B0BFS  = 4,
    SRT_B0BFE  = 2,
    SRT_B0RTS  = 3,
  };




  uint8_t getRegAddr(uint8_t buf, uint8_t base)
  {
    return base + (buf * 16);
  }

  void transferScratch(uint8_t len)
  {
    ChipSelectGuardType guard(*this);
    this->transferData(len, _scratch.bytes);
  }

  void readRegisters(uint8_t count)
  {
    ReadRegisters & b = _scratch.rr;

    b.cmd = CMD_READ_REGISTER;

    transferScratch(offsetof(ReadRegisters, val[count]));
  }

  uint8_t readRegister(uint8_t reg)
  {
    ReadRegisters & b = _scratch.rr;

    b.reg = reg;

    readRegisters(1);
    return b.val[0];
  }

  void writeRegisters(uint8_t count)
  {
    WriteRegisters & b = _scratch.wr;

    b.cmd = CMD_WRITE_REGISTER;

    transferScratch(offsetof(WriteRegisters, val[count]));
  }

  void writeRegister(uint8_t reg, uint8_t val)
  {
    WriteRegisters & b = _scratch.wr;

    b.reg = reg;
    b.val = val;

    writeRegisters(1);
  }

  void modifyRegister(uint8_t reg, uint8_t mask, uint8_t val)
  {
    ModifyRegister & b = _scratch.mr;

    b.cmd  = CMD_BITMODIFY_REGISTER;
    b.reg  = reg;
    b.mask = mask;
    b.val  = val;

    transferScratch(4);
  }


  bool isTxPending(uint8_t buf)
  {
    const uint8_t reg = getRegAddr(buf, REG_TXB0CTRL);
    const uint8_t val = readRegister(reg);
    return (val & MSK_TXREQ);
  }

  uint8_t transmitStandard(uint8_t buf, uint16_t sid, uint8_t len)
  {
    /* poll until transmitt buffer is ready */
    const bool pending = isTxPending(buf);

    if (pending)
    {
      len = 0;
    }
    else
    {
      if(len > 8)
        len = 8;

      TransmitBuffer & b = _scratch.tx;

      b.cmd    = CMD_LOAD_TXBUFFER | (buf * 2);
      b.sidh   = sid >> 3;
      b.sidl   = sid << 5;
      b.eid8   = 0;
      b.eid0   = 0;
      b.dlc = len;

      transferScratch(offsetof(TransmitBuffer, data[len]));

      /* initiate send */
      modifyRegister(getRegAddr(buf, REG_TXB0CTRL),
          MSK_TXREQ, MSK_TXREQ);
    }

    return len;
  }

  void setRxBfPins(uint8_t mask, uint8_t oe)
  {
    mask = (mask << SRT_B0BFS) | (oe << SRT_B0BFE);
    modifyRegister(REG_BFPCTRL, MSK_B0BFS | MSK_B1BFS | MSK_B0BFE | MSK_B1BFE,
        mask);
  }

  uint8_t getTxRtsPins()
  {
    const uint8_t val = readRegister(REG_TXRTSCTRL);
    return (val >> SRT_B0RTS);
  }


  void requestModeChange(uint8_t mode)
  {
    const uint8_t val = (mode << SRT_REQOP);
    modifyRegister(REG_CANCTRL, MSK_REQOP, val);
  }

  uint8_t getMode()
  {
    const uint8_t mode = readRegister( REG_CANCTRL) >> SRT_REQOP;
    return mode;
  }

  /**
   * fosc is in hz, tq is in ns
   * precision of fosc is 0.1 Mhz
   */
  static constexpr uint8_t
  calcBRP(long fosc, long tq)
  {
    return 0x3F & (tq * (fosc / 100000) / 10000 / 2 - 1);
  }

  /* outputs bit time in nanoseconds
   * baudrate is in bits/s*/
  static constexpr long
  calcBitTime(long baudrate)
  {
    return 1000000000 / baudrate;
  }

  /* outputs tq  in nanoseconds
   * baudrate is in bits/s*/
  static constexpr long
  calcTQ(long BitTime, uint8_t PropSeq, uint8_t PhaseSeg1, uint8_t PhaseSeg2)
  {
    return BitTime / (1 + PropSeq + PhaseSeg1 + PhaseSeg2);
  }

  static constexpr uint8_t
  getCNF1(long FOsc, long BaudRate, uint8_t PropSeq, uint8_t PhaseSeg1, uint8_t PhaseSeg2)
  {
    return calcBRP(FOsc, calcTQ(calcBitTime(BaudRate), PropSeq, PhaseSeg1, PhaseSeg2)) |
        (((PhaseSeg2 > 4 ? 4 : PhaseSeg2) - 1) << SRT_SJW);
  }

  static constexpr uint8_t
  getCNF2(long FOsc, long BaudRate, uint8_t PropSeq, uint8_t PhaseSeg1, uint8_t PhaseSeg2)
  {
    return (((PropSeq - 1) << SRT_PRSEG) & MSK_PRSEG) |
        (((PhaseSeg1 - 1) << SRT_PHSEG1) & MSK_PHSEG1) |
        MSK_BTLMODE;
  }

  static constexpr uint8_t
  getCNF3(long FOsc, long BaudRate, uint8_t PropSeq, uint8_t PhaseSeg1, uint8_t PhaseSeg2)
  {
    return (((PhaseSeg2 - 1) << SRT_PHSEG2) & MSK_PHSEG2);
  }

  void configure(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3)
  {
    requestModeChange(MODE_CONFIG);

    while(MODE_CONFIG != getMode());

    WriteRegisters & b = _scratch.wr;

    b.reg    = REG_CNF3;
    b.val[0] = cnf3;
    b.val[1] = cnf2;
    b.val[2] = cnf1;

    writeRegisters(3);

    requestModeChange(MODE_NORMAL);
  }

  void setOneshot(uint8_t oneshot)
  {
    modifyRegister(REG_CANCTRL, MSK_OSM,
        oneshot ? MSK_OSM : 0);
  }

  uint8_t
  (& getTxBuffer()) [8]
  {
    return _scratch.tx.data;
  }
};

enum
{
  VOUTA  = 0,
  IOUTA  = 1,
  VOUTB  = 2,
  IOUTB  = 3,
  VSOLAR = 4,
};

template<typename T>
class Clock
{
private:
  T _cnt;

public:
  template<typename C>
  void update(const C val)
  {
    const C maskc = ~static_cast<C>(0);
    const C part  = static_cast<C>(_cnt & maskc);

    if (val != part)
    {
      const T maskt = ~static_cast<T>(maskc);

      if (val > part)
      {
        _cnt = (_cnt & maskt) + val;
      }
      else
      {
        const T add = static_cast<T>(maskc) + 1;
        _cnt = (_cnt & maskt) + add + val;
      }
    }
  }

  constexpr T value()
  {
    return _cnt;
  }

  template<typename C>
  class Timer
  {
  private:
    C _tmout;
  public:

    C hasTimedOut(const Clock & clock)
    {
      const C maskc = ~static_cast<C>(0);
      const C part  =  static_cast<C>(clock.value() & maskc);
      C delta = part - _tmout;

      if (delta > TypeProperties<C>::MaxSigned)
        delta = 0;
      else if (delta == 0)
        delta = 1;

      return delta;
    }

    void updateTimeout(C iv)
    {
      _tmout += iv;
    }

    constexpr C value()
    {
      return _tmout;
    }

  };
};

class App
{
private:
  typedef Clock<uint32_t> ClockType;

  MCP2515Driver<CanDevice> _Can;
  ClockType                _Clock;

  uint8_t                  _CanState;
  uint8_t                  _CanSid;

  uint8_t                  _AdcState;
  uint16_t                 _Values[5];

  ClockType::Timer<uint8_t>  _AdcTimer;
  ClockType::Timer<uint16_t> _CanTimer;
  ClockType::Timer<uint16_t> _LedTimer;

  uint16_t                 _LastCycle;
  uint16_t                 _CycleTime;

  enum AdcMux{
    ADMUX_VOUTA  = _BV(REFS0) | _BV(ADLAR) | 0x3,
    ADMUX_IOUTA  = _BV(REFS0) | _BV(ADLAR) | 0x12,
    ADMUX_VOUTB  = _BV(REFS0) | _BV(ADLAR) | 0x5,
    ADMUX_IOUTB  = _BV(REFS0) | _BV(ADLAR) | 0x17,
    ADMUX_VSOLAR = _BV(REFS0) | _BV(ADLAR) | 0x9,
  };

  static const FlashBuffer<5,uint8_t> ADMUXMapping;

  enum Event {
    EV_ADC  = 0x1,
  };

  enum Led {
    LED_RED   = 0x01,
    LED_GREEN = 0x02,
    LED_BOTH  = 0x03,
  };

public:
  void setDutyCycle(uint8_t duty)
  {
    const uint8_t max_duty = OCR1C - 1;

    if (duty > max_duty)
      duty = max_duty;

    if (duty == 0)
    {
      TCCR1A &= ~_BV(COM1A0);
      TCCR1A &= ~_BV(COM1B0);
    }
    else
    {

      OCR1A = duty;
      OCR1B = max_duty + 1 - duty;
      TCCR1A |= _BV(COM1A0);
      TCCR1A |= _BV(COM1B0);
    }
  }

  void setLedStatus(uint8_t mask)
  {
    _Can.setRxBfPins(~mask, 0x3);
  }

  void init()
  {
    setLedStatus(0x3);

    /* wait for pll */
    while(0 == (PLLCSR & _BV(PLOCK)));
    PLLCSR |= _BV(PCKE);

    const uint8_t cnf1 = _Can.getCNF1(8000000, 125000, 2, 7, 6);
    const uint8_t cnf2 = _Can.getCNF2(8000000, 125000, 2, 7, 6);
    const uint8_t cnf3 = _Can.getCNF3(8000000, 125000, 2, 7, 6);

    _Can.configure(cnf1, cnf2, cnf3);
    _Can.setOneshot(true);

#if 0
    /* set configuration mode */
    _Can.writeRegister(_Can.REG_CANCTRL, 0b100 << 5);;

    _Can.writeRegister(_Can.REG_CNF1, 0x1);
    _Can.writeRegister(_Can.REG_CNF2, 0x1 | 0x6 << 3 | 0x80);
    _Can.writeRegister(_Can.REG_CNF3, 0x5);

    /* set normal mode */
    _Can.writeRegister(_Can.REG_CANCTRL, 0b000 << 5 | 0x08);
#endif

    _CanState = 0;
    _AdcState = 0xFF;
    _CanSid   = _Can.getTxRtsPins();
  }

  void perfStartCycle()
  {
    _LastCycle = _Clock.value();
  }

  void perfFinishCycle()
  {
    uint16_t time = _Clock.value();
    _CycleTime = time - _LastCycle;
  }

  void startADCConversion()
  {
    if(_AdcState < 10)
    {
      if (0 == (_AdcState % 2))
      {
        /* first conversation after changing admux is garbage
         * we start conversation immediately */
        ADMUX = ADMUXMapping[_AdcState/2];
        ADCSRA |= _BV(ADSC);
      }
      else
      {
        /* now perform the real conversation, this is synced to
         * Timer 1 PWM */
        TIFR   |= _BV(TOV1);
      }
    }
  }


  void handleRegul()
  {
    uint8_t tmout = _AdcTimer.hasTimedOut(_Clock);

    if (_AdcState >= 10 && tmout)
    {
      perfStartCycle();

      _AdcState = 0;
      startADCConversion();

      _AdcTimer.updateTimeout(10);
    }
  }


  void handleADCEvent()
  {

    if(GPIOR1 & EV_ADC)
    {
      uint8_t state = _AdcState;

      GPIOR1 &= ~EV_ADC;

      if(state < 10)
      {
        uint16_t value = ADC;

        _AdcState++;
        startADCConversion();

        if((state & 0x01) == 0)
        {
          /* garbage, first conversation after changing admux */
        }
        else if(state & 0x2)
        {
          /* this was a current
           * 0,1 Ohm 20x, 1/11, vref 2.5 v
           * measure / 1024 / 20 * 2.5 * 11 / 0.1 * 1000
           * measure * 2.5 * 11 * 1000 / 1024 / 20 / 0.1
           * measure * 25 * 11 * 1000 / 4096 / 5
           * measure *  5 * 11 * 1000 / 4096
           * measure *  80 * 11 * 1000 / 65536
           *
           * compensate for adlar and unit is 10mA
           * */
          const uint32_t fact_mul = 80UL * 11 * 1000 / 64 / 10;
          const uint32_t fact_div = 65536;
          _Values[state/2] = (value * fact_mul) / fact_div;
        }
        else
        {
          /* this is a voltage
           * 1/11, vref 2.5V
           * measure / 1024 * 2.5 * 11 * 1000
           * measure * 5 * 11 * 1000 * 32 / 65536
           *
           * compensate for adlar and unit is 100mV
           */
          const uint32_t fact_mul = 160UL * 11 * 1000 / 64 / 100;
          const uint32_t fact_div = 65536;
          _Values[state/2] = (value * fact_mul) / fact_div;
        }

        if(state == 9)
          perfFinishCycle();
      }
    }
  }

  void handleCan()
  {
    uint8_t state = _CanState;

    if (_CanTimer.hasTimedOut(_Clock))
    {
      _CanTimer.updateTimeout(1000);

      if(state >= 5)
        state = 0;
    }

    if(state < 5 && !_Can.isTxPending(0))
    {
      const uint16_t sid = (state << 3) | (_CanSid);
      auto buf = _Can.getTxBuffer();

      uint16_t usValue;

      usValue = _Clock.value();
      buf[0] = usValue >> 8;
      buf[1] = usValue & 0xFF;

      usValue = _CycleTime;
      buf[2] = usValue >> 8;
      buf[3] = usValue & 0xFF;

      usValue = _Values[state];
      buf[4] = usValue >> 8;
      buf[5] = usValue & 0xFF;

      _Can.transmitStandard(0, sid, 6);

      state = state + 1;
    }

    _CanState = state;
  }

  void handleLeds()
  {
    const uint16_t tmout = _LedTimer.hasTimedOut(_Clock);

    if(tmout)
    {
      uint8_t status = 0;

      if(tmout < 50)
        status |= LED_GREEN;
      else
        _LedTimer.updateTimeout(1000);

      setLedStatus(status);
    }
  }

  void cycle()
  {
    _Clock.update(GPIOR0);

    handleADCEvent();
    handleRegul();
    handleCan();
    handleLeds();
  }

  static void isrADC()
  {
    /* store that ADC was done, we need this because the ADIF is cleared because of the interrupt routine */
    GPIOR1 |= EV_ADC;
  }

  static void isrTick()
  {
    GPIOR0 = GPIOR0 + 1;
  }
};

const FlashBuffer<5,uint8_t>  App::ADMUXMapping PROGMEM =
{{
  static_cast<uint8_t>(ADMUX_VOUTA),
  static_cast<uint8_t>(ADMUX_IOUTA),
  static_cast<uint8_t>(ADMUX_VOUTB),
  static_cast<uint8_t>(ADMUX_IOUTB),
  static_cast<uint8_t>(ADMUX_VSOLAR),
}};

ISR(ADC_vect)
{
  Sys_AVR8::disableSleep();
  App::isrADC();
}

ISR(TIMER0_COMPA_vect)
{
  Sys_AVR8::disableSleep();
  App::isrTick();
}

int main()
{
  static App app;

  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  uint8_t i;
  for(i = 0; i < ElementCount(g_RegisterInit); ++i)
    g_RegisterInit[i].writeRegister();

  app.init();

  while(1)
  {
    Sys_AVR8::enableSleep();
    Sys_AVR8::enableInterrupts();

    app.cycle();

    Sys_AVR8::enterSleep();
    Sys_AVR8::disableInterupts();
  }
}


