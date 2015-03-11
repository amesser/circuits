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

const FlashBuffer<26,RegisterConfig> g_RegisterInit PROGMEM =
{{
  {DDRA, 0},
  {DDRB, 0},

  {GPIOR0, 0},
  {GPIOR1, 0},
  /* enable pll */
  {PLLCSR, _BV(PLLE)},

  /* configure adc */
  {DIDR0,  _BV(AREFD) | _BV(ADC3D) | _BV(ADC4D) | _BV(ADC5D) | _BV(ADC6D)},
  {DIDR1,  _BV(ADC9D)},
  {ADCSRA, _BV(ADEN)  | _BV(ADPS2) | _BV(ADPS1) |  _BV(ADATE) },
  {ADCSRB, _BV(ADTS2) | _BV(ADTS1)}, /* we want to be triggered by Timer 1 Overflow */

  /* configure tick timer */
  {TCCR0A, _BV(WGM00)},
  {OCR0A,  125-1},
  {TCCR0B, _BV(CS01) | _BV(CS00) | _BV(PSR0)},
  {TCNT0L,  0},

  /* preconfigure pwm timer */
  {OCR1C ,  0x7F},
  {TCCR1A, _BV(PWM1A)| _BV(PWM1B)},
  {TCCR1D, _BV(WGM10)},
  {TCCR1B, _BV(CS10)},
  {DT1, 0},

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

    void init(const Clock &clock, C iv)
    {
      _tmout = clock.value() + iv;
    }

    constexpr C value()
    {
      return _tmout;
    }

  };
};

/* ku long example parameters */
struct parameters
{
  uint16_t imax;
  uint8_t  ulow;
  uint8_t  ufull;
  uint8_t  umax;
  int8_t   coef;
};

struct parameters accu_prm =
{
  /* 25 °C */
  .imax  =  36, /* 360 mA maximum charge current */
  .ulow  = 105, /* 10.5 V */
  .ufull = 135, /* 13.5 V full */
  .umax  = 138, /* 13.8 V maximum charge voltage (used for balancing*) */
  .coef  = -18, /* in mv */
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

  ClockType::Timer<uint16_t>  _AdcTimer;
  ClockType::Timer<uint16_t> _CanTimer;
  ClockType::Timer<uint16_t> _LedTimer;
  ClockType::Timer<uint16_t> _SimTimer;

  uint16_t                 _LastCycle;
  uint16_t                 _CycleTime;
   int16_t                 _OffsetI[2];
  uint8_t                  _Duty;
  uint8_t                  _Hit;
  uint8_t                  _BuckState;

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

  enum RegulatorState
  {
    STATE_ADCSAMPLE_VOUTA   = 0,
    STATE_ADCSAMPLE_IOUTA   = 1,
    STATE_ADCSAMPLE_VOUTB   = 2,
    STATE_ADCSAMPLE_IOUTB   = 3,
    STATE_ADCSAMPLE_VSOLAR  = 4,
    STATE_UPDATE            = 5,
    STATE_IDLE              = 6,
  };

  enum BuckState
  {
    BUCK_STATE_OFF               = 0,
    BUCK_STATE_SETUP             = 1, /* configure pwm and wait 1 cycle */
    BUCK_STATE_START             = 2, /* startup buck and wait one cycle */
    BUCK_STATE_RUN               = 3, /* buck active and running */
  };
public:
  void handleBuck(uint8_t tmout)
  {
    const uint8_t max_duty = OCR1C - 1;
    uint8_t state          = _BuckState;
    uint8_t duty           = _Duty;

    if (duty > max_duty - 10)
      duty = max_duty - 10;

    /* adjust duty cycle to be at least 100 ns, otherwise the
     * buck will not start properly because of switching delays
     */
    if (BUCK_STATE_RUN > state && duty < 50 && duty > 0)
      duty = 50;

    /* update duty cycle */
    OCR1A = duty;
    OCR1B = max_duty + 1 - duty;

    if (duty)
    {
      if (BUCK_STATE_RUN > state && tmout)
      {
        state++;

        if (BUCK_STATE_START == state)
        {
          /* peek low side switch to generate some charge
           * for high side driver
           */
          PORTB |=  _BV(PB0);
          PORTB &= ~_BV(PB0);

          PORTB |=  _BV(PB3);
          PORTB &= ~_BV(PB3);

          /* disable the low side switches */
          DDRB   &= ~_BV(PB0);
          DDRB   &= ~_BV(PB3);

          /* connect pwm */
          TCCR1A |= _BV(COM1A0);
          TCCR1A |= _BV(COM1B0);
        }
      }
    }
    else
    {
      /* disconnect pwm */
      TCCR1A &= ~_BV(COM1A0);
      TCCR1A &= ~_BV(COM1B0);

      /* enable data output for low side switches
       * this forces them to be disabled
       */
      DDRB   |= _BV(PB0);
      DDRB   |= _BV(PB3);

      state = BUCK_STATE_OFF;
    }

    _BuckState = state;
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


    _CanState = 0;
    _AdcState = 0xFF;
    _CanSid   = (~_Can.getTxRtsPins()) &  0x7;
    _Duty     = 0;
    _Hit      = 0xFF;
    _BuckState = BUCK_STATE_OFF;

    _CanTimer.init(_Clock, 0);
    _LedTimer.init(_Clock, 10000);
    _SimTimer.init(_Clock, 1000);

  }

  void calibrate()
  {
    const uint8_t bak = (ADCSRA & ~_BV(ADEN)) | _BV(ADIF);
    int i;

    for(i = 0; i < 2; ++i)
    {
      ADCSRA  = bak;
      ADMUX   =  ADMUXMapping[STATE_ADCSAMPLE_IOUTA + i*2];
      ADCSRB |= _BV(BIN);
      ADCSRA  = bak | _BV(ADEN) | _BV(ADSC);

      while(ADCSRA & _BV(ADSC));
      _OffsetI[i] = ADC;

      ADCSRA |= _BV(ADIF);
    }
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
    if(_AdcState < STATE_UPDATE)
    {
      if((_AdcState % 2) == (STATE_ADCSAMPLE_IOUTA % 2))
      {
        ADCSRA &= ~_BV(ADEN);
        ADCSRB |= _BV(BIN);
      }
      else
      {
        ADCSRB &= ~_BV(BIN);
      }


      ADMUX  = ADMUXMapping[_AdcState];

      if((_AdcState % 2) == (STATE_ADCSAMPLE_IOUTA % 2))
        ADCSRA |= _BV(ADEN);

      TIFR   |= _BV(TOV1);
    }
  }


  void handleRegul(uint8_t tmout)
  {
    const uint8_t max_duty = OCR1C - 1;

    if(_AdcState == STATE_UPDATE)
    {
      const uint8_t vout   = static_cast<uint8_t>(_Values[VOUTA] + _Values[VOUTB] + 1) / 2;
      const uint8_t vsolar = static_cast<uint8_t>(_Values[VSOLAR]);

      const uint16_t iout = (_Values[IOUTA] + _Values[IOUTB]);

      const uint16_t imax = accu_prm.imax;
      const uint8_t  umax = accu_prm.ufull;

      uint8_t duty = _Duty;
      uint8_t hit;

      if(duty && _BuckState != BUCK_STATE_RUN)
      {
        hit = 0;
      }
      else if ((vout + 5) > vsolar)
      {
        duty = 0;
        hit = 1;
      }
      else if ((vout + 10) > vsolar)
      {
        uint8_t delta = vout + 10 - vsolar;
        delta *= 2;

        if (duty > delta)
          duty = duty - delta;
        else
          duty = 0;

        hit = 2;
      }
      else if (iout > imax)
      {
        uint16_t delta = iout - imax;

        delta = (delta + 3) / 4;

        if (duty > delta)
          duty = duty - delta;
        else
          duty = 0;

        hit = 3;
      }
      else if (vout > umax)
      {
        uint8_t delta = vout - umax;

        if (delta > 0 && duty > 0)
          duty = duty - 1;

        hit = 4;
      }
      else if (iout < 5 && duty > (max_duty / 2))
      {
        duty = max_duty / 2;
        hit = 5;
      }
      else if (vout < umax && iout < imax)
      {
        if (duty < max_duty)
          duty = duty + 1;

        hit = 6;
      }
      else
      {
        hit = 7;
      }

      _Hit = hit;
      _Duty = duty;

      ++_AdcState;
      perfFinishCycle();
    }
    else if (_AdcState >= STATE_IDLE && tmout)
    {
      perfStartCycle();

      _AdcState = STATE_ADCSAMPLE_VOUTA;
      startADCConversion();
    }
  }

  void handleSim(uint8_t tmout)
  {
    const uint8_t max_duty = OCR1C - 1;

    if(_AdcState == STATE_UPDATE)
    {
      if(_SimTimer.hasTimedOut(_Clock))
      {
        uint8_t duty = _Duty;

        _SimTimer.updateTimeout(1000);

        if(duty && _BuckState != BUCK_STATE_RUN)
        {
          ;
        }
        else if (duty == 0)
        {
          duty = 40;
          _Hit = 1;
        }
        else
        {
          if(_Hit)
          {
            if((duty + 10) < max_duty)
            {
              duty = duty + 10;
            }
            else
            {
              duty = max_duty;
              _Hit = 0;
            }
          }
          else
          {
            if(duty > 10)
            {
              duty = duty - 10;
            }
            else
            {
              duty = 0;
            }
          }

        }

       _Duty = duty;
        ++_AdcState;
      }
    }
    else if (_AdcState >= STATE_IDLE && tmout)
    {
      if(_Duty == 0)
        calibrate();

      perfStartCycle();

      _AdcState = STATE_ADCSAMPLE_VOUTA;
      startADCConversion();
    }
  }

  void handleADCEvent()
  {

    if(ADCSRA & _BV(ADIF))
    {
      uint8_t state = _AdcState;

      ADCSRA |= _BV(ADIF);

      if(state < STATE_UPDATE)
      {
        uint16_t value = ADC;

        _AdcState +=1;
        startADCConversion();

        if((state % 2) == (STATE_ADCSAMPLE_IOUTA % 2))
        {
          /* this was a current
           * 0,1 Ohm 20x, 1/11, vref 2.5 v
           * measure / 1024 / 20 * 2.5 * 11 / 0.1 * 1000
           * measure * 2.5 * 11 * 1000 / 1024 / 20 / 0.1
           * measure * 25 * 11 * 1000 / 4096 / 5
           * measure *  5 * 11 * 1000 / 4096
           * measure *  80 * 11 * 1000 / 65536
           *
           * compensate for adlar, bipolar and unit is 10mA
           * */
          const int32_t fact_mul = 80UL * 11 * 1000 * 2/ 64 / 10;
          const int32_t fact_div = 65536;

          int16_t sign_value = (int16_t)value - _OffsetI[(state - STATE_ADCSAMPLE_IOUTA) / 2];
          _Values[state] = (sign_value * fact_mul + fact_div / 2) / fact_div;
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
          _Values[state] = (value * fact_mul) / fact_div;
        }

      }

      if(_AdcState >= STATE_UPDATE)
      {
        perfFinishCycle();
      }

    }
  }

  void handleCan()
  {
    uint8_t state = _CanState;

    if (_CanTimer.hasTimedOut(_Clock))
    {
      _CanTimer.updateTimeout(500);

      if(state >= 6)
        state = 0;
    }

    if(state < 6 && !_Can.isTxPending(0))
    {
      const uint16_t sid = (state << 3) | (_CanSid);
      auto buf = _Can.getTxBuffer();
      uint8_t len = 0;


      if (state < 5)
      {
        uint16_t usValue;
        usValue = _Clock.value();
        buf[0] = 0;
        buf[1] = _Duty;

        usValue = _Values[state];
        buf[2] = usValue >> 8;
        buf[3] = usValue & 0xFF;
        len = 4;
      }
      else if (state == 5)
      {
        uint16_t usValue;
        usValue = _Clock.value();
        buf[0] = usValue >> 8;
        buf[1] = usValue & 0xFF;

        usValue = _CycleTime;
        buf[2] = usValue >> 8;
        buf[3] = usValue & 0xFF;


        buf[4] = _Duty;
        buf[5] = _Hit;

        len = 6;
      }

      _Can.transmitStandard(0, sid, len);
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

    uint8_t tmout = _AdcTimer.hasTimedOut(_Clock);

    if(tmout)
      _AdcTimer.updateTimeout(10);

    handleLeds();

    handleADCEvent();

    //handleRegul(tmout);
    handleSim(tmout);
    handleBuck(tmout);

    handleCan();
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

ISR(TIMER0_COMPA_vect)
{
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

  Sys_AVR8::enableInterrupts();

  while(1)
  {
    app.cycle();
  }
}


