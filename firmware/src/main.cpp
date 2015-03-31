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
  {ADCSRB, _BV(ADTS2) | _BV(ADTS1) }, /* we want to be triggered by Timer 1 Overflow */

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

const EEPROMBuffer<2,int8_t> g_TempCalibration EEMEM = {{0, 0,}};

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

template<typename TYPE, int SAMPLES>
class PT1
{
private:
  TYPE _sum;
public:

  void addValue(const TYPE value);
  TYPE getValue() const;

  TYPE getSum() const;

  TYPE operator - (const PT1 & rhs) const;


};

template<typename TYPE, int SAMPLES>
void PT1<TYPE, SAMPLES>::addValue(const TYPE value)
{
  TYPE s = _sum - getValue();
  _sum = s + value;
}


template<typename TYPE, int SAMPLES>
TYPE PT1<TYPE,SAMPLES>::getValue(void) const
{
  return (_sum + (TYPE)(SAMPLES/2)) / SAMPLES;
}

template<typename TYPE, int SAMPLES>
TYPE PT1<TYPE,SAMPLES>::getSum(void) const
{
  return _sum;
}


template<typename TYPE, int SAMPLES>
TYPE PT1<TYPE,SAMPLES>::operator - (const PT1 & rhs) const
{
  return (_sum - rhs._sum + SAMPLES/2) / SAMPLES;
}


struct parameters
{
  uint16_t icharge;
  uint16_t ibalance;
  uint16_t icharged;
  uint16_t ulow;
  uint16_t uwarning;
  uint16_t udischarging;
  uint16_t utrickle;
  uint16_t ucharge;
  uint16_t ubalance;
  int16_t  coef;
};

/* ku long example parameters */
#if 1
struct parameters accu_prm =
{
  /* 25 °C */
  .icharge      = 360,
  .ibalance     = 150,
  .icharged     = 75,
  .ulow         = 11800, /* 11.5 V */
  .uwarning     = 12200, /* 11.5 V */
  .udischarging = 12600, /* 12.5 V */
  .utrickle     = 13500, /* 13.5 V full */
  .ucharge      = 13700, /* 13.7 V maximum charge voltage (used for balancing*) */
  .ubalance     = 13800, /* 13.8 V maximum charge voltage (used for balancing*) */
  .coef         =   -18, /* in mv */
};
#else
struct parameters accu_prm =
{
  /* MOLL Soloar 100 Ah 25 °C */
  .icharge      = 10000,
  .ibalance     =  3500,
  .icharged     =  1000,
  .ulow         = 11800, /* 11.5 V */
  .uwarning     = 12200, /* 11.5 V */
  .udischarging = 12600, /* 12.5 V */
  .utrickle     = 13500, /* 13.5 V full */
  .ucharge      = 14100, /* 14.1 V maximum charge voltage (used for balancing*) */
  .ubalance     = 14400, /* 14.4 V maximum charge voltage (used for balancing*) */
  .coef         =   -24, /* in mv */
};
#endif

class App
{
private:
  typedef Clock<uint32_t> ClockType;

  MCP2515Driver<CanDevice> _Can;
  ClockType                _Clock;

  uint8_t                  _CanState;
  uint8_t                  _CanSid;

  uint8_t                  _AdcState;
  uint8_t                  _StartCycle;

  uint8_t                  _ChargerState;
  uint8_t                  _BatteryLimited;

  union {
    struct {
       int16_t                 _Currents[2];
       uint16_t                _Temp;
       uint16_t                _Voltages[3];
    };

    struct {
      int16_t  _IChargeA;
      int16_t  _IChargeB;
      uint16_t _TKelvin;
      uint16_t _UBatA;
      uint16_t _UBatB;
      uint16_t _USolar;
    };
  };

  uint16_t                     _UBattery;
  uint16_t                     _ICharge;

  ClockType::Timer<uint16_t> _AdcTimer;
  ClockType::Timer<uint16_t> _CanTimer;
  ClockType::Timer<uint16_t> _LedTimer;
  ClockType::Timer<uint16_t> _BatTimer;
  ClockType::Timer<uint16_t> _MPPTTimer;
  ClockType::Timer<uint32_t> _TempTimer;

  uint16_t                 _LastCycle;
  uint16_t                 _CycleTime;
  uint8_t                  _Duty;
  uint8_t                  _BuckState;
  uint8_t                  _BatState;

  PT1<int16_t, 64>         _IOffset[2];
  PT1<int16_t, 64>         _IAvg[2];

  uint8_t                  _CntOffset;
  uint8_t                  _MPPTCnt;

  uint32_t                 _Pwr;
  PT1<uint16_t, 64>        _PAvg;

  uint16_t                 _TrackPwr;
  uint16_t                 _TrackU;

  uint16_t                 _ITrack;
  int8_t                   _DutyTrack;

  static const FlashBuffer<9,uint8_t> ADMUXMapping;

  enum Led {
    LED_RED   = 0x01,
    LED_GREEN = 0x02,
    LED_BOTH  = 0x03,
  };

  enum AdcState
  {
    STATE_CONTROL_IDLE  = 0,
    STATE_ADC_SAMPLETEMP,
    STATE_ADC_CALIBRATEIOUTA,
    STATE_ADC_CALIBRATEIOUTB,
    STATE_ADC_SAMPLEIOUTA,
    STATE_ADC_SAMPLEIOUTB,
    STATE_ADC_SAMPLEVOUTA,
    STATE_ADC_SAMPLEVOUTB,
    STATE_ADC_SAMPLEVSOLAR,
    STATE_CONTROL_CALC,
  };

  enum BuckState
  {
    BUCK_STATE_OFF               = 0,
    BUCK_STATE_SETUP             = 1, /* configure pwm and wait 1 cycle */
    BUCK_STATE_START             = 2, /* startup buck and wait one cycle */
    BUCK_STATE_RUN               = 3 + 64, /* buck active and running */
    BUCK_STATE_SHUTDOWN          = 4 + 64, /* buck active and running */
    BUCK_STATE_DISABLED          = 4 + 64 + 20, /* buck active and running */
  };

  enum ChargerState
  {
    CHARGER_STATE_TRICKLE,
    CHARGER_STATE_NORMAL,
    CHARGER_STATE_BALANCE,
  };

  enum BatteryState
  {
    BATTERY_STATE_EMPTY   = 0,
    BATTERY_STATE_WARNING = 1,
    BATTERY_STATE_HALF    = 2,
    BATTERY_STATE_FULL    = 3,
  };

  enum MPPTState
  {
    MPPT_CNT_OFF        = 0,
    MPPT_CNT_START      = 1,
    MPPT_CNT_TRACK      = 16,
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

    if (BUCK_STATE_RUN == state &&
        (_IChargeA < 1 || _IChargeB < 1))
    {
      /* converter not running anymore,
       * force restart */
      state = BUCK_STATE_DISABLED;
      duty = 0;
    }
    else if (BUCK_STATE_SHUTDOWN <= state)
    {
      /* buck is shutting down, force buck beeing disabled */
      duty = 0;
    }

    /* update duty cycle */
    OCR1A = duty;
    OCR1B = max_duty + 1 - duty;

    if (duty)
    {
      if (BUCK_STATE_START >= state && tmout)
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

      if (state >= BUCK_STATE_DISABLED)
      {
        state = BUCK_STATE_OFF;
      }
      else if (state >= BUCK_STATE_SHUTDOWN)
      {
        if(tmout)
          state++;
      }
      else if (state != BUCK_STATE_OFF)
      {
        state = BUCK_STATE_SHUTDOWN;
      }
    }

    _BuckState = state;
  }

  void setLedStatus(uint8_t mask)
  {
    if(GPIOR2 != mask)
    {
      _Can.setRxBfPins(~mask, 0x3);
      GPIOR2 = mask;
    }
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

    _CanState     = 0;
    _StartCycle   = false;
    _AdcState     = STATE_CONTROL_IDLE;
    _CanSid       = (~_Can.getTxRtsPins()) &  0x7;
    _Duty         = 0;
    _BuckState    = BUCK_STATE_OFF;
    _ChargerState = CHARGER_STATE_NORMAL;
    _CntOffset    = 0;
    _BatState     = BATTERY_STATE_HALF;
    _MPPTCnt      = MPPT_CNT_OFF;

    _CanTimer.init(_Clock, 0);
    _LedTimer.init(_Clock, 10000);
    _BatTimer.init(_Clock, 10000);
    _TempTimer.init(_Clock, 1000UL * 60 * 15);
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

  uint16_t getChargeVoltage()
  {
    uint8_t state = _ChargerState;
    uint16_t u;

    switch(state)
    {
    case CHARGER_STATE_TRICKLE:
      u = accu_prm.utrickle;
      break;
    case CHARGER_STATE_NORMAL:
      u = accu_prm.ucharge;
      break;
    case CHARGER_STATE_BALANCE:
      u = accu_prm.ubalance;
      break;
    default:
      u = accu_prm.utrickle;
      break;
    }
    return u;
  }

  uint16_t getChargeCurrent()
  {
    uint8_t state = _ChargerState;
    uint16_t icharge;

    if (state == CHARGER_STATE_BALANCE)
    {
      if(_UBattery > accu_prm.ucharge)
      {
        icharge = accu_prm.ibalance;
      }
      else if ((_UBattery + 128) > accu_prm.ucharge)
      {
        icharge = accu_prm.icharge - accu_prm.ibalance;
        icharge = (int32_t)icharge * (accu_prm.ucharge - _UBattery) / 128;
        icharge += accu_prm.ibalance;
      }
      else
      {
        icharge = accu_prm.icharge;
      }
    }
    else
    {
      icharge = accu_prm.icharge;
    }

    return icharge;
  }

  void updateBatteryState()
  {
    bool    retrigger = false;
    uint8_t state = _BatState;

    uint16_t tmout = _BatTimer.hasTimedOut(_Clock);


    switch(state)
    {
    case BATTERY_STATE_EMPTY:
      if (_UBattery < accu_prm.udischarging)
        retrigger = true;
      else if (tmout)
        state = BATTERY_STATE_HALF;
      break;
    case BATTERY_STATE_WARNING:
      if (_UBattery > accu_prm.ulow)
        retrigger = true;
      else
        state = BATTERY_STATE_EMPTY;
      break;
    case BATTERY_STATE_HALF:
      if (_UBattery > accu_prm.uwarning)
        retrigger = true;
      else
        state = BATTERY_STATE_WARNING;
      break;
    case BATTERY_STATE_FULL:
      if (_UBattery > accu_prm.udischarging)
        retrigger = true;
      else if (tmout)
        state = BATTERY_STATE_HALF;
      break;
    }

    if(retrigger || tmout)
      _BatTimer.init(_Clock, 10000);

    _BatState = state;
  }

  void updateChargerState()
  {
    uint8_t state = _ChargerState;
    uint8_t batstate = _BatState;

    if(batstate <= BATTERY_STATE_WARNING)
       state = CHARGER_STATE_BALANCE;

    switch(state)
    {
    case CHARGER_STATE_TRICKLE:
      {
        if (batstate != BATTERY_STATE_FULL)
          state = CHARGER_STATE_NORMAL;
      }
      break;
    case CHARGER_STATE_NORMAL:
      {
        if ((_ICharge < accu_prm.icharged) &&
            _BatteryLimited > 10)
        {
          _BatState = BATTERY_STATE_FULL;
          state = CHARGER_STATE_TRICKLE;
        }
      }
      break;
    case CHARGER_STATE_BALANCE:
      {
        if ((_UBattery + 50) > accu_prm.ubalance)
          state = CHARGER_STATE_NORMAL;
      }
      break;
    }

    _ChargerState = state;
  }

  enum {
    MSK_GPIOR1_STARTCYCLE     = 0x01,
    MSK_GPIOR1_RECALIBRATE    = 0x04,
  };

  void setStartCycle()
  {
    GPIOR1 |= MSK_GPIOR1_STARTCYCLE;
  }

  bool getStartCycle()
  {
    return (GPIOR1 & MSK_GPIOR1_STARTCYCLE);
  }

  void confirmStartCycle()
  {
    GPIOR1 &= ~MSK_GPIOR1_STARTCYCLE;
  }

  void setRecalibrate()
  {
    GPIOR1 |= MSK_GPIOR1_RECALIBRATE;
  }

  bool getRecalibrate()
  {
    return (GPIOR1 & MSK_GPIOR1_RECALIBRATE);
  }

  void confirmRecalibrate()
  {
    GPIOR1 &= ~MSK_GPIOR1_RECALIBRATE;
  }

  void enterADCState(uint8_t state)
  {
    if (state != _AdcState)
    {
      if(state == STATE_ADC_SAMPLETEMP)
      {
        ADCSRA &= ~_BV(ADEN);
        ADCSRB &= ~_BV(BIN);
        ADCSRB |=  _BV(MUX5);
      }
      else if(state >= STATE_ADC_CALIBRATEIOUTA &&
         state <= STATE_ADC_SAMPLEIOUTB)
      {
        ADCSRA &= ~_BV(ADEN);
        ADCSRB |=   _BV(BIN);
        ADCSRB &=  ~_BV(MUX5);
      }
      else if(state >= STATE_ADC_SAMPLEVOUTA &&
              state <= STATE_ADC_SAMPLEVSOLAR)
      {
        ADCSRB &= ~_BV(BIN);
        ADCSRB &=  ~_BV(MUX5);
      }

      if(state >= STATE_ADC_SAMPLETEMP &&
         state <= STATE_ADC_SAMPLEVSOLAR)
      {
        ADMUX  = ADMUXMapping[state];

        ADCSRA |= _BV(ADEN);
        TIFR   |= _BV(TOV1);
      }

      if(state == STATE_CONTROL_CALC)
      {
        /* in start up phase of buck regulator we must wait until the average current reflects
         * the real current before the buck is switched to state 'run'. therefore after each
         * adc measurement increment the buck state until it reaches buck state run
         */
        if (_BuckState > BUCK_STATE_START && _BuckState < BUCK_STATE_RUN)
        {
          _BuckState = _BuckState + 1;
        }

        updateBatteryState();
        updateChargerState();
      }
    }

    _AdcState = state;
  }


  void handleRegul(uint8_t tmout)
  {
    uint8_t state = _AdcState;

    if(tmout && _CntOffset >= 0xFF)
      setStartCycle();

    if(state == STATE_CONTROL_IDLE)
    {
      if(getStartCycle())
      {
        confirmStartCycle();
        state = STATE_ADC_SAMPLEIOUTA;
      }
      else if (_Duty == 0 && _BuckState == BUCK_STATE_OFF)
      {
        state = STATE_ADC_SAMPLETEMP;
      }
    }
    else if (state == STATE_CONTROL_CALC)
    {
      /* compute new duty cycle value */
      uint8_t  duty     = _Duty;
      int8_t   dduty    = 0;

      int16_t  du = (_USolar-_UBattery);

      if (_BuckState > BUCK_STATE_START &&
          _BuckState < BUCK_STATE_RUN)
      {
        /* while buck is starting up do not play with duty cycle */
      }
      else if(getRecalibrate())
      {
        /* recalibaration of current and temperature measurement requested */
        duty = 0;
        _BatteryLimited = 0;
      }
      else if(du > 1000)
      {
        const uint16_t ucharge = getChargeVoltage();
        const uint16_t icharge = getChargeCurrent();

        if (_ICharge > icharge)
        {
          dduty = -1;
          _BatteryLimited = 0;
        }
        else if (_UBattery > ucharge)
        {
          dduty = -1;

          if(_BatteryLimited < 100)
            _BatteryLimited += 10;
        }
        else if ((_UBattery + 25) < ucharge)
        {
          if (_USolar < (_UBattery + 1500))
          {
            dduty = -1;
          }
          else if(duty == 0)
          {
            dduty = 50;
          }
          if(_ICharge < 100)
          {
            dduty = 1;
          }
          else if (_MPPTCnt >= 128)
          {
            if(_DutyTrack < duty)
              dduty = 1;
            else
              dduty = -1;

            if((_ICharge + 3) < _ITrack)
            {
              dduty = -dduty;

              if(_ICharge > 25)
                _ITrack = _ICharge - 25;
              else
                _ITrack = 0;
            }
          }

          if(dduty > 0 && _BatteryLimited > 0)
            _BatteryLimited -= 1;
        }
      }
      else
      {
        duty = 0;
        _BatteryLimited = 0;
      }


      if(dduty <= 0)
      {
        if(duty < (-dduty))
          duty = 0;
        else
          duty += dduty;
      }
      else
      { /* calculate maximum duty according input and output voltage of
         * buck */
        const uint8_t  max_pwmduty = OCR1C - 1;
        const uint16_t max_calcduty = max_pwmduty * (_UBattery/256) / (_USolar/256);
        uint8_t max_duty;

        if (max_pwmduty > max_calcduty + 2)
          max_duty = max_calcduty + 2;
        else
          max_duty = max_pwmduty;

        if ((dduty + duty) > max_duty)
          duty = max_duty;
        else
          duty += dduty;
      }

      if((_ICharge + 25) > _ITrack)
        _ITrack = _ICharge - 25;
      else if (_ICharge < (_ITrack + 25))
        _ITrack = _ICharge + 25;


      if((duty != _Duty) || duty == 0)
      {
        _MPPTCnt   = 0;
        _DutyTrack = _Duty;
      }
      else
      {
        if(_MPPTCnt < 128)
          _MPPTCnt += 1;
      }

      _Duty    = duty;

      state = STATE_CONTROL_IDLE;
    }


    enterADCState(state);
  }

  void handleADCEvent()
  {
    if(ADCSRA & _BV(ADIF))
    {
      ADCSRA |= _BV(ADIF);

      uint8_t  state = _AdcState;
      uint16_t val   = ADC;

      if(state >= STATE_ADC_SAMPLEIOUTA && state <= STATE_ADC_SAMPLEVSOLAR)
        enterADCState(state + 1);
      else if (state >= STATE_ADC_SAMPLETEMP && state < STATE_ADC_CALIBRATEIOUTB)
        enterADCState(state + 1);
      else if (state == STATE_ADC_CALIBRATEIOUTB)
      {
        if (_CntOffset < 0xFF)
          _CntOffset += 1;

        _TempTimer.init(_Clock, 1000UL * 60 * 15);
        confirmRecalibrate();

        enterADCState(STATE_CONTROL_IDLE);
      }

      switch(state)
      {
      case STATE_ADC_SAMPLETEMP:
        {
          int16_t temp = val;

          temp += temp * g_TempCalibration[0] / 256;
          temp += g_TempCalibration[1];

          _Temp = temp;
        }
        break;
      case STATE_ADC_CALIBRATEIOUTA:
      case STATE_ADC_CALIBRATEIOUTB:
        {
          const int index = state - STATE_ADC_CALIBRATEIOUTA;
          _IOffset[index].addValue(val);
        }
        break;
      case STATE_ADC_SAMPLEIOUTA:
      case STATE_ADC_SAMPLEIOUTB:
        {
          const int index = state - STATE_ADC_SAMPLEIOUTA;

          _IAvg[index].addValue(val);

          /* a current was sampled */
          /* this was a current
           * 0,05 Ohm 20x, 1/11, vref 2.5 v
           * measure / 1024 / 20 * 2.5 * 11 / 0.05 * 1000
           * measure * 2.5 * 11 * 1000 / 1024 / 20 / 0.05
           * measure * 50 * 11 * 1000 / 4096 / 5
           * measure *  10 * 11 * 1000 / 4096
           * measure *  160 * 11 * 1000 / 65536
           *
           * compensate for adlar, bipolar and unit is mA
           * */

          const int32_t fact_mul = 160UL * 11 * 1000 * 2 / 64;
          const int32_t fact_div = 65536;

          int16_t sign_value = _IAvg[index].getSum() - _IOffset[index].getSum();
          _Currents[index] = (sign_value * fact_mul + fact_div / 2) / fact_div;
        }
        break;
      case STATE_ADC_SAMPLEVOUTA:
      case STATE_ADC_SAMPLEVOUTB:
      case STATE_ADC_SAMPLEVSOLAR:
        {
          /* this is a voltage
           * 1/11, vref 2.5V
           * measure / 1024 * 2.5 * 11 * 1000
           * measure * 5 * 11 * 1000 * 32 / 65536
           *
           * compensate for adlar and unit is mV
           */
          const uint32_t fact_mul = 160UL * 11 * 1000 / 64;
          const uint32_t fact_div = 65536;
          _Voltages[state - STATE_ADC_SAMPLEVOUTA] = (val * fact_mul) / fact_div;
        }
        break;
      }

      if(state == STATE_ADC_SAMPLEVOUTB)
      {
        int16_t I = _Currents[0] + _Currents[1];

        if(I < 0)
          _ICharge = 0;
        else
          _ICharge = I;

        _UBattery = (_Voltages[0] + _Voltages[1] + 1) / 2;

        _Pwr = (uint32_t)_ICharge * _UBattery;

        _PAvg.addValue((uint32_t)(_Pwr >> 16UL));
      }
    }
  }

  void handleCan()
  {
    uint8_t state = _CanState;

    if (_CanTimer.hasTimedOut(_Clock))
    {
      _CanTimer.updateTimeout(500);

      if(state >= 3)
        state = 0;
    }

    if(state < 3 && !_Can.isTxPending(0))
    {
      const uint16_t sid = (state << 3) | (_CanSid);
      auto buf = _Can.getTxBuffer();
      uint8_t len = 0;

      if(state == 0)
      {
        uint16_t usValue;

        buf[0]  = _BatState;

        usValue = _UBattery;

        buf[1] = usValue >> 8;
        buf[2] = usValue & 0xFF;

        usValue = _Temp;

        buf[3] = usValue >> 8;
        buf[4] = usValue & 0xFF;

        len = 5;
      }
      else if(state == 1)
      {
        uint16_t usValue;

        buf[0]  = _ChargerState;

        usValue = _USolar;

        buf[1] = usValue >> 8;
        buf[2] = usValue & 0xFF;

        usValue = _ICharge;

        buf[3] = usValue >> 8;
        buf[4] = usValue & 0xFF;

        uint32_t ulValue = _Pwr / 1000;

        buf[5] = (ulValue >> 16) & 0xFF;
        buf[6] = (ulValue >> 8)  & 0xFF;
        buf[7] = (ulValue >> 0)  & 0xFF;

        len = 8;
      }
      else if(state == 2)
      {
        uint16_t usValue;

        buf[0]  = _MPPTCnt;

        usValue = _ITrack;

        buf[1] = usValue >> 8;
        buf[2] = usValue & 0xFF;

        usValue = _ICharge;

        buf[3] = usValue >> 8;
        buf[4] = usValue & 0xFF;

        buf[5] =  _DutyTrack;

        buf[6] =  _Duty;

        len = 7;
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

      if(_ChargerState == CHARGER_STATE_TRICKLE)
        status |= LED_GREEN;
      else if (_ICharge > 50)
        status |= LED_BOTH;
      else
        status |= LED_RED;

      if(_ChargerState == CHARGER_STATE_TRICKLE &&
          _USolar > (_UBattery + 1000))
      {
        _LedTimer.updateTimeout(1000);
      }
      else if(tmout > 50)
      {
         status = 0;
        _LedTimer.updateTimeout(1000);
      }

      setLedStatus(status);
    }
  }

  void cycle()
  {
    _Clock.update(GPIOR0);

    uint8_t tmout = _AdcTimer.hasTimedOut(_Clock);

    if(tmout)
      _AdcTimer.updateTimeout(5);

    if(_TempTimer.hasTimedOut(_Clock))
      setRecalibrate();

    handleLeds();

    handleADCEvent();

    handleRegul(tmout);
    //handleSim(tmout);
    handleBuck(tmout);

    handleCan();
  }

  static void isrTick()
  {
    GPIOR0 = GPIOR0 + 1;
  }
};

const FlashBuffer<9,uint8_t>  App::ADMUXMapping PROGMEM =
{{
    0,
    _BV(REFS1)              | 0x1F,
    _BV(REFS0)              | 0x12,
    _BV(REFS0)              | 0x17,
    _BV(REFS0) /* | _BV(ADLAR) */ | 0x12,
    _BV(REFS0) /* | _BV(ADLAR) */ | 0x17,
    _BV(REFS0) | _BV(ADLAR) | 0x3,
    _BV(REFS0) | _BV(ADLAR) | 0x5,
    _BV(REFS0) | _BV(ADLAR) | 0x9,
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



