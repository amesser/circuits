/*
 * main.cpp
 *
 *  Created on: 10.02.2015
 *      Author: andi
 */
#include "util/guard.hpp"
#include "ecpp/Byteorder.hpp"
#include "ecpp/Datatypes.hpp"
#include "ecpp/Arch/AVR8.hpp"
#include "ecpp/Target.hpp"
#include "ecpp/Filter.hpp"

using namespace ecpp;

using namespace Platform::Guards;

typedef USISPIMaster<1000000> SPIDriverType;

static constexpr uint8_t param_maxpwmduty = 0x7F - 1;

struct adc_settings
{
  uint8_t admux;
  uint8_t adcsrb;
};

struct AccuPrm
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
  uint16_t tlow;
  uint16_t thigh;
  int16_t  coef;
};

struct EEDataLayout
{
  EEVariable<RegisterConfig> RegisterInit[25];
  EEVariable<adc_settings>   ADCSettings[9];
  EEVariable<uint16_t>       CanIds[3];
  EEVariable<int8_t>         TempCalibration[2];
  EEVariable<AccuPrm>        AccuParameters;
  EEVariable<uint16_t>       MPPTVoltage;
  EEVariable<uint8_t >       CanSid;
};

const struct EEDataLayout EEData EEMEM =
{
  .RegisterInit = {
    {{DDRA, 0}},
    {{DDRB, 0}},

    {{GPIOR0, 0}},
    {{GPIOR1, 0}},
    /* enable pll */
    {{PLLCSR, _BV(PLLE)}},

    /* configure adc */
    {{DIDR0,  _BV(AREFD) | _BV(ADC3D) | _BV(ADC4D) | _BV(ADC5D) | _BV(ADC6D)}},
    {{DIDR1,  _BV(ADC9D)}},
    {{ADCSRA, _BV(ADEN)  | _BV(ADPS2) | _BV(ADPS1) |  _BV(ADATE) }},

    /* configure tick timer */
    {{TCCR0A, _BV(WGM00)}},
    {{OCR0A,  125-1}},
    {{TCCR0B, _BV(CS01) | _BV(CS00) | _BV(PSR0)}},
    {{TCNT0L,  0}},

    /* preconfigure pwm timer */
    {{OCR1C ,  param_maxpwmduty + 1}},
    {{TCCR1A, _BV(PWM1A)| _BV(PWM1B)}},
    {{TCCR1D, _BV(WGM10)}},
    {{TCCR1B, _BV(CS10)}},
    {{DT1, 0}},

    /* configure ports */
    {{PORTA,  0}},
    {{PORTB, _BV(PB5)}},
    {{DDRA,  _BV(PA1) | _BV(PA2)}},
    {{DDRB,  _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB5)}},
    {{USIPP, _BV(USIPOS)}},
    SPIDriverType::RegisterInit<SPIDriverType::SPI_MODE3>(),

    /* configure timer interrupts */
    {{TIMSK, _BV(OCIE0A)}},
    /* configure sleep mode idle */
    {{MCUCR, 0}},
  },

  .ADCSettings = {
    {{                             0, _BV(ADTS2) | _BV(ADTS1)}},
    {{_BV(REFS0)              | 0x12, _BV(ADTS2) | _BV(ADTS1) | _BV(BIN)}},
    {{_BV(REFS0)              | 0x17, _BV(ADTS2) | _BV(ADTS1) | _BV(BIN)}},
    {{_BV(REFS1)              | 0x1F, _BV(ADTS2) | _BV(ADTS1) | _BV(MUX5)}},
    {{_BV(REFS0)              | 0x12, _BV(ADTS2) | _BV(ADTS1) | _BV(BIN)}},
    {{_BV(REFS0)              | 0x17, _BV(ADTS2) | _BV(ADTS1) | _BV(BIN)}},
    {{_BV(REFS0) | _BV(ADLAR) |  0x3, _BV(ADTS2) | _BV(ADTS1)}},
    {{_BV(REFS0) | _BV(ADLAR) |  0x5, _BV(ADTS2) | _BV(ADTS1)}},
    {{_BV(REFS0) | _BV(ADLAR) |  0x9, _BV(ADTS2) | _BV(ADTS1)}},
  },

  .CanIds = {
    0x000,
    0x600,
    0x700
  },

  . TempCalibration = {
      -68, 67
  },

#if 0
  .AccuParameters = {{
    /* ku long example parameters */
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
    .tlow         = 298,
    .thigh        = 298,
    .coef         =   0 /*-18*/, /* in mv */
  }},

   .MPPTVoltage = 0,
#else
    .AccuParameters = {{
     /* ku long example parameters */
     /* MOLL Soloar 100 Ah 20 °C */
    .icharge      = 10000,
    .ibalance     =  3500,
    .icharged     =  1000,
    .ulow         = 11800, /* 11.5 V */
    .uwarning     = 12200, /* 11.5 V */
    .udischarging = 12600, /* 12.5 V */
    .utrickle     = 13500, /* 13.5 V full */
    .ucharge      = 14100, /* 14.1 V maximum charge voltage (used for balancing*) */
    .ubalance     = 14400, /* 14.4 V maximum charge voltage (used for balancing*) */
    .tlow         = 273 + 10, /* Moll sugests no temp compensation between these two temperatures */
    .thigh        = 273 + 30,
    .coef         =   -24, /* in mv */
  }},

  .MPPTVoltage = 18200, /* MPP Voltage according solar module */
#endif

  .CanSid       = 0,
};

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
  void update(C val)
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

  void init(T value)
  {
    _cnt = value;
  }

  template<typename C>
  class Timer
  {
  private:
    C _tmout;
  public:
    C hasTimedOut(const Clock & clock);
    void updateTimeout(C iv)
    {
      _tmout += iv;
    }

    void init(C iv);
    void init(const Clock &clock, C iv);

    constexpr C value()
    {
      return _tmout;
    }
  };
};

template<typename T>
template<typename C>
C Clock<T>::Timer<C>::hasTimedOut(const Clock & clock)
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

template<typename T>
template<typename C>
void Clock<T>::Timer<C>::init(const Clock &clock, C iv)
{
  _tmout = clock.value() + iv;
}

template<typename T>
template<typename C>
void Clock<T>::Timer<C>::init(C iv)
{
  _tmout = iv;
}








class App
{
private:
  typedef Clock<uint16_t> ClockType;

  MCP2515Driver<CanDevice> _Can;
  ClockType                _Clock1ms;
  ClockType                _Clock30s;

  uint8_t                  _CanState;
  uint8_t                  _CanSid;

  uint8_t                  _AdcState;

  uint8_t                  _ChargerState;
  union {
    struct {
       uint16_t                _Currents[2];
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
  ClockType::Timer<uint16_t> _HalfMinuteTimer;
  ClockType::Timer<uint16_t> _TempTimer;
  ClockType::Timer<uint16_t> _BalanceTimer;

  uint8_t                  _Duty;
  uint8_t                  _BuckState;
  uint8_t                  _BatState;
  uint8_t                  _BatLimitCnt;

  PT1<int16_t, 64>         _IOffset[2];
  PT1<int16_t, 64>         _IAvg[2];
  PT1<uint32_t, 256>       _IChargeAvg;

  uint16_t                 _CyclesSinceDutyInc;
  uint16_t                 _CyclesSinceDutyDec;

  uint16_t                 _MPPTVoltage;

  uint16_t                 _ITrack;
  int8_t                   _TDuty;
  uint8_t                  _Hit;

  struct AccuPrm           _accuprm;

  enum Led {
    LED_RED   = 0x01,
    LED_GREEN = 0x02,
    LED_BOTH  = 0x03,
  };

  enum AdcState
  {
    STATE_CONTROL_IDLE  = 0,
    STATE_ADC_CALIBRATEIOUTA,
    STATE_ADC_CALIBRATEIOUTB,
    STATE_ADC_SAMPLETEMP,
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

  enum {
    MSK_GPIOR1_STARTCYCLE     = 0x01,
    MSK_GPIOR1_BATTERYLIMIT   = 0x02,
    MSK_GPIOR1_RECALIBRATE    = 0x04,
    MSK_GPIOR1_ENABLED        = 0x08,
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

  void setEnabled()
  {
    GPIOR1 |= MSK_GPIOR1_ENABLED;
  }

  bool getEnabled()
  {
    return (GPIOR1 & MSK_GPIOR1_ENABLED);
  }


  void incBatteryLimit()
  {
    if(_BatLimitCnt < 40)
    {
      _BatLimitCnt += 10;
    }
    else
    {
      _BatLimitCnt  = 50;
    }
  }

  void clearBatteryLimit()
  {
    _BatLimitCnt = 0;
  }

  void decBatteryLimit()
  {
    if(_BatLimitCnt > 0)
    {
      _BatLimitCnt -= 1;
    }
  }

  bool getBatteryLimit()
  {
    return (_BatLimitCnt > 20);
  }



public:
  void handleBuck(uint8_t tmout)
  {
    const uint8_t max_duty = param_maxpwmduty;
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
      duty = 0;
    }

    uint8_t lastduty = OCR1A;

    if(duty > lastduty)
      _CyclesSinceDutyInc = 0;
    else if (tmout && _CyclesSinceDutyInc < 0xFFFF)
      _CyclesSinceDutyInc += 1;

    if(duty < lastduty)
      _CyclesSinceDutyDec = 0;
    else if (tmout && _CyclesSinceDutyDec < 0xFFFF)
      _CyclesSinceDutyDec += 1;

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

      state = BUCK_STATE_OFF;
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

    if (EEData.CanSid < 0x8)
      _CanSid = EEData.CanSid;
    else
      _CanSid       = (~_Can.getTxRtsPins()) & 0x7;

    _ChargerState = CHARGER_STATE_NORMAL;
    _BatState     = BATTERY_STATE_HALF;

    _LedTimer.init(30000);
    _BatTimer.init(10000);
    _HalfMinuteTimer.init(30000);

    _TempTimer.init(30);
    _BalanceTimer.init(2 * 60 * 24 * 7);

    _accuprm     = EEData.AccuParameters;
    _MPPTVoltage = EEData.MPPTVoltage;
  }

  uint16_t getChargeVoltage()
  {
    uint8_t state = _ChargerState;
    uint16_t u;

    int16_t utemp;

    if(_Temp > _accuprm.thigh)
    {
      utemp = (_Temp - _accuprm.thigh);
    }
    else if (_Temp < _accuprm.tlow)
    {
      utemp = (_Temp - _accuprm.tlow);
    }
    else
    {
      utemp = 0;
    }

    utemp = utemp * _accuprm.coef;

    switch(state)
    {
    case CHARGER_STATE_TRICKLE:
      u = _accuprm.utrickle;
      break;
    case CHARGER_STATE_NORMAL:
      u = _accuprm.ucharge + utemp;
      break;
    case CHARGER_STATE_BALANCE:
      u = _accuprm.ubalance + utemp;
      break;
    default:
      u = _accuprm.utrickle;
      break;
    }

    /* some hard limits to protect accu */
    if (u > 14700)
      u = 14700;
    else if (u < 12000)
      u = 12000;

    return u;
  }

  uint16_t getChargeCurrent()
  {
    uint8_t state = _ChargerState;
    uint16_t icharge;

    if (state == CHARGER_STATE_BALANCE)
    {
      if(_UBattery > _accuprm.ucharge)
      {
        icharge = _accuprm.ibalance;
      }
      else if ((_UBattery + 256) > _accuprm.ucharge)
      {
        icharge = _accuprm.icharge - _accuprm.ibalance;
        icharge = (uint32_t)icharge * (_accuprm.ucharge - _UBattery) / 256;
        icharge += _accuprm.ibalance;
      }
      else
      {
        icharge = _accuprm.icharge;
      }
    }
    else
    {
      icharge = _accuprm.icharge;
    }

    return icharge;
  }

  void updateBatteryState()
  {
    uint8_t state = _BatState;

    switch(state)
    {
    case BATTERY_STATE_EMPTY:
      if (_UBattery > _accuprm.udischarging)
        state = BATTERY_STATE_HALF;
      break;
    case BATTERY_STATE_WARNING:
      if (_UBattery < _accuprm.ulow)
        state = BATTERY_STATE_EMPTY;
      else if (_UBattery > _accuprm.udischarging)
        state = BATTERY_STATE_HALF;
      break;
    case BATTERY_STATE_HALF:
      if (_UBattery < _accuprm.uwarning)
        state = BATTERY_STATE_WARNING;
      else if (getBatteryLimit() && _IChargeAvg.getValue() < _accuprm.icharged)
        state = BATTERY_STATE_FULL;
      break;
    case BATTERY_STATE_FULL:
      if (_UBattery < _accuprm.udischarging)
        state = BATTERY_STATE_HALF;
      break;
    }

    uint16_t tmout = _BatTimer.hasTimedOut(_Clock1ms);

    if(state == _BatState || tmout)
    {
      _BatTimer.init(_Clock1ms, 10000);
    }

    if(tmout)
      _BatState = state;
  }

  void updateChargerState()
  {
    uint8_t state = _ChargerState;
    uint8_t batstate = _BatState;

    if(batstate <= BATTERY_STATE_WARNING)
       state = CHARGER_STATE_BALANCE;

    if(_BalanceTimer.hasTimedOut(_Clock30s))
      state = CHARGER_STATE_BALANCE;

    switch(state)
    {
    case CHARGER_STATE_TRICKLE:
      {
        if (batstate != BATTERY_STATE_FULL)
        {
          state = CHARGER_STATE_NORMAL;
        }
      }
      break;
    case CHARGER_STATE_NORMAL:
      {
        if (batstate == BATTERY_STATE_FULL)
        {
          state = CHARGER_STATE_TRICKLE;
        }
      }
      break;
    case CHARGER_STATE_BALANCE:
      {
        _BalanceTimer.init(_Clock30s, 2 * 60 * 24 * 7);

        if (getBatteryLimit())
          state = CHARGER_STATE_NORMAL;
      }
      break;
    }

    _ChargerState = state;
  }


  void enterADCState(uint8_t state)
  {
    if (state != _AdcState)
    {
      if (state <= STATE_ADC_SAMPLEVOUTA)
      {
        ADCSRA &= ~_BV(ADEN);
      }

      if(state >  STATE_CONTROL_IDLE &&
         state <  STATE_CONTROL_CALC)
      {
        struct adc_settings s = EEData.ADCSettings[state];

        ADCSRB =  s.adcsrb;
        ADMUX  =  s.admux;

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

  void updateDuty()
  {
    const uint16_t ucharge = getChargeVoltage();
    const uint16_t icharge = getChargeCurrent();

    /* compute new duty cycle value */
    uint8_t  duty     = _Duty;
    int8_t   tduty    = _TDuty;

    int8_t   dduty    = 0;

    uint16_t IChargeAvg = _IChargeAvg.getValue();

    if(getRecalibrate() || !getEnabled())
    {
      /* recalibaration of current and temperature measurement requested */
      duty  = 0;
      tduty = 0;
      _Hit = 2;

      clearBatteryLimit();
    }
    else if(_USolar < (_UBattery + 1000))
    {
      duty  = 0;
      tduty = 0;
      _Hit = 3;

      clearBatteryLimit();
    }
    else if(_USolar < (_UBattery + 1500))
    {
      dduty = -1;
      tduty = -1;
      _Hit = 4;

      clearBatteryLimit();
    }
    else if(_UBattery > ucharge)
    {
      dduty = -1;
      tduty =  0;
      _Hit = 5;

      incBatteryLimit();
    }
    else if(IChargeAvg > icharge &&
            _CyclesSinceDutyDec >= 256)
    {
      dduty = -1;
      tduty =  0;
      _Hit = 6;
    }
    else if (0 == duty)
    {
      dduty = 50;
      tduty =  0;
      _Hit = 7;
    }
    else if(_MPPTVoltage)
    { /* MPPT Voltage was specified by Solar module vendor
       * thus, don't do any tracking, just try to hold this input voltage */
      if(_USolar > _MPPTVoltage)
      {
        dduty = +1;
        _Hit = 8;
      }
      else
      {
        dduty = -1;
        decBatteryLimit();
        _Hit = 9;
      }

      tduty = 0;
    }
    else if (tduty > 0)
    {
      if(_CyclesSinceDutyInc > 1024)
      {
        if(IChargeAvg + 25 < _ITrack)
          tduty = -1;

        dduty = tduty;
        _Hit = 10;

        decBatteryLimit();
      }
    }
    else if (tduty < 0)
    {
      if(_CyclesSinceDutyDec > 1024)
      {
        if(IChargeAvg + 25 < _ITrack)
          tduty = 1;

        dduty = tduty;
        _Hit = 11;

        decBatteryLimit();
      }
    }
    else
    {
      tduty = 1;
      dduty = 1;
      _Hit = 12;

      decBatteryLimit();
    }

    if(tduty != _TDuty || 0 == tduty)
    {
      _TDuty  = tduty;
      _ITrack = IChargeAvg;
    }
    else if(IChargeAvg > _ITrack)
    {
      _ITrack = IChargeAvg;
    }

    if(dduty <= 0)
    {
      if(duty < (-dduty))
        duty = 0;
      else
        duty += dduty;
    }
    else
    {
      if ((dduty + duty) > param_maxpwmduty)
        duty = param_maxpwmduty;
      else
        duty += dduty;
    }


    _TDuty = tduty;
    _Duty  = duty;

  }


  void handleRegul()
  {
    uint8_t state = _AdcState;

    if(state == STATE_CONTROL_IDLE)
    {
      if(getStartCycle())
      {
        confirmStartCycle();

        bool recalibrate = getRecalibrate() || !getEnabled();

        if(recalibrate && _Duty == 0 &&
           _CyclesSinceDutyDec > 128)
        {
          confirmRecalibrate();
          state = STATE_ADC_CALIBRATEIOUTA;
        }
        else
        {
          state = STATE_ADC_SAMPLETEMP;
        }
      }
    }
    else if (state == STATE_CONTROL_CALC)
    {
      updateDuty();
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

      if(state > STATE_CONTROL_IDLE && state < STATE_CONTROL_CALC)
        enterADCState(state + 1);

      switch(state)
      {
      case STATE_ADC_SAMPLETEMP:
        {
          int16_t temp = val;

          temp += temp * EEData.TempCalibration[0] / 256;
          temp += EEData.TempCalibration[1];

          _Temp = temp;
          _TempTimer.init(_Clock30s, 30);
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

          int16_t  I = _IAvg[index].getSum() - _IOffset[index].getSum();

          if(I < 0)
            _Currents[index] = 0;
          else
            _Currents[index] = (static_cast<uint16_t>(I) * fact_mul + fact_div / 2) / fact_div;
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
        uint16_t I = _Currents[0] + _Currents[1];

        if(I < 0)
          I = 0;

        _ICharge = I;
        _IChargeAvg.addValue(I);

        _UBattery = (_Voltages[0] + _Voltages[1] + 1) / 2;
      }
    }


    if(ADCSRA &_BV(ADEN))
      Sys_AVR8::disableSleep();
  }

  void handleCan()
  {
    uint8_t state = _CanState;

    if (_CanTimer.hasTimedOut(_Clock1ms))
    {
      _CanTimer.updateTimeout(500);

      if(state >= 3)
        state = 0;
    }

    if(state == 0 && _Can.getMode() != _Can.MODE_NORMAL)
    {
      _Can.requestModeChange(_Can.MODE_NORMAL);
    }
    else if( state < 3)
    {
      if(!_Can.isTxPending(0))
      {
        const uint16_t sid = EEData.CanIds[state] | (_CanSid);
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

          len = 5;
        }
        else if(state == 2)
        {
          uint16_t usValue;

          buf[0]  = _Duty;

          usValue = _ITrack;

          buf[1]  = usValue >> 8;
          buf[2]  = usValue & 0xFF;

          usValue = _IChargeAvg.getValue();

          buf[3]  = usValue >> 8;
          buf[4]  = usValue & 0xFF;

          buf[5]  =  _BatLimitCnt;
          buf[6]  =  (_TDuty << 4) | (_Hit & 0xF);

          len = 7;
        }

        if(len)
        {
          _Can.transmitStandard(0, sid, len);
        }
        else
        {
        }
        state = state + 1;
      }
    }
    else if (state < 4)
    {
      if(!_Can.isTxPending(0))
      {
        //_Can.requestModeChange(_Can.MODE_SLEEP);
        state = state + 1;
      }
    }

    if(state < 4)
      Sys_AVR8::disableSleep();

    _CanState = state;
  }

  void handleLeds()
  {
    const uint16_t tmout = _LedTimer.hasTimedOut(_Clock1ms);

    if(tmout)
    {
      uint8_t status = 0;

      if(_BatState == BATTERY_STATE_FULL)
      {
        if(tmout < 50 || _USolar > _UBattery + 1500)
          status |= LED_GREEN;
      }
      else if (tmout < 50)
      {
        status |= LED_RED;
      }

      if (_ICharge > 50 && _BatState != BATTERY_STATE_FULL)
        status |= LED_RED;

      if(tmout >= 50)
      {
        _LedTimer.updateTimeout(1000);
      }

      setLedStatus(status);

      /* the following makes sure that the charger will be enabled
       * after a delay of 10 seconds
       * this is required to wait for the current offsets to be
       * calibrated */
      setEnabled();
    }
  }

  void cycle()
  {

    _Clock1ms.update(GPIOR0);

    uint8_t tmout = _AdcTimer.hasTimedOut(_Clock1ms);

    if(tmout)
    {
      _AdcTimer.updateTimeout(5);
      setStartCycle();
    }


    if(_HalfMinuteTimer.hasTimedOut(_Clock1ms))
    {
      _HalfMinuteTimer.updateTimeout(30000);
      _Clock30s.update(_Clock30s.value() + 1);
    }

    if(_TempTimer.hasTimedOut(_Clock30s))
      setRecalibrate();

    handleLeds();

    handleADCEvent();

    handleRegul();
    //handleSim(tmout);
    handleBuck(tmout);

    handleCan();

    //Sys_AVR8::disableSleep();
    Sys_AVR8::enterSleep();
    Sys_AVR8::enableSleep();
  }

  static void isrTick()
  {
    GPIOR0 = GPIOR0 + 1;
  }
};

ISR(TIMER0_COMPA_vect)
{
  App::isrTick();
  Sys_AVR8::disableSleep();
}

int main()
{
  static App app = App();

  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  uint8_t i;
  for(i = 0; i < ElementCount(EEData.RegisterInit); ++i)
  {
    RegisterConfig cfg = EEData.RegisterInit[i];
    cfg.writeRegister();
  }

  app.init();

  Sys_AVR8::enableInterrupts();

  while(1)
  {
    app.cycle();
  }
}



