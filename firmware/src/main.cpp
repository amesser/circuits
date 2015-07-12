/*
 * main.cpp
 *
 *  Created on: 22.04.2015
 *      Author: andi
 */

#include "ecpp/Target.hpp"
#include "ecpp/Byteorder.hpp"
#include "ecpp/Time.hpp"

#include "protocol.hpp"
#include "voltage-modulator.hpp"

using namespace ecpp;

static IOPin<AVR_IO_PB0> RefCapacitor;
static IOPin<AVR_IO_PB1> HumCapacitor;

static IOPin<AVR_IO_PB2> LedCathode;
static IOPin<AVR_IO_PB3> LedAnode;

static IOPin<AVR_IO_PB4> Temp;
static IOPin<AVR_IO_PB5> Reset;

static IOPort<AVR_IO_PB> Port;


class BoardSupportPackage
{
public:
  enum {
    ADC_CHANNEL_VCC  = _BV(ADLAR) | _BV(MUX1),
    ADC_CHANNEL_TEMP = _BV(MUX1),

    AC_CHANNEL_HUM   = 0,
  };

  static void      enableADC     (uint8_t channel);
  static void      enableAC      (uint8_t channel);
  static void      disableAnalog ();

  static uint8_t   getVcc()     {return ADCH;}
  static uint16_t  getTemp()    {return ADC;}
  static bool      getAcState() {return ACSR & _BV(ACO);}

  enum {
    TIMER_PRESCALE_8  = _BV(CS01),
    TIMER_PRESCALE_64 = _BV(CS01) | _BV(CS00),
  };
  typedef SimpleTimer<uint16_t> TimerType;

  static void enableTimer0(uint8_t prescaler, uint8_t max);

  static TimerType startTimerMs(uint16_t timeout)
  {
    enableTimer0(TIMER_PRESCALE_8, 150);
    return TimerType((uint64_t)F_CPU * timeout / 8 / 150 / 1000);
  }

  static bool      handleTimer();
  static bool      handleTimer(TimerType & timer);

  static void      waitTimer();

  static void      setPortState(const IOPortState & state)
  {
    Port.setState(state);
  }

};


static BoardSupportPackage bsp;

void BoardSupportPackage::enableTimer0(uint8_t prescale, uint8_t max)
{
  uint8_t resetmask = _BV(PSR10) | _BV(TOV0);

  /* configure timer */
  OCR0A  = max;
  TCCR0B = prescale;
  TCCR0A = _BV(WGM01);

  /* reset prescaler, timer and tov */
  GTCCR  = resetmask;
  TCNT0  = 0;
  TIFR0  = resetmask;
}

/* start ADC conversion, ADC will be enabled in free running mode */
void BoardSupportPackage::enableADC(uint8_t channel)
{
  ADMUX  = channel;
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADPS1) | _BV(ADPS0);

  /* wait for first measurement */
  while(!(ADCSRA & _BV(ADIF)));
}

void BoardSupportPackage::enableAC(uint8_t channel)
{
  ADMUX  = channel;
  ADCSRA = 0;
  ADCSRB = _BV(ACME);
  ACSR   = 0;
}

void
BoardSupportPackage::disableAnalog()
{
  ADMUX  = 0;
  ADCSRA = 0;
  ADCSRB = 0;
  ACSR   = 0;
}

bool
BoardSupportPackage::handleTimer()
{
  uint8_t mask;

  mask  = _BV(TOV0) & TIFR0;
  TIFR0 = mask;

  return mask != 0;
}

bool
BoardSupportPackage::handleTimer(TimerType & timer)
{
  if (handleTimer())
  {
    timer.update(1);
  }

  return timer.hasTimedOut();
}

void
BoardSupportPackage::waitTimer()
{
  while(!handleTimer());
}


class VoltageModulatorSupport
{
public:
  enum
  {
    SUPPLY_5V = 2,
    SUPPLY_4V = 1,
    SUPPLY_3V = 0,
  };

  static BoardSupportPackage::TimerType startTimer(uint16_t Timeout1Ms)
  {
    return bsp.startTimerMs(Timeout1Ms);
  }

  static bool checkTimeout(BoardSupportPackage::TimerType & timer)
  {
    return bsp.handleTimer(timer);
  }

  static uint8_t getState();
};


uint8_t
VoltageModulatorSupport::getState()
{
  /* check if we have 3, 4 or 5 V */
  const uint8_t limit_a = 256. * 0.55 / (3. + 0.666);
  const uint8_t limit_b = 256. * 0.55 / (4. + 0.333);

  uint8_t readout = bsp.getVcc();
  uint8_t  state  = SUPPLY_5V;

  if(limit_b < readout)
  { /* < 4.3 V */
    state--;
  }

  if (limit_a < readout)
  { /* < 3.6 V */
    state--;
  }

  return state;
}


class Evaluator
{
private:
  uint32_t m_Mult;
  uint16_t m_Offset;
  uint16_t m_Max;
public:
  Evaluator() {};
  constexpr Evaluator(uint32_t Mult, uint16_t Offset, uint16_t Max) : m_Mult(Mult), m_Offset(Offset), m_Max(Max) {};

  uint16_t scale(uint16_t value) const;
};

static uint32_t mult32x16 (uint32_t lhs, uint16_t rhs) __attribute__((always_inline));
static uint32_t mult32x16 (uint32_t lhs, uint16_t rhs)
{
  uint32_t result = 0;

  while(rhs)
  {
    if(rhs & 0x0001)
    {
      result += lhs;
    }

    rhs >>= 1;
    lhs <<= 1;
  }

  return result;
}

uint16_t Evaluator::scale(uint16_t value) const
{
  value = m_Offset + value;

  if(value <= m_Max)
  {
    uint32_t result = mult32x16(m_Mult, value);
    value = result >> 16U;
  }
  else
  {
    value = 0xFFFF;
  }

  return value;
}

struct Evaluators
{
  Evaluator Humidity;
  Evaluator Light;
  Evaluator Temperature;

  Evaluators () {}
  constexpr Evaluators (const Evaluator& hum, const Evaluator& light, const Evaluator& temp)
    : Humidity(hum), Light(light), Temperature(temp) {}
};


EEVariable<struct Evaluators> calibration_parameters EEMEM =
{{
  {0x10000UL, 0x0000, 0xFFFF},
  {0x10000UL, 0x0000, 0xFFFF},
  {0x10000UL, 0x0000, 0xFFFF}
}} ;

struct Evaluators calibrators __attribute__((section(".noinit")));
struct measurement_data data __attribute__((section(".noinit")));


#define USE_AC 1

void init(void) __attribute__((noreturn, naked, section (".vectors")));
void init(void)
{

  asm volatile ( "ldi r18, %0 \n"
                 "out %1, r18 \n"
                 "clr __zero_reg__ \n"
                 "rjmp main \n"
   :: "i" (RAMEND), "i" (_SFR_IO_ADDR(SPL)));
}

int main(void) __attribute__ ((OS_main));
int main(void)
{
  { /* Step 1: Check operating voltage */

    /* forward bias temperature diode */
    bsp.setPortState(RefCapacitor.OutLow | HumCapacitor.OutLow | LedCathode.OutHigh | LedAnode.OutHigh);
    bsp.enableADC(bsp.ADC_CHANNEL_VCC);

    auto    timer = bsp.startTimerMs(5000);
    uint8_t state = VoltageModulatorSupport::SUPPLY_5V;

    while(!bsp.handleTimer(timer))
    {
      state = VoltageModulatorSupport::getState();

      if(state != VoltageModulatorSupport::SUPPLY_3V)
        break;
    }

    if(state == VoltageModulatorSupport::SUPPLY_3V)
    { /* supply voltage stayed at 3 volts for about 5 seconds
       * calibration parameterization requested */

      VoltageModulator<80,20,VoltageModulatorSupport> modulator;

      uint8_t len_received = modulator.receive(&calibrators, static_cast<uint8_t>(sizeof(calibrators)));

      if(len_received == sizeof(calibrators))
      {
        eeprom_update_block(&calibrators, &calibration_parameters, sizeof(calibration_parameters));
      }
    }
  }

  /* Step 2: measure humidity */
  {
    uint16_t count;

#ifdef USE_AC
    bsp.enableAC(bsp.AC_CHANNEL_HUM);
#else
    bsp.disableAnalog();
#endif
    bsp.setPortState(HumCapacitor.OutLow | RefCapacitor.OutLow | LedCathode.OutLow | LedAnode.OutLow | Temp.OutLow);

    for (count = 0xFFFF; count > 0; --count)
    {
      /* measure voltage and discharge
       * hum capacitor */
      HumCapacitor.enableOutput();
      asm ( "nop");

      /* check if ref capacitor is charged */
#ifdef USE_AC
      asm ("nop");
      if(bsp.getAcState())
        break;
#else
      if(RefCapacitor.getInput())
        break;
#endif

      HumCapacitor.disableOutput();

      /* transfer charge */
      RefCapacitor.setOutput();
      RefCapacitor.enableOutput();

      RefCapacitor.disableOutput();
      RefCapacitor.clearOutput();
    }

    data.humidity_counts = count;

#ifdef USE_AC
    bsp.disableAnalog();
#endif
  }

  /* Step 3: Measure Light */
  {
    uint16_t count;

    /* charge led */
    bsp.setPortState(HumCapacitor.OutLow | RefCapacitor.OutLow | LedCathode.OutHigh | LedAnode.OutLow | Temp.OutLow);

    bsp.enableTimer0(bsp.TIMER_PRESCALE_8, F_CPU / 8 * 1 / 1000);
    bsp.waitTimer();

    /* discharge capacitor via led and count time */
    LedCathode.disableOutput();
    LedCathode.clearOutput();

    for (count = 0xFFFF; count > 0; --count)
    {
      bsp.waitTimer();

      if (!LedCathode.getInput())
        break;
    }

    data.led_counts = count;
  }

  /* Step 4: Measure Temperature */
  {
    bsp.setPortState(HumCapacitor.OutLow | RefCapacitor.OutLow | LedCathode.OutLow | LedAnode.OutHigh);
    bsp.enableADC(bsp.ADC_CHANNEL_TEMP);

    auto    timer = bsp.startTimerMs(1000);
    while(!bsp.handleTimer(timer));

    data.temp = - bsp.getTemp();

    bsp.disableAnalog();
  }

  calibrators = calibration_parameters;

  /* Step 5: Perform calibration */
  {
    data.sync = 0xC0;
    data.type = 0;

    Evaluator * pe = &(calibrators.Humidity);
    uint16_t  * pd = &(data.humidity_counts);

    for(uint_fast8_t cnt = 3; cnt > 0; --cnt)
    {
      *pd = hton16(pe->scale(*pd));
      ++pd; ++pe;
    }
  }

  /* RS232 */
  bsp.setPortState(HumCapacitor.OutLow | RefCapacitor.OutLow | LedCathode.OutLow | LedAnode.OutHigh | Temp.OutLow);

  /* uart */
  while(1)
  {
    const uint8_t *p = reinterpret_cast<const uint8_t*>(&(data));

    auto    timer = bsp.startTimerMs(1000);
    while(!bsp.handleTimer(timer));

    /* baud rate is 1200, 9 databits odd parity, 2 stopbits */
    bsp.enableTimer0(bsp.TIMER_PRESCALE_8, F_CPU / 8  / 1200);

    enum {
      SRT_START  = 0,
      SRT_DATA   = 1,
      SRT_PARITY = 1 + 9,
      SRT_STOP   = 1 + 9 + 1
    };

    for(uint_fast8_t bytes = sizeof(data); bytes > 0; --bytes)
    {
      /* to avoid unnecessary bit shifts, the whole data
       * will be shift by one */

      uint_fast16_t val     = (0x7 << (SRT_STOP - 1)) | *(p++);
      val <<= 1; /* shift by one */

      uint_fast16_t parmask = (0x1 << SRT_PARITY);

      while(val)
      {
        if(val & 0x01)
        {
          LedAnode.setOutput();
          val ^= parmask;
        }
        else
        {
          LedAnode.clearOutput();
        }

        val     >>= 1;
        parmask >>= 1;

        bsp.waitTimer();
      }
    }
  }

}
