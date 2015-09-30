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
    ADC_CHANNEL_VCC  = _BV(MUX1),
    ADC_CHANNEL_TEMP = _BV(REFS0) | _BV(MUX1),

    AC_CHANNEL_HUM   = 0,
  };

  static void      enableADC     (uint8_t channel);
  static void      enableAC      (uint8_t channel);
  static void      disableAnalog ();

  static uint8_t   getVcc()     {return ADC;}
  static uint16_t  getTemp()    {return ADC;}
  static bool      getAcState() {return ACSR & _BV(ACO);}

  enum {
    TIMER_PRESCALE_8  = _BV(CS01),
    TIMER_PRESCALE_64 = _BV(CS01) | _BV(CS00),
  };
  typedef SimpleTimer<uint16_t, 1> TimerType;

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
  /* configure timer */
  OCR0A  = max - 1;
  TCCR0B = prescale;
  TCCR0A = _BV(WGM01);

  /* reset prescaler, timer and tov */
  GTCCR  = _BV(PSR10);
  TCNT0  = 0;
  TIFR0  = _BV(OCF0A);
}

bool
BoardSupportPackage::handleTimer()
{
  uint8_t mask;

  mask  = _BV(OCF0A) & TIFR0;
  TIFR0 = mask;

  return mask != 0;
}

bool
BoardSupportPackage::handleTimer(TimerType & timer)
{
  if (handleTimer())
  {
    timer.handleTicksPassed(1);
  }

  return timer.hasTimedOut();
}

void
BoardSupportPackage::waitTimer()
{
  while(!handleTimer());
}

/* start ADC conversion, ADC will be enabled in free running mode */
void BoardSupportPackage::enableADC(uint8_t channel)
{
  ADMUX  = channel;
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | _BV(ADPS1) | _BV(ADPS0);

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
  ACSR   = _BV(ACD);
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
  const uint8_t limit_a = 1024. * 0.7 / (3. + 0.666);
  const uint8_t limit_b = 1024. * 0.7 / (4. + 0.333);

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


const FlashVariable<struct calibration_param> calibration_default PROGMEM =
{
  {0x10000UL, 0x0000, 0xFFFF},
};

EEVariable<struct calibration_data> calibration_parameters EEMEM =
{{
  {0x10000UL, 0x0000, 0xFFFF},
  {0x10000UL, 0x0000, 0xFFFF},
  {0x10000UL, 0x0000, 0xFFFF}
}} ;

struct calibration_data calibrators __attribute__((section(".noinit")));
struct measurement_data data        __attribute__((section(".noinit")));


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

    auto    timer            = bsp.startTimerMs(1000);
    uint8_t LastVoltageState = 4;

    while(!bsp.handleTimer(timer))
    {
      uint8_t ActualVoltageState = VoltageModulatorSupport::getState();

      if (LastVoltageState != ActualVoltageState)
      {
        if(LastVoltageState == VoltageModulatorSupport::SUPPLY_3V ||
           ActualVoltageState == VoltageModulatorSupport::SUPPLY_3V)
        {
          bsp.startTimerMs(1000);
        }

        LastVoltageState = ActualVoltageState;
      }
    }

    if(LastVoltageState == VoltageModulatorSupport::SUPPLY_3V)
    { /* supply voltage stayed at 3 volts for about 1 seconds
       * calibration parameterization requested */

      typedef VoltageModulator<sizeof(calibrators), 80,20,VoltageModulatorSupport> VoltageModulatorType;

      uint8_t len_received = VoltageModulatorType::receive(&calibrators);

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

  calibration_parameters.read(calibrators);

  /* Step 5: Perform calibration */
  {
    data.sync = 0xC0;
    data.type = 0;

    Evaluator * pe = reinterpret_cast<Evaluator*>(&(calibrators.Humidity));
    uint16_t  * pd = &(data.humidity_counts);

    for(uint_fast8_t cnt = 3; cnt > 0; --cnt)
    {
      if (calibration_default != *pe)
      {
        data.type = 1;
      }

      *pd = hton16(pe->scale(*pd));
      //*pd = hton16(*pd);
      ++pd; ++pe;
    }
  }

  /* RS232 */
  bsp.setPortState(HumCapacitor.OutLow | RefCapacitor.OutLow | LedCathode.OutLow | LedAnode.OutHigh);

  /* uart */
  while(1)
  {
    const uint8_t *p = reinterpret_cast<const uint8_t*>(&(data));

    auto    timer = bsp.startTimerMs(250);
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
          LedCathode.clearOutput();
          val ^= parmask;
        }
        else
        {
          LedCathode.setOutput();
        }

        val     >>= 1;
        parmask >>= 1;

        bsp.waitTimer();
      }
    }
  }

}
