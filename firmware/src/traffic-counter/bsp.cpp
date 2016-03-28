/*
 *  Copyright 2016 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of the radar based traffic counting device firmware.
 *
 *  The Radar based traffic counting device firmware is free software: you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation,
 *  either version 3 of the License, or (at your option) any later
 *  version.
 *
 *  Embedded C++ Platform Project is distributed in the hope that it
 *  will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ECPP.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of ECPP give you
 *  permission to link ECPP with independent modules to produce an
 *  executable, regardless of the license terms of these independent
 *  modules, and to copy and distribute the resulting executable under
 *  terms of your choice, provided that you also meet, for each linked
 *  independent module, the terms and conditions of the license of that
 *  module.  An independent module is a module which is not derived from
 *  or based on ECPP.  If you modify ECPP, you may extend this exception
 *  to your version of ECPP, but you are not obligated to do so.  If you
 *  do not wish to do so, delete this exception statement from your
 *  version.
 *  */
#include <ecpp/Target.hpp>
#include <ecpp/Peripherals/TextLCD_HD44780_KS6600U.hpp>

#include "bsp.hpp"
#include "app.hpp"
#include "freqcnt.hpp"

using namespace ecpp;
using namespace ecpp::Peripherals;

static constexpr uint8_t s_LevelKeyNext =  3;
static constexpr uint8_t s_LevelKeyOK   =  9;

static IOPort<AVR_IO_PB> s_PortB;
static IOPort<AVR_IO_PC> s_PortC;
static IOPort<AVR_IO_PD> s_PortD;

static IOPin<AVR_IO_PD0> s_PinLcdE;
static IOPin<AVR_IO_PD1> s_PinLcdRW;
static IOPin<AVR_IO_PD2> s_PinLcdRS;

static IOPin<AVR_IO_PD6> s_PinSDCardPower;
static IOPin<AVR_IO_PD7> s_PinRadarShutdown;

/* input pin for one of the radar signal
 * this pin is used to compute the direction */
static IOPin<AVR_IO_PD3> s_PinRadarI;

/* input pin for the other one of the rdar signal
 * this pin is used to calculate speed */
static IOPin<AVR_IO_PD3> s_PinRadarQ;


static IOPin<AVR_IO_PB1> s_PinLed;
static IOPin<AVR_IO_PB2> s_PinSDCardCS;
static IOPin<AVR_IO_PB3> s_PinSDCardSI;
static IOPin<AVR_IO_PB4> s_PinSDCardSO;
static IOPin<AVR_IO_PB5> s_PinSDCardCLK;

static ReciprocalCounter s_FreqCounter;
static volatile uint16_t s_StartTick1ms;
static volatile uint16_t s_StopTick1ms;
static uint8_t  s_DetectCnt;
static volatile uint16_t s_Tick1ms    = 0;
static uint16_t          s_TickTime1ms   = 0;
static uint8_t           s_TickFat1ms = 0;

class LcdBsp
{
private:
  void setRS()   __attribute__((always_inline)) {s_PinLcdRS = 1;}
  void clearRS() __attribute__((always_inline)) {s_PinLcdRS = 0;}

  void setEnable()   __attribute__((always_inline)) {s_PinLcdE = 1;}
  void clearEnable() __attribute__((always_inline)) {s_PinLcdE = 0;}

  void setWriteMode()
  {
    s_PinLcdRW = 0;
    s_PortC.updateDirection(0x0F,0x0F);
  }

  void setReadMode()
  {
    s_PinLcdRW = 1;
    s_PortC.updateDirection(0x00,0x0F);
  }

  void setNibble(uint8_t data)
  {
    s_PortC.updateOutputs(data,0x0F);
  }

  uint8_t getNibble()
  {
    asm volatile ("nop");
    asm volatile ("nop");
    return s_PortC.getInputs() & 0x0F;
  }

protected:
  void delayMicroseconds(uint16_t delay);

  uint8_t readByte();
  void    writeByte(uint8_t data, bool nibble = false);

  void writeCommandNibble(uint8_t cmd);

  void writeCommand(uint8_t cmd);
  void writeRam(uint8_t data);

public:
  uint8_t isBusy();
};

void LcdBsp::delayMicroseconds(uint16_t delay)
{
  _delay_loop_2(delay * 2);
}

uint8_t LcdBsp::readByte()
{
  uint8_t data;

  setReadMode();
  setEnable();
  data = getNibble() << 4;
  clearEnable();

  getNibble();

  setEnable();
  data |= getNibble();
  clearEnable();

  return data;
}

void LcdBsp::writeByte(uint8_t data, bool nibble)
{
  setWriteMode();
  setEnable();
  setNibble(data >> 4);
  clearEnable();

  setNibble(data >> 4);

  if (!nibble)
  {
    setEnable();
    setNibble(data & 0x0F);
    clearEnable();
    setNibble(data & 0x0F);
  }
}

void LcdBsp::writeCommandNibble(uint8_t cmd)
{
  clearRS();
  writeByte(cmd, true);
}

uint8_t LcdBsp::isBusy()
{
  clearRS();
  return (0x80 & readByte());
}

void LcdBsp::writeCommand(uint8_t cmd)
{
  while(isBusy());

  clearRS();
  writeByte(cmd);
}

void LcdBsp::writeRam(uint8_t data)
{
  while(isBusy());

  setRS();
  writeByte(data);
}

uint16_t
Bsp::poll(void)
{
  uint16_t Milliseconds;

  { /* compute the number of milliseconds passed since last call */
    uint16_t Tick1ms;

    do { /* Safely read tick variable generated
          * in irq */
      Tick1ms = s_Tick1ms;
    } while(Tick1ms != s_Tick1ms);

    Milliseconds = Tick1ms - s_TickTime1ms;
    s_TickTime1ms = Tick1ms;


    { /* run the filesystem timer */
      uint8_t Delta = (uint8_t)(Tick1ms) - s_TickFat1ms;

      if(Delta >= 10)
      {
        s_TickFat1ms += 10;
        disk_timerproc();
      }
    }
  }

  {
    uint8_t keymask;

    m_KeyTimer.handleMillisecondsPassed(Milliseconds);

    if(ADCSRA & _BV(ADIF))
    {
      const uint16_t counts  = ADC;
      uint16_t voltage;

      voltage = (25 * counts) / 512;

      if (voltage < s_LevelKeyNext)
      {
        keymask = KEY_NEXT;
      }
      else if (voltage < s_LevelKeyOK)
      {
        keymask = KEY_OK;
      }
      else
      {
        m_BatteryVoltage = voltage * (100 + 47) / 47;
        keymask = 0;
      }


      m_KeyDebouncer.poll(keymask, m_KeyTimer);
    }
  }

  { /* handle clock ticks */
    uint16_t ClockTicks1ms;

    /* lets hope there is no overflow, but in that case
     * we have a real problem */
    ClockTicks1ms = Milliseconds + m_ClockTicks1ms;

    if(ClockTicks1ms < m_ClockTicks1ms)
    { /* overflow */
      /* TODO */;
    }

    if(ClockTicks1ms >= 1000)
    {
      m_ClockTicks1ms = ClockTicks1ms - 1000;
      m_Clock.tick();
    }
    else
    {
      m_ClockTicks1ms = ClockTicks1ms;
    }
  }

  return Milliseconds;
}

static TextLCD_HD44780_KS0066U<KS0066U_MODE_4BIT<LcdBsp, 16,2> > s_Lcd __attribute__((section(".bss")));

void
Bsp::init()
{
  s_PortB = s_PinSDCardCLK.OutHigh | s_PinSDCardSO.InPullUp | s_PinSDCardSI.OutHigh | s_PinSDCardCS.OutHigh | s_PinLed.OutHigh;
  s_PortD = s_PinRadarShutdown.In  | s_PinSDCardPower.In    | s_PinLcdE.OutLow | s_PinLcdRS.OutLow | s_PinLcdRW.OutLow;

  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  /* Timer 0 counts the signal periods. We count rising edges of the signal
   * It shall generate an compare event if at least 10 periods have been count
   * */
  OCR0A  =  9;

  /* Timer/Counter 1 is to count the 1 Mhz reference count
   * Input capture is set to falling edge */
  TCCR1B = _BV(CS11);

  /* Timer/Count 2 generates a 2ms tick for timing purposes */
  OCR2A  = 125 - 1;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS20);

  TIMSK2 = _BV(OCIE2A);

  /* ADC continuously samples ADC5 to monitor
   * battery and keys */
  ADMUX  = _BV(REFS0) | 5;
  ADCSRA = _BV(ADEN)  | _BV(ADSC) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  s_Lcd.init();
}





void StartMeas()
{
  /* disable timer interrupts */
  TIMSK1 = 0;

  s_FreqCounter.nextMeasurement();

  /* Set Timer 0 to count rising edges of clock
   * signal */
  TCCR0B = _BV(CS02) | _BV(CS01) | _BV(CS00);

  /* Clear Timer 1 interrupts
   * and enable Timer 1 capture interrupt */
  OCR1B  = TCNT1 - 1;

  /* clear pending interrupts and enable interupt mask */
  TIFR1  = 0xFF;
  TIMSK1 = _BV(OCF1B) | _BV(ICIE1);
}

static void processMeasurement()
{
  auto & Detector = g_Globals.Detector;
  auto & Recorder = g_Globals.Recorder;

  if(s_FreqCounter.getState() == s_FreqCounter.STATE_FINISH)
  {
    const uint32_t f_div       = 8;
    const uint32_t f_ref       = (F_CPU + f_div/2) / f_div;
    const uint8_t  speed_per_f = 22;
    uint32_t speed;
    uint32_t n_ref;
    uint8_t  n_sig;

    /* Get signal & reference counts */
    n_sig = s_FreqCounter.getSignalCnt();
    n_ref = s_FreqCounter.getReferenceCnt();

    /* scale reference count with speed factor */
    n_ref = n_ref * speed_per_f;

    speed = (n_sig * f_ref + n_ref/2) / (n_ref);

    Detector.sampleMeasurement(speed);

    /* start another measurement */
    StartMeas();
  }
  else if(s_FreqCounter.getState() == s_FreqCounter.STATE_TIMEOUT)
  {
    uint16_t Duration;
    int8_t phasecnt;

    phasecnt = s_FreqCounter.getPhaseCnt();
    Duration = s_StopTick1ms - s_StartTick1ms;

    Detector.timeout(phasecnt);

    if(Detector.getState() == Detector.STATE_DETECTED)
    {
      auto DetectCnt = s_DetectCnt + 1;

      s_DetectCnt    = DetectCnt;

      auto & Event = Recorder.createTrafficRecord();
      enum TrafficRecord::Direction Direction;

      if(Detector.getDirection() < 0)
      {
        Direction = Event.LEFT;
      }
      else if(Detector.getDirection() > 0)
      {
        Direction = Event.RIGHT;
      }
      else
      {
        Direction = Event.UNKOWN;
      }

      Event.setTime(Bsp::getInstance().getClock());
      Event.setSpeed(Detector.getSpeed(), Direction, Duration);

      Recorder.recordEvent(Event);
    }

    s_FreqCounter.init();
    Detector.init();

    StartMeas();
  }
}

ISR(TIMER1_CAPT_vect)
{
  decltype(s_FreqCounter.getState()) state;
  uint16_t Ticks;
  uint16_t cnt;
  bool     phase;

  /* Get current input capture value */
  cnt   = ICR1;
  phase = s_PinRadarI;

  s_FreqCounter.countPhase(phase);

  state = s_FreqCounter.getState();

  Ticks         = s_Tick1ms;
  s_StopTick1ms = Ticks;

  /* initial time measurement */
  if(state == s_FreqCounter.STATE_WSTART)
  {
    auto & Detector = g_Globals.Detector;

    /* Reset Timer 0 and clear compare interrupt flag
     */
    TCNT0      =  0;
    TIFR0     |= _BV(OCF0A);

    /* Compare 1 A flag is set when 1000 reference ticks have been counted */
    OCR1A      = cnt + 1000;

    /* Compare 1 B flag shall be set if Timer 1 overflows */
    OCR1B      = cnt - 1;

    /* clear pending interrupts and enable interupt mask */
    TIFR1      = _BV(OCF1A) | _BV(OCF1B);
    TIMSK1    |= _BV(OCF1B);

    s_FreqCounter.startCounting(cnt);

    if(Detector.STATE_W_ENTER == Detector.getState())
    {
      s_StartTick1ms = Ticks;
    }
  }
  else if(state == s_FreqCounter.STATE_MEASURING)
  {
    if ((TIFR0 & _BV(OCF0A)) &&
        (TIFR1 & _BV(OCF1A)))
    {
      /* Stop Timer 0 */
      TCCR0B = 0;

      /* Disable timer 1 interrupts */
      TIMSK1 = 0;

      if(TIFR1 & _BV(OCF1B))
      {
        s_FreqCounter.timeout();
      }
      else
      {
        s_FreqCounter.stopCounting(cnt, TCNT0);
      }
    }
  }
}

ISR(TIMER1_COMPB_vect)
{
  /* Stop Timer 0 */
  TCCR0B = 0;

  /* Disable timer 1 interrupts */
  TIMSK1 = 0;

  s_FreqCounter.timeout();
}


ISR(TIMER2_COMPA_vect)
{
  s_Tick1ms += 2;

  /* every 2 ms check if a new timing was measured and process it */
  processMeasurement();
}

void
Bsp::enableRadar()
{
  s_PinRadarShutdown.enableOutput();

  g_Globals.Detector.init();
  StartMeas();

}

void
Bsp::disableRadar()
{
  s_PinRadarShutdown.disableOutput();
}

void Bsp::updateFrameBuffer(Ui::FramebufferType& FrameBuffer)
{
  FrameBuffer.update(s_Lcd);
}

Bsp Bsp::s_Instance;
