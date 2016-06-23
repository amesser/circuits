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

static IOPin<AVR_IO_PD3> s_PinSDCardPower;
static IOPin<AVR_IO_PD7> s_PinRadarShutdown;

/* input pin for one of the radar signal
 * this pin is used to compute the direction */
static IOPin<AVR_IO_PB1> s_PinRadarI;

/* input pin for the other one of the rdar signal
 * this pin is used to calculate speed */
static IOPin<AVR_IO_PD3> s_PinRadarQ;


static IOPin<AVR_IO_PD5> s_PinLed1;
static IOPin<AVR_IO_PD6> s_PinLed2;

static IOPin<AVR_IO_PB2> s_PinSDCardCS;
static IOPin<AVR_IO_PB3> s_PinSDCardSI;
static IOPin<AVR_IO_PB4> s_PinSDCardSO;
static IOPin<AVR_IO_PB5> s_PinSDCardCLK;

//static ReciprocalCounter s_FreqCounter;
static volatile uint16_t s_StartTick256Hz;
static volatile uint16_t s_StopTick256Hz;
static uint8_t  s_DetectCnt;

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

uint8_t
Bsp::poll(void)
{
  uint8_t TicksPassed256Hz;

  { /* compute the number of milliseconds passed since last call */
    uint8_t Ticks256Hz = TCNT2;
    uint8_t Calc;

    Calc = Ticks256Hz - m_TicksFat256Hz;

    if(Calc >= 3)
    {
      m_TicksFat256Hz = Ticks256Hz;
      disk_timerproc();
    }

    TicksPassed256Hz = Ticks256Hz - m_TicksHandled256Hz;
    m_TicksHandled256Hz = Ticks256Hz;
  }

  {
    uint8_t keymask;

    m_KeyTimer.handleTicksPassed(TicksPassed256Hz);

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


  if(Ticks1s != Ticks1sHandled)
  { /* Handle clock if necessary */
    Ticks1sHandled += 1;
    m_Clock.tick();
  }

  return TicksPassed256Hz;
}

static TextLCD_HD44780_KS0066U<KS0066U_MODE_4BIT<LcdBsp, 16,2> > s_Lcd __attribute__((section(".bss")));

void
Bsp::init()
{
  s_PortB = s_PinSDCardCLK.OutHigh | s_PinSDCardSO.InPullUp | s_PinSDCardSI.OutHigh | s_PinSDCardCS.OutHigh;
  s_PortD = s_PinRadarShutdown.In  | s_PinSDCardPower.OutLow    | s_PinLcdE.OutLow | s_PinLcdRS.OutLow | s_PinLcdRW.OutLow | s_PinLed1.OutLow | s_PinLed2.OutHigh;

  /* switch to 8mhz system clock */
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  ASSR   = _BV(AS2);

  /* Timer/Counter 2 runs on clock Crystal It overflows once per Second
   * It runs in asnyc mode, therefore setup it first */
  TCNT2  = 0;
  TCCR2A = 0;
  TCCR2B = _BV(CS22) | _BV(CS20);

  /* Timer 0 counts the signal periods. We count rising edges of the signal */
  TCCR0B = _BV(CS02) | _BV(CS01) | _BV(CS00);

  /* Timer/Counter 1 is to count the 1 Mhz reference count
   * Input capture is set to falling edge */
  TCCR1B = _BV(CS11);

  /* ADC continuously samples ADC5 to monitor
   * battery and keys */
  ADMUX  = _BV(REFS0) | 6;
  ADCSRA = _BV(ADEN)  | _BV(ADSC) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  /* wait for TCCR2 to become ready */
  while(ASSR & (_BV(TCN2UB) | _BV(OCR2AUB) | _BV(OCR2BUB) | _BV(TCR2AUB) | _BV(TCR2BUB)));

  OCR2A  = 0;
  TIFR2  = _BV(TOV2)  | _BV(OCF2A);

  /* setup interrupts */
  TIMSK1 = _BV(OCIE1B) | _BV(ICIE1);
  TIMSK2 = _BV(OCIE2A);

  s_Lcd.init();
}

#if 0
static void processMeasurement()
{
  auto & Detector = g_Globals.Detector;
  auto & Recorder = g_Globals.Recorder;

  if(s_FreqCounter.getState() == s_FreqCounter.STATE_FINISH)
  {
#if !defined(DIAGFIRMWARE)
    const uint32_t f_ref       = 1000000;
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

#else
    auto * record = Recorder.createRecord();

    if(record)
    {
      record->Ticks1s  = Bsp::getInstance().getSecond();
      record->Ticks256Hz = TCNT2;
      record->NSig    = s_FreqCounter.getSignalCnt();
      record->NPhase  = s_FreqCounter.getPhaseCnt();
      record->NRef      = s_FreqCounter.getReferenceCnt();

      Recorder.storeRecord(record);
    }

    s_FreqCounter.init();

#endif

    /* start another measurement */
    StartMeas();
  }
  else if(s_FreqCounter.getState() == s_FreqCounter.STATE_TIMEOUT)
  {
#if !defined(DIAGFIRMWARE)
    uint16_t Duration;
    int8_t phasecnt;

    phasecnt = s_FreqCounter.getPhaseCnt();
    Duration = s_StopTick256Hz - s_StartTick256Hz;

    Detector.timeout(phasecnt);

    if(Detector.getState() == Detector.STATE_DETECTED)
    {
      auto DetectCnt = s_DetectCnt + 1;

      s_DetectCnt    = DetectCnt;


      auto record = Recorder.createTrafficRecord();
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
#endif

    s_FreqCounter.init();
    Detector.init();

    StartMeas();
  }
}

ISR(TIMER1_OVF_vect)
{
  s_PinLed1.toggleOutput();
}
#endif

ISR(TIMER1_CAPT_vect)
{
  Bsp &bsp = Bsp::getInstance();

  uint16_t refcnt;
  uint8_t  sigcnt;

  uint8_t  tcnt0;
  uint16_t icr1;
  bool     phase;

  uint8_t action;


  tcnt0 = TCNT0;
  icr1  = ICR1;

  phase = s_PinRadarI;

  if(s_PinRadarI)
  {
    if(bsp.m_FreqPhaseCnt < 20)
    {
      bsp.m_FreqPhaseCnt += 1;
    }
  }
  else
  {
    if(bsp.m_FreqPhaseCnt > -20)
    {
      bsp.m_FreqPhaseCnt -= 1;
    }
  }

  if(bsp.m_FreqCntState == bsp.FREQCNT_IDLE)
  {
    bsp.m_FreqCntState = bsp.FREQCNT_RUN;
    action = 1;
  }
  else
  {
    /* check for signal counter overflow, that means > 256 khz
     * we can not evaluate the signal then  */
    if(TIFR0 & _BV(OCF0A))
    {
      action = 1;
    }
    else if (1)
    { /* enough counts have been done */
      refcnt = icr1  - bsp.m_FreqICR1Start;
      sigcnt = tcnt0 - bsp.m_FreqTCNT0Start;

      action = 2;
    }
    else
    {
      action = 0;
    }
  }

  if(action)
  {
    bsp.m_FreqTCNT0Start = tcnt0;
    bsp.m_FreqICR1Start  = icr1;

    /* at least 4000 counts shall be done.  */
    OCR1A      = icr1 + 4000;
    /* timeout after 65536 counts */
    OCR1B      = icr1 - 1;

    /* disable input capture interrupt now. No need to call this vector
     * until at least OCR1A has been hit.
     *
     * Interrupt will be reenabled by TIMER1_COMPA_vect */
    TIFR1  = _BV(OCF1A)  | _BV(OCF1B);
    TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);

    /* check for overflows in signal count */
    OCR0A = tcnt0 - 1;
    TIFR0 = _BV(OCF0A);
  }

  if(action == 2)
  { /* a successfull measurement was done */
    uint8_t  ticks1s;
    uint8_t  ticks256Hz;
    bool ov;

    /* we need to read TCNT2 and TIFR2 consistently for timing information
     * */
    do
    {
      ticks256Hz = TCNT2;
      ov = TIFR2 & _BV(TOV2);
    } while( ticks256Hz != TCNT2);

    if (ov)
    {
      ticks1s = bsp.Ticks1s + 1;
    }
    else
    {
      ticks1s = bsp.Ticks1s;
    }

#if defined(DIAGFIRMWARE)
    { /* Put all values into ring buffer */
      auto & Recorder = g_Globals.Recorder;
      auto * record = Recorder.createRecord();

      if(record)
      {
        record->Ticks1s     = ticks1s;
        record->Ticks256Hz = ticks256Hz;
        record->NSig       = sigcnt;
        record->NRef       = refcnt;
        record->NPhase     = bsp.m_FreqPhaseCnt;

        Recorder.storeRecord(record);
      }
    }
#endif
  }

}

ISR(TIMER1_COMPA_vect)
{
  /* reenable input capture interrupt */
  TIFR1  = _BV(ICF1);
  TIMSK1 = _BV(OCIE1B) | _BV(ICIE1);
}

ISR(TIMER1_COMPB_vect)
{
  auto & bsp = Bsp::getInstance();

  /* reset state on time out */
  bsp.m_FreqCntState = bsp.FREQCNT_IDLE;
  bsp.m_FreqPhaseCnt = 0;

  TIFR1  = _BV(ICF1);
  TIMSK1 = _BV(OCIE1B) | _BV(ICIE1);
}

ISR(TIMER2_COMPA_vect)
{
  Bsp &bsp = Bsp::getInstance();

  {
    const uint8_t  tcnt2 = TCNT2;

    /* rearm timer for next event */
    OCR2A = tcnt2 + 7;

    bsp.FreqRefCnt2  = tcnt2 - bsp.FreqRefTCNT2;
    bsp.FreqRefTCNT2 = tcnt2;
  }

  {
    const uint16_t tcnt1 = TCNT1;
    bsp.FreqRefCnt1  = tcnt1 - bsp.FreqRefTCNT1;
    bsp.FreqRefTCNT1 = tcnt1;
  }

  if (TIFR2 & _BV(TOV2))
  {
    bsp.Ticks1s += 1;
    TIFR2      |= _BV(TOV2);
  }
}

uint16_t
Bsp::calcFrequency(uint8_t sigcnt, uint16_t refcnt)
{
  uint16_t freq;

  uint8_t  cnt2;
  uint16_t cnt1;

  /* consistent read of ref frequency state */
  do
  {
    cnt2 = FreqRefCnt2;
    cnt1 = FreqRefCnt1;
  } while(cnt2 != FreqRefCnt2);

  /* cnt1 is counting with 256 Hz */

  if(0)
  {
    uint32_t f_ref;

    f_ref = 256UL * cnt1 / cnt2;
    freq = (sigcnt * f_ref + refcnt / 2) / refcnt;
  }
  else
  {
    uint32_t divisor = cnt2 * refcnt;

    freq = (sigcnt * 256UL * cnt1 + cnt2 * refcnt / 2) / divisor;
  }

  return freq;
}

void
Bsp::enableRadar()
{
  s_PinRadarShutdown.enableOutput();
  g_Globals.Detector.init();
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

void Bsp::setDate(const DateTimeType::DateType & Date)
{
  m_Clock.set(Date);

  /* prevent time tick while setting the date */
  m_Clock.setSecond(0);
}

void Bsp::setTime(const DateTimeType::TimeType & Time)
{
  m_Clock.set(Time);

  /* prevent time tick while setting the date */
  m_Clock.setSecond(0);
}

uint8_t Bsp::getCal() const
{
  return OSCCAL;
}
Bsp Bsp::s_Instance;
