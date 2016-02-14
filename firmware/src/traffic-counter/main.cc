#include <string.h>

#include "ecpp/Target.hpp"
#include "ecpp/String.hpp"
#include "ecpp/Datatypes.hpp"
#include "ecpp/Ringbuffer.hpp"
#include "ecpp/Math/Statistics.hpp"
#include "ecpp/Graphics/TextFramebuffer.hpp"
#include "ecpp/Peripherals/TextLCD_HD44780_KS6600U.hpp"

#include "freqcnt.hpp"
#include "trafficdetector.hpp"

#include "diskio.h"
#include "ff.h"

using namespace ecpp;
using namespace ecpp::Peripherals;

static IOPort<AVR_IO_PB> s_PortB;
static IOPort<AVR_IO_PC> s_PortC;
static IOPort<AVR_IO_PD> s_PortD;

static IOPin<AVR_IO_PD0> s_PinLcdE;
static IOPin<AVR_IO_PD1> s_PinLcdRW;
static IOPin<AVR_IO_PD2> s_PinLcdRS;

static IOPin<AVR_IO_PD7> s_PinRadarShutdown;

/* input pin for one of the radar signal
 * this pin is used to compute the direction */
static IOPin<AVR_IO_PD3> s_PinRadarI;

/* input pin for the other one of the rdar signal
 * this pin is used to calculate speed */
static IOPin<AVR_IO_PD3> s_PinRadarQ;

static IOPin<AVR_IO_PB2> s_PinSDCardCS;
static IOPin<AVR_IO_PB3> s_PinSDCardSI;
static IOPin<AVR_IO_PB4> s_PinSDCardSO;
static IOPin<AVR_IO_PB5> s_PinSDCardCLK;

class BspLcd
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

void BspLcd::delayMicroseconds(uint16_t delay)
{
  _delay_loop_2(delay * 2);
}

uint8_t BspLcd::readByte()
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

void BspLcd::writeByte(uint8_t data, bool nibble)
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

void BspLcd::writeCommandNibble(uint8_t cmd)
{
  clearRS();
  writeByte(cmd, true);
}

uint8_t BspLcd::isBusy()
{
  clearRS();
  return (0x80 & readByte());
}

void BspLcd::writeCommand(uint8_t cmd)
{
  while(isBusy());

  clearRS();
  writeByte(cmd);
}

void BspLcd::writeRam(uint8_t data)
{
  while(isBusy());

  setRS();
  writeByte(data);
}

static TextFramebuffer<16,2> s_Framebuffer;
static TextLCD_HD44780_KS0066U<KS0066U_MODE_4BIT<BspLcd, 16,2> > s_Lcd __attribute__((section(".bss")));


static ReciprocalCounter s_FreqCounter;


ISR(TIMER1_CAPT_vect)
{
  decltype(s_FreqCounter.getState()) state;

  uint16_t cnt;
  bool     phase;

  /* Get current input capture value */
  cnt   = ICR1;
  phase = s_PinRadarI;

  state = s_FreqCounter.getState();

  /* initial time measurement */
  if(state == s_FreqCounter.STATE_WSTART)
  {
    /* Reset Timer 0 and clear compare interrupt flag
     * and clear capture */
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

  s_FreqCounter.countPhase(phase);
}


ISR(TIMER1_COMPB_vect)
{
  /* Stop Timer 0 */
  TCCR0B = 0;

  /* Disable timer 1 interrupts */
  TIMSK1 = 0;

  s_FreqCounter.timeout();
}

static uint8_t s_Tick1ms    = 0;
static uint8_t s_TickFat1ms = 0;

ISR(TIMER2_OVF_vect)
{
  s_Tick1ms += 4;
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

constexpr FlashVariable<char,16> s_Greeting PROGMEM   = "Started         ";
constexpr FlashVariable<char,16> s_MsgMountSD PROGMEM = "SD              ";


static TrafficDetector s_Detector;

static uint8_t  s_DetectCnt;

struct TcEvent
{
  uint8_t Speed;
  uint8_t Time;
  uint8_t Direction;
};

class Application
{
public:
  enum State {
    STATE_NODISK = 0,
    STATE_MOUNTED,
    STATE_OPENED,
    STATE_ERROR,
  };

private:
  uint8_t m_state;

  FATFS   m_Volume;
  FRESULT m_FileResult;
  FIL     m_File;

  uint8_t m_Cnt;

  /** Small ringbuffer to buffer events */
  Ringbuffer<struct TcEvent, 8> m_EventRing;
  char                          m_FileBuffer[512];

  void setState(enum State State) {m_state = State;}

public:
  constexpr enum State getState() {return (enum State)(m_state);}
  constexpr FRESULT    getFileResult(void) { return m_FileResult;}
  void poll();
};

void
Application::poll()
{
  auto state = getState();

  if(STATE_NODISK == state)
  {
    FRESULT result;

    result = f_mount(&m_Volume, "", 1);

    if(FR_NO_FILESYSTEM == result)
    {
      result = f_mkfs("",0,0);
    }

    if(FR_OK == result)
    {
      setState(STATE_MOUNTED);
      m_Cnt = 0;
    }

    m_FileResult = result;
  }
  else if(STATE_MOUNTED == state)
  {
    FRESULT result;

    char path [] = "XXXXXXXX.tc";

    if(m_Cnt < 100)
    {
      String::convertToDecimal(path+6, 2, m_Cnt);

      result = f_open(&m_File, "LOG.DAT", FA_WRITE | FA_CREATE_NEW);

      if(FR_OK == result)
      {
        setState(STATE_OPENED);
        f_sync(&m_File);
      }
      else if (FR_EXIST == result)
      {
        m_Cnt += 1;
      }
      else
      {
        setState(STATE_ERROR);
      }
    }
    else
    {
      setState(STATE_ERROR);
    }
  }
}

static Application s_App;

int main(void) __attribute__ ((OS_main));
int main()
{
  s_PortB = s_PinSDCardCLK.OutHigh | s_PinSDCardSO.InPullUp | s_PinSDCardSI.OutHigh | s_PinSDCardCS.OutHigh;
  s_PortD = s_PinRadarShutdown.In | s_PinLcdE.OutLow | s_PinLcdRS.OutLow | s_PinLcdRW.OutLow;

  CLKPR = _BV(CLKPCE);
  CLKPR = 0;

  /* Timer 0 counts the signal periods. We count rising edges of the signal
   * It shall generate an compare event if at least 10 periods have been count
   * */
  OCR0A  =  9;

  /* Timer/Counter 1 is to count the 1 Mhz reference count
   * Input capture is set to falling edge */
  TCCR1B = _BV(CS11);

  /* Timer/Count 2 generates a 4ms tick for timing purposes */
  OCR2A  = 250 - 1;
  TCCR1B = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS20);

  TIMSK2 = _BV(TOIE2);

  s_Lcd.init();
  s_Framebuffer.clear();

  s_Framebuffer.displayString({0,0}, s_Greeting.begin(), s_Greeting.end());

  /* enable radar */
  s_PinRadarShutdown.enableOutput();

  Sys_AVR8::enableInterrupts();

  StartMeas();

  s_Detector.init();

  while(1)
  {
    {
      auto & row = s_Framebuffer.getRow({0,0});
      auto State = s_App.getState();

      s_App.poll();

      if(State != s_App.getState())
      {

        s_Framebuffer.displayString({0,0}, s_MsgMountSD.begin(), s_MsgMountSD.end());

      }

      String::formatUnsigned(row + 3, 2, s_App.getFileResult());
      String::formatUnsigned(row + 6, 2, s_App.getState());
    }

    auto & row = s_Framebuffer.getRow({0,1});

    String::formatUnsigned(row, 3, TCNT0);

    String::formatUnsigned(row + 4, 1, s_FreqCounter.getState());

    //String::formatUnsigned(row + 6, 2, s_SpeedCnt);
    String::formatSigned(row + 9, 3, s_FreqCounter.getPhaseCnt());

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

      /* start another measurement */
      StartMeas();

      /* scale reference count with speed factor */
      n_ref = n_ref * speed_per_f;

      speed = (n_sig * f_ref + n_ref/2) / (n_ref);

      s_Detector.sampleMeasurement(speed);
      String::formatUnsigned(row + 16 - 5, 5, speed);
    }
    else if(s_FreqCounter.getState() == s_FreqCounter.STATE_TIMEOUT)
    {
      int8_t phasecnt;

      phasecnt = s_FreqCounter.getPhaseCnt();

      s_FreqCounter.init();

      StartMeas();

      s_Detector.timeout(phasecnt);

      if(s_Detector.getState() == s_Detector.STATE_DETECTED)
      {
        auto & row     = s_Framebuffer.getRow({0,0});
        auto DetectCnt = s_DetectCnt + 1;

        s_DetectCnt    = DetectCnt;

        if(s_Detector.getDirection() > 0)
        {
          row[16-1] = '>';
        }
        else
        {
          row[16-1] = '<';
        }

        String::formatUnsigned(row + 16 - 8, 3, DetectCnt);
        String::formatUnsigned(row + 16 - 4, 3, s_Detector.getSpeed());
      }

      s_Detector.init();
    }

    s_Framebuffer.update(s_Lcd);

    {
      uint8_t Delta = s_Tick1ms - s_TickFat1ms;

      if(Delta >= 10)
      {
        s_TickFat1ms += 10;
        disk_timerproc();
      }
    }
  };
}
