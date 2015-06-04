/*
 * controller.cpp
 *
 *  Created on: 09.05.2015
 *      Author: andi
 */

#include "ecpp/Target.hpp"
#include "ecpp/Time.hpp"
#include "ecpp/Byteorder.hpp"
#include "ecpp/Peripherals/SDCard.hpp"
#include "ecpp/Ringbuffer.hpp"
#include "protocol.hpp"
#include <string.h>

using namespace ecpp;
using ::ecpp::Peripherals::SDCard;

static volatile uint8_t isrTickCnt;
Clock<uint16_t>         _Clock1ms;

static IOPort<AVR_IO_PC> LCDDataPort;

static IOPort<AVR_IO_PB> SDPort;
static IOPin<AVR_IO_PB0> SDPowerEnable;
static IOPin<AVR_IO_PB2> SDCS;
static IOPin<AVR_IO_PB3> SDDataIn;
static IOPin<AVR_IO_PB4> SDDataOut;
static IOPin<AVR_IO_PB5> SDClock;

static IOPort<AVR_IO_PD> LCDCtrlPort;
static IOPin<AVR_IO_PD1> SensorPowerEnable;
static IOPin<AVR_IO_PD2> LCDPinE;
static IOPin<AVR_IO_PD3> LCDPinRW;
static IOPin<AVR_IO_PD4> LCDPinRS;

class LCD
{
public:
  void setEnable()
  {
    LCDPinE.setOutput();
    asm volatile ( "nop" );
    asm volatile ( "nop" );
    asm volatile ( "nop" );
  }

  void clearEnable()
  {
    LCDPinE.clearOutput();
  }

  void delay50us()
  {
    _delay_us(50);
  }

  void delay5ms()
  {
    _delay_ms(5);
  }

  void writeNibble(uint8_t data)
  {
    LCDPinRW.clearOutput();

    setEnable();
    *(LCDDataPort.DDR) |= 0x0F;
    *(LCDDataPort.PORT) = data & 0x0F;
    clearEnable();
    delay50us();
  }

  uint8_t readNibble()
  {
    uint8_t data;

    *(LCDDataPort.DDR) &= ~0x0F;
    LCDPinRW.setOutput();

    setEnable();
    delay50us();
    data = *(LCDDataPort.PORT) & 0x0F;
    clearEnable();

    return data;
  }

  void writeByte(uint8_t data)
  {
    writeNibble(data >> 4);
    writeNibble(data & 0x0F);
  }

  uint8_t readByte()
  {
    uint8_t data;

    data =  readNibble() << 4;
    data |= readNibble();

    return data;
  }

  void writeData(uint8_t data)
  {
    LCDPinRS.setOutput();
    writeByte(data);
  }

  void writeData(uint8_t *ptr, uint8_t len)
  {
    while(len--)
    {
      writeData(*(ptr++));
    }
  }

  void writeData(char *ptr, uint8_t len)
  {
    while(len--)
    {
      writeData(*(ptr++));
    }
  }

  void writeCommand(uint8_t data)
  {
    LCDPinRS.clearOutput();
    writeByte(data);
  }

  uint8_t readStatus()
  {
    LCDPinRS.clearOutput();
    return readByte();
  }

  void moveCursor(uint8_t pos)
  {
    writeCommand(0x80 | pos);
  }

  void init()
  {
    delay5ms();
    delay5ms();
    delay5ms();
    delay5ms();

    LCDPinRW.clearOutput();
    LCDPinRS.clearOutput();
    writeNibble(0x03);
    delay5ms();
    writeNibble(0x03);
    delay5ms();
    writeNibble(0x03);
    writeNibble(0x02);

    writeCommand(0x28);
    writeCommand(0x0F);
    writeCommand(0x04);
    writeCommand(0x02);

    writeCommand(0x01);
    delay5ms();
  }
};

class SDCardDevice : public SPIMaster
{
private:
  Clock<uint16_t>::Timer<uint16_t> m_Timer;

protected:
  void enablePower()
  {
    SDPowerEnable.setOutput();
  }

  void disablePower()
  {
    SDPowerEnable.clearOutput();
  }

  void selectChip()
  {
    SDCS.clearOutput();
  }

  void deselectChip()
  {
    SDCS.setOutput();
  }

  void startTimer1ms(uint16_t timout)
  {
    m_Timer.init(_Clock1ms, timout);
  }

  uint16_t checkTimerExpired()
  {
    return m_Timer.hasTimedOut(_Clock1ms);
  }

  void setSlowSpiSpeed()
  {
    configureMasterSPI<0, 400000>();
  }

  void setFullSpiSpeed()
  {
    configureMasterSPI<0, 400000>();
  }
};


const uint16_t s_FormatDecimalSub[] = {10,100,1000,10000};

void format(char *buffer, uint8_t digits, uint16_t value, char fill =' ')
{
  char digit = fill;

  uint8_t    i = 5;

  while (digits > 5)
  {
    *(buffer++) = fill;
    digits--;
  }

  while(i > 1)
  {
    uint16_t   sub = s_FormatDecimalSub[i-2];

    if (value >= sub)
    {
      fill = '0';
      digit = '0';

      if(digits < i)
      {
        value = '#' - '0';
        break;
      }
      else
      {
        while(value >= sub)
        {
          value -= sub;
          digit ++;
        }
      }

    }
    else
    {
      digit = fill;
    }

    while(digits >= i)
    {
      *(buffer++) = digit;
      digits--;

    }

    i -= 1;
  }

  while ((digits--) > 0)
  {
    *(buffer++) = value + '0';
  }
}

static const char TextClock[9] = "XX:XX:XX";

class WaterControl
{
private:

  enum UartState
  {
    UART_RECV    = 0,
    UART_PROCESS = 10,
    UART_WAIT    = 11,
  };

  enum MasurementState
  {
    MEAS_IDLE = 0,
    MEAS_WAIT_UART,
    MEAS_PROCESS,
  };

  enum LogState
  {
    LOG_CLOSED     = 0,
    LOG_IDENTIFYSD = 1,
    LOG_COLLECT    = 2,
    LOG_ACTIVATESD = 3,
    LOG_WRITESD    = 4,
  };

  char    m_LCDBuffer[16];

  union {
    uint8_t m_UartBuffer[UART_PROCESS];
    struct measurement_data data;
  };


  uint8_t   m_UartStatus0;

  volatile uint8_t m_UartState;
  volatile uint8_t m_UartRecvLen;

  uint8_t m_MeasState;

  uint8_t                     m_LogState;
  uint16_t                    m_LogBlockSize;
  uint32_t                    m_LogBlockAddr;
  Ringbuffer<uint8_t,512>     m_LogRingbuffer;

public:
  Clock<uint16_t>::Timer<uint8_t>    m_UartTimer;
  Clock<uint16_t>::Timer<uint16_t>   m_MinuteTimer;
  Time                               m_Time;
  Time::Timer                        m_MeasurementTimer;
  Time::Timer                        m_LogTimer;

  LCD                  _LCD;
  SDCard<SDCardDevice> m_SDCard;

  void init()
  {
    _LCD.init();
    _LCD.writeData('A');
    _LCD.writeData('B');
    _LCD.writeData('C');
    _LCD.writeData('D');
    _LCD.writeData('E');
    _LCD.writeData('F');
  }

  void handleUART()
  {
    uint8_t state = m_UartState;
    uint8_t len = 0;

    if(m_UartRecvLen > state)
    {
      state = m_UartRecvLen;
      m_UartTimer.init(_Clock1ms, 20);
    }

    if(state > 0 && state < UART_PROCESS)
    {
      if (m_UartTimer.hasTimedOut(_Clock1ms))
      {
        len = state;
        state = UART_PROCESS;
      }
    }


    if(state == UART_PROCESS)
    {
      uint16_t divisor = (UBRRH << 8) | UBRRL;

      if (len >= 1)
      {
        int8_t delta = 0;

        if(data.sync & 0x20)
        {
          delta = -1;
        }
        else if (0 == (data.sync & 0x40))
        {
          delta = +1;
        }
        else if(0 == (data.sync & 0x80))
        {
          /* uart to slow, decriease ubr */
          delta = -1;
        }
        else if (m_UartStatus0 & 0x01)
        {
          delta = +1;
        }
        else if (m_UartStatus0 & _BV(PE))
        {
          delta = -1;
        }
        else if (m_UartStatus0 & _BV(FE))
        {
          delta = +1;
        }
        else if (m_UartStatus0 & (_BV(PE << 3)))
        {
          len = 0;
        }

        if(delta < 0)
        {
          if (divisor > 356)
          {
            divisor -= 1;
            UCSRB &= ~_BV(RXEN);
          }

          len = 0;
        }
        else if(delta > 0)
        {
          if (divisor < 499)
          {
            divisor += 1;
            UCSRB &= ~_BV(RXEN);
          }

          len = 0;
        }
      }

      if (len == 0)
      {
        memcpy(m_LCDBuffer, "BRR     ", 8);
        format(m_LCDBuffer + 4, 3, divisor);

        _LCD.moveCursor(0);
        _LCD.writeData(m_LCDBuffer, 8);
      }


      if(0 == (UCSRB & _BV(RXEN)))
      {
        UBRRH  = (divisor >> 8) & 0x7F;
        UBRRL  = divisor & 0xFF;
        UCSRB |= _BV(RXEN);
      }

      if (data.type == 0x00 && len == 8)
      {
        if(m_MeasState == MEAS_WAIT_UART)
          m_MeasState = MEAS_PROCESS;
      }

      state = UART_WAIT;
    }

    if(state == UART_WAIT && m_MeasState == MEAS_WAIT_UART)
    {
      m_UartTimer.init(_Clock1ms, 20);
      m_UartRecvLen = 0;

      state         = UART_RECV;
    }


    m_UartState = state;
  }

  void handleTime()
  {
    if(m_MinuteTimer.hasTimedOut(_Clock1ms))
    {
      m_MinuteTimer.updateTimeout(1000);
      m_Time.tick();

      memcpy(m_LCDBuffer, TextClock, 8);

      format(m_LCDBuffer,     2, m_Time.getHours(),'0');
      format(m_LCDBuffer + 3, 2, m_Time.getMinutes(),'0');
      format(m_LCDBuffer + 6, 2, m_Time.getSeconds(),'0');

      _LCD.moveCursor(8);
      _LCD.writeData(m_LCDBuffer, 8);
    }
  }

  void handleMeasurement()
  {
    uint8_t  state = m_MeasState;
    uint16_t tmout = m_MeasurementTimer.hasTimedOut(m_Time);

    if(state == MEAS_IDLE)
    {
      if(tmout)
      {
        state = MEAS_WAIT_UART;
      }
    }

    if(state == MEAS_WAIT_UART)
    {
      /* wait no longer than 2 minutes for result from sensor */
      if(tmout > 120)
      {
        state = MEAS_IDLE;
      }
    }

    if(state == MEAS_PROCESS)
    {
      /* data is valid */;
      memset(m_LCDBuffer, ' ', 16);
      format(m_LCDBuffer,      4, hton16(data.humidity_counts));
      format(m_LCDBuffer + 5,  5, hton16(data.led_counts));
      format(m_LCDBuffer + 11, 4, hton16(data.temp));

      _LCD.moveCursor(0x40);
      _LCD.writeData(m_LCDBuffer, 16);

      uint8_t i;

      /* write entries two time to detect bad data later */
      for(i = 0; i < 2; ++i)
      {
        if((m_LogRingbuffer.getCount() + 1 + 3 + 10 + 2) <= m_LogRingbuffer.getSize())
        {
          uint8_t *p;


          m_LogRingbuffer.pushForced(0x00);

          p = reinterpret_cast<uint8_t*>(&m_Time);

          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));

          p = m_UartBuffer;

          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));
          m_LogRingbuffer.pushForced(*(p++));

          m_LogRingbuffer.pushForced((uint8_t)UBRRH);
          m_LogRingbuffer.pushForced((uint8_t)UBRRL);
        }
      }

      memcpy(m_LCDBuffer, "RB      ", 8);
      format(m_LCDBuffer + 3, 4, m_LogRingbuffer.getCount());

      _LCD.moveCursor(0);
      _LCD.writeData(m_LCDBuffer, 8);


      state = MEAS_IDLE;
    }

    if(state == MEAS_IDLE)
    {
      SensorPowerEnable.setOutput();
    }
    else
    {
      SensorPowerEnable.clearOutput();
    }

    if(state == MEAS_IDLE && m_MeasState != MEAS_IDLE)
    {
      m_MeasurementTimer.updateTimeout(0,5,0);
    }

    m_MeasState = state;
  }


  enum LogState getLogState() const {return static_cast<enum LogState>(m_LogState);}
  void          setLogState(enum LogState state) {m_LogState = state;}

  void handleLog()
  {
    auto LogState = getLogState();

    m_SDCard.handleSDCard();

    if (LogState == LOG_CLOSED)
    {
      if(m_LogTimer.hasTimedOut(m_Time))
      {
        if(m_SDCard.getState() == m_SDCard.SD_CLOSED)
        {
          LogState = LOG_IDENTIFYSD;
          m_SDCard.activate();
        }
        else
        {
          m_LogTimer.initTimeout(m_Time, 0 , 1, 0);
        }
      }
    }

    if (LogState == LOG_IDENTIFYSD)
    {

      if(m_SDCard.getState() == m_SDCard.SD_CLOSED)
      {
        LogState = LOG_CLOSED;
        m_LogTimer.initTimeout(m_Time, 0 , 1, 0);
      }
      else if (m_SDCard.getState() == m_SDCard.SD_IDLE)
      {
        LogState       = LOG_COLLECT;

        m_LogBlockSize = m_SDCard.getBlockSize();
        m_LogBlockAddr = 0;

        m_SDCard.close();

        memcpy(m_LCDBuffer, "SD      ", 8);
        format(m_LCDBuffer + 3, 4, m_LogBlockSize);

        _LCD.moveCursor(0);
        _LCD.writeData(m_LCDBuffer, 8);

      }
    }

    if(LogState == LOG_COLLECT)
    {
      if(m_LogRingbuffer.getCount() >= m_LogBlockSize)
      {
        if(m_SDCard.getState() == m_SDCard.SD_CLOSED)
        {
          LogState = LOG_ACTIVATESD;
          m_SDCard.activate();
        }
      }
    }

    if(LogState == LOG_ACTIVATESD)
    {
      if(m_SDCard.getState() == m_SDCard.SD_CLOSED)
      {
        LogState = LOG_CLOSED;
        m_LogTimer.initTimeout(m_Time, 0 , 1, 0);
      }
      else if (m_SDCard.getState() == m_SDCard.SD_IDLE)
      {
        LogState = LOG_WRITESD;
        m_SDCard.startWriteBlock(m_LogBlockAddr);
        m_LogBlockAddr += m_LogBlockSize;
      }
    }

    if(LogState == LOG_WRITESD)
    {
      if(m_SDCard.getState() == m_SDCard.SD_WRITESINGLE)
      {
        if(m_SDCard.getBlockIndex() < m_SDCard.getBlockSize())
        {
          m_SDCard.writeByte(m_LogRingbuffer.popForced());

          memcpy(m_LCDBuffer, "   /    ", 8);
          format(m_LCDBuffer + 0, 3, m_SDCard.getBlockIndex());
          format(m_LCDBuffer + 4, 3, m_SDCard.getBlockSize());

          _LCD.moveCursor(0);
          _LCD.writeData(m_LCDBuffer, 8);
        }
      }
      else if (m_SDCard.getState() == m_SDCard.SD_IDLE)
      {
        LogState = LOG_COLLECT;
        m_SDCard.powerdown();

        memcpy(m_LCDBuffer, "SD DONE ", 8);

        _LCD.moveCursor(0);
        _LCD.writeData(m_LCDBuffer, 8);
      }
      else if (m_SDCard.getState() == m_SDCard.SD_CLOSED)
      {
        LogState = LOG_CLOSED;
        m_LogTimer.initTimeout(m_Time, 0 , 1, 0);

        memcpy(m_LCDBuffer, "SD FAIL ", 8);

        _LCD.moveCursor(0);
        _LCD.writeData(m_LCDBuffer, 8);
      }
    }

    setLogState(LogState);
  }

  void cycle()
  {
    handleTime();
    handleMeasurement();
    handleUART();
    handleLog();
  }


  void isrUartRecv()
  {
    uint8_t status, resh, resl;

    status = UCSRA;
    resh   = UCSRB;
    resl   = UDR;

    if(m_UartState < UART_PROCESS)
    {
      uint8_t mask   = _BV(PE) | _BV(FE);
      uint8_t offset = m_UartRecvLen;

      if(offset == 0)
      {
        m_UartStatus0 = ((resh >> 1) & 0x01) | (status & mask);
      }

      m_UartStatus0 |= (status & mask) << 3;

      if(offset < sizeof(m_UartBuffer))
      {
        m_UartBuffer[offset] = resl;
        m_UartRecvLen = offset + 1;
      }
    }
  }
};


static WaterControl App;

int main()
{
  /* setup timer 2 for ctc to generate 1 ms clock */
  OCR2  = 250 - 1;
  TCCR2 = _BV(WGM21) | _BV(CS21) | _BV(CS20);

  TIMSK = _BV(OCIE2);

  /* setup usart for receive 1200 baud */
  UBRRH = (0x7F & (416 >> 8));
  UBRRL = (0xFF &  416);

  UCSRA = 0;
  UCSRB = _BV(RXCIE) | _BV(RXEN)  | _BV(UCSZ2);
  UCSRC = _BV(URSEL) | _BV(UPM1)  /* | _BV(UPM0) */ | _BV(UCSZ1) | _BV(UCSZ0);

  *(LCDDataPort.PORT) = 0x00;
  *(LCDDataPort.DDR)  = 0x0F;

  *(LCDCtrlPort.PORT) = 0;
  *(LCDCtrlPort.DDR)  = LCDPinE.MASK | LCDPinRW.MASK | LCDPinRS.MASK | SensorPowerEnable.MASK;

  *(SDPort.PORT) = SDCS.MASK | SDPowerEnable.MASK;
  *(SDPort.DDR)  = SDCS.MASK | SDPowerEnable.MASK | SDDataIn.MASK | SDClock.MASK;

  Sys_AVR8::enableInterrupts();

  App.init();

  while(1)
  {
    _Clock1ms.update(isrTickCnt);

    App.cycle();

    Sys_AVR8::enterSleep();
    Sys_AVR8::enableSleep();
  }
}

ISR(TIMER2_COMP_vect)
{
  isrTickCnt += 1;
  Sys_AVR8::disableSleep();
}

ISR(USART_RXC_vect)
{
  App.isrUartRecv();
  Sys_AVR8::disableSleep();
}

