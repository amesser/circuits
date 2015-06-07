/*
 * controller_bsp.hpp
 *
 *  Created on: 04.06.2015
 *      Author: andi
 */

#include "ecpp/Target.hpp"
#include "ecpp/Time.hpp"
#include "ecpp/Byteorder.hpp"
#include "ecpp/Peripherals/SDCard.hpp"
#include "ecpp/Ringbuffer.hpp"
#include "ecpp/String.hpp"
#include <string.h>

#include "protocol.hpp"
#include "font.hpp"

using namespace ecpp;



class BSP
{
public:
  enum UartState
  {
    UART_RECV0    = 0,
    UART_RECV9    = 9,
    UART_RECVDONE = 10,
  };

private:
  static IOPort<AVR_IO_PB> PortB;
  static IOPort<AVR_IO_PC> PortC;
  static IOPort<AVR_IO_PD> PortD;

  static IOPin<AVR_IO_PB0> PinLCD_Reset;
  static IOPin<AVR_IO_PB1> PinBoostConverter;
  static IOPin<AVR_IO_PB2> PinLCD_A0;
  static IOPin<AVR_IO_PB3> PinSPI_MOSI;
  static IOPin<AVR_IO_PB4> PinSPI_MISO;
  static IOPin<AVR_IO_PB5> PinSPI_SCK;

  static IOPin<AVR_IO_PC0> PinKey0;
  static IOPin<AVR_IO_PC1> PinKey1;
  static IOPin<AVR_IO_PC2> PinKey2;
  static IOPin<AVR_IO_PC3> PinADCVPwr;
  static IOPin<AVR_IO_PC4> PinLed;
  static IOPin<AVR_IO_PC5> PinSD_CS;

  static IOPin<AVR_IO_PD0> PinSens_RXD;
  static IOPin<AVR_IO_PD1> PinCAN_CS;
  static IOPin<AVR_IO_PD2> PinOut_ChB;
  static IOPin<AVR_IO_PD3> PinOut_ChA;
  static IOPin<AVR_IO_PD4> PinSD_PwrE;
  static IOPin<AVR_IO_PD5> PinLCD_CS;
  static IOPin<AVR_IO_PD6> PinSens_ChA;
  static IOPin<AVR_IO_PD7> PinSens_ChB;

public:
  enum OutputMask
  {
    OUTPUT_CHA = PinOut_ChA.MASK,
    OUTPUT_CHB = PinOut_ChB.MASK,
  };

  enum AccuState
  {
    ACCU_EMPTY = 0,
    ACCU_OK    = 1,
  };

  struct LCD
  {
    char Lines[4][10];
    char Status[10];
  };


private:

  static struct Uart
  {
    uint8_t RcvLen;
    uint8_t Status;
    uint8_t Buffer[10];
  } s_Uart;

  static struct Time
  {
    volatile uint8_t      IsrTickCnt;
    Clock<uint16_t>       Clock1Ms;
    Timer<uint16_t, 5000> SecondTimer;
  } s_Time;

  static struct Timers1Ms
  {
    SimpleTimer<uint16_t> Boost;
    SimpleTimer<uint16_t> Accu;
    SimpleTimer<uint16_t> Keyboard;
    SimpleTimer<uint16_t> Ui;
    SimpleTimer<uint16_t> UiRefresh;
  } s_Timers1Ms;

  enum BoostState
  {
    BOOST_OFF = 0,
    BOOST_STARTUP,
    BOOST_ACTIVE
  };


  static struct Outputs
  {
    uint8_t State;
  } s_Outputs;

  static struct LCD s_LCD;
  struct Keyboard
  {
    uint8_t mask;
  } m_Keyboard;


  static struct Accumulator
  {
    uint16_t Voltage;
    uint8_t  State;
  } s_Accu;

public:
  static Clock<uint16_t> & get1MsClock() {return s_Time.Clock1Ms;}

  static struct LCD &getDisplay() {return s_LCD;}

  uint8_t  getKeyState() const {return m_Keyboard.mask >> 4;}
  uint16_t getKeyTime()  const {return s_Timers1Ms.Keyboard.getElapsedTime(10000);}

  SimpleTimer<uint16_t> & getUiTimer()        const { return s_Timers1Ms.Ui;}
  SimpleTimer<uint16_t> & getUiRefreshTimer() const { return s_Timers1Ms.UiRefresh;}

  static uint16_t getAccuVoltage() {return s_Accu.Voltage;}
  static uint8_t  getAccuState()   {return s_Accu.State;}

  static void toggleLed()
  {
    if(PinLed.getOutput())
    {
      PinLed.clearOutput();
    }
    else
    {
      PinLed.setOutput();
    }
  }

  static void enableOutputs(uint8_t mask)
  {
    s_Outputs.State = s_Outputs.State | mask;
  }

  static void disableOutputs(uint8_t mask)
  {
    s_Outputs.State = s_Outputs.State & (~mask);
  }

  static uint8_t getOutputState()
  {
    return s_Outputs.State & (PinOut_ChA.MASK | PinOut_ChB.MASK);
  }

  static uint8_t getRealOutputState()
  {
    return PortD & (PinOut_ChA.MASK | PinOut_ChB.MASK);
  }

  static void initialize();

  static void isrUartRecv() __attribute__((always_inline))
  {
    /* we must readout the registers always, otherwise
     * the interrupt will not be cleared */
    const uint8_t status = UCSRA; /* fe/pe error bits */
    const uint8_t resh   = UCSRB; /* 9th bit */
    const uint8_t resl   = UDR;   /* data */

    auto Offset = s_Uart.RcvLen;

    if(Offset < sizeof(s_Uart.Buffer))
    {
        const uint8_t mask   = _BV(PE) | _BV(FE);

      if(Offset == 0)
      {
        s_Uart.Status  = ((resh >> RXB8) & 0x01) | (status & mask);
      }
      else
      {
        s_Uart.Status |= (status & mask) << 3;
      }

      s_Uart.Buffer[Offset] = resl;
      Offset += 1;

      s_Uart.RcvLen = Offset;
    }

    if(Offset >= sizeof(s_Uart.Buffer))
    {
      UCSRB &= _BV(RXEN);
    }
  }

  static void isrTimer0Ovf() __attribute__((always_inline))
  {
    /* Generate 125 Hz Irq */
    TCNT0 = 256 - 250;
    s_Time.IsrTickCnt += 8;
  }

  void cycle();

  static void transferCommand(uint8_t command)
  {
    SPIMaster spi;

    PinLCD_CS.clearOutput();
    spi.transferByte(command);
    PinLCD_CS.setOutput();

  }

  static void initLCD()
  {
    SPIMaster spi;
    spi.configureMasterSPI<3,4000000>();

    PinLCD_A0.clearOutput();
    PinLCD_CS.clearOutput();

    PinLCD_Reset.clearOutput();
    _delay_ms(100);
    PinLCD_Reset.setOutput();
    _delay_us(5);

    spi.transferByte(0xA2);
    spi.transferByte(0xA1); /* this command is not working */
    spi.transferByte(0xC0);

    spi.transferByte(0x25);
    spi.transferByte(0x81);
    spi.transferByte(0x2F);
    spi.transferByte(0x2F);

    spi.transferByte(0x40);

    for (uint8_t page = 0; page < 8; ++page)
    {
      PinLCD_A0.clearOutput();

      spi.transferByte(0xB0 + page);
      spi.transferByte(0x10);
      spi.transferByte(0x00);

      PinLCD_A0.setOutput();
      for(int i = 0; i < 101; ++i)
        spi.transferByte(0x01 << (i% 8));
      PinLCD_A0.clearOutput();
    }

    spi.transferByte(0xAF);

    PinLCD_CS.setOutput();

  }

  void handleKeyboard();

  static void handleOutputs();
  static void handleLCD();
};


extern BSP bsp;
