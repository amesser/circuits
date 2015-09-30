/*
 * calibrator_bsp.hpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#ifndef CALIBRATOR_BSP_HPP_
#define CALIBRATOR_BSP_HPP_

#include "ecpp/Target.hpp"
#include "ecpp/Time.hpp"

#include "ecpp/Peripherals/LCD_HD44780.hpp"
#include "uart_atmega.hpp"

#include "voltage-modulator.hpp"
#include "protocol.hpp"
#include "globals.hpp"

using namespace ecpp;
using namespace ecpp::Peripherals;

/* Forward declarations of ISRs to define C linkage */
extern "C"
{
  void TIMER1_COMPA_vect (void) __attribute__ ((signal,__INTR_ATTRS));
  void USART_RXC_vect (void) __attribute__ ((signal,__INTR_ATTRS));
  void TWI_vect (void) __attribute__ ((signal,__INTR_ATTRS));
};

template<unsigned int N>
class TWIMaster
{
private:
  uint8_t m_Buffer[N+1];
  uint8_t m_Idx;
  uint8_t m_Len;
public:
  typedef uint8_t TransferBufferType[N];

  template<typename T = TransferBufferType>
  T & getBuffer()
  {
    union
    {
      T*       pReturn;
      uint8_t* pBuffer;
    };

    pBuffer = &(m_Buffer[1]);
    return *pReturn;
  }

  void init(uint32_t Frequency)
  {
    /* TWBR * 4^TWPS */
    uint16_t divider   = (F_CPU / Frequency - 16) / 2;
    uint8_t  prescaler = 0;

    while(divider > 255 && prescaler < 3)
    {
      divider    = divider / 4;
      prescaler += 1;
    }

    TWCR = 0;
    TWBR = divider;
    TWSR = prescaler;
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) /* | _BV(TWIE) */;
  }

  void handleIrq();
  void handleCyclic();

  void sendStartAndWrite(uint8_t address, uint8_t len);
  void sendStartAndRead(uint8_t address, uint8_t len);
  void sendStop();

  bool    hasFinished() {return m_Idx >= m_Len;}
  uint8_t getError()    {return (m_Len == 0) ? (TWSR & 0xF8) : 0;}
  uint8_t getStatus()   {return (TWSR & 0xF8);}
};

template<unsigned int N>
void TWIMaster<N>::handleIrq()
{
  TWCR = TWCR & ~(_BV(TWIE) | _BV(TWSTA));
}

template<unsigned int N>
void TWIMaster<N>::handleCyclic()
{
  if(TWCR & _BV(TWINT) && m_Len > m_Idx)
  {
    uint8_t status = TWSR & 0xF8;

    switch(status)
    {
    case 0x08: /* start condition */
    case 0x10: /* repeated start cond */
      if(m_Idx < m_Len)
      {
        TWDR  = m_Buffer[m_Idx];
        TWCR  = /*_BV(TWIE) |*/ _BV(TWEN) | _BV(TWINT);
      }
      break;
    case 0x18: /* sla+w + ack */
    case 0x28: /* write + ack  */
      {
        m_Idx += 1;

        if(m_Idx < m_Len)
        {
          TWDR   = m_Buffer[m_Idx];
          TWCR  = /*_BV(TWIE) | */ _BV(TWEN) | _BV(TWINT);
        }
      }
      break;
    case 0x20: /* sla+w + nack */
    case 0x30: /* write + nack */
    case 0x48: /* sla+r + nack, */
    case 0x58: /* read  + nack return */
      m_Len = 0; /* this means nack */
      break;
#if 0
    case 0x38: /* arbitration lost */
      if(m_Idx < m_Len)
      {
        TWCR = /* _BV(TWIE) | */ _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);
      }
      break;
#endif
    case 0x40: /* sla+r + ack */
      {
        m_Idx += 1;

        if(m_Idx < m_Len)
        {
          TWCR  = /* _BV(TWIE) | */ _BV(TWEN) | _BV(TWEA) | _BV(TWINT);
        }
      }
      break;
    case 0x50: /* read + ack return */
      {
        m_Buffer[m_Idx] = TWDR;
        m_Idx += 1;

        if(m_Idx < m_Len)
        {
          TWCR  = /* _BV(TWIE) | */ _BV(TWEN) | _BV(TWEA) | _BV(TWINT);
        }
      }
      break;
    }
  }
}

template<unsigned int N>
void TWIMaster<N>::sendStartAndWrite(uint8_t address, uint8_t len)
{
  m_Buffer[0] = (address << 1);
  m_Idx       = 0;
  m_Len       = len + 1;

  //while(TWCR & _BV(TWSTO));
  TWCR = _BV(TWINT) | _BV(TWSTA) /* | _BV(TWIE) */ | _BV(TWEN);
}

template<unsigned int N>
void TWIMaster<N>::sendStartAndRead(uint8_t address, uint8_t len)
{
  m_Buffer[0] = (address << 1) | 0x01;
  m_Idx       = 0;
  m_Len       = len + 1;

  //while(TWCR & _BV(TWSTO));
  TWCR = _BV(TWINT) | _BV(TWSTA) /* | _BV(TWIE) */ | _BV(TWEN);
}

template<unsigned int N>
void TWIMaster<N>::sendStop()
{
  TWCR = _BV(TWINT) | _BV(TWSTO) /* | _BV(TWIE) */ | _BV(TWEN);
}

class CalibratorBsp
{
protected:
  static IOPin<AVR_IO_PD1> PinE;
  static IOPin<AVR_IO_PD2> PinRw;
  static IOPin<AVR_IO_PD3> PinRs;

  static IOPort<AVR_IO_PB> PortB;
  static IOPort<AVR_IO_PC> PortNibble;
  static IOPort<AVR_IO_PD> PortLcdControl;

  static IOPin<AVR_IO_PD6> Pin1k8;
  static IOPin<AVR_IO_PD7> Pin8k2;
  static IOPin<AVR_IO_PB0> Pin3k3;

  static IOPin<AVR_IO_PB2> PinKeyA;
  static IOPin<AVR_IO_PB1> PinKeyB;

  class VoltageModulatorBsp
  {
  protected:
    void                        setVoltageState(uint_fast8_t State) {CalibratorBsp::setSensorVoltage(State);}
    Globals::MillisecondTimer & getTimer() {return Globals::getGlobals().getVoltageModulatorTimer();}
  };


  class LcdBsp
  {
  protected:
    static const FlashVariable<HD44780_CMD, 5> InitSequence PROGMEM;

    static void delay(uint_fast16_t us);
    static void setRW()       {PortNibble.updateDirection(0x0F,0x00); PinRw = 1; }
    static void clearRW()     {PinRw = 0; PortNibble.updateDirection(0x0F,0x0F);}
    static void setRS()       {PinRs = 1;}
    static void clearRS()     {PinRs = 0;}
    static void clearEnable() {PinE  = 0;}
    static void setEnable()   {PinE  = 1;}
    static void setNibble(uint8_t data) {PortNibble.updateOutputs(data, 0x0F);}
  };

public:
  typedef LCD_HD44780<HD44780_MODE_4BIT, LcdBsp>                                 LCDType;
  typedef AdaptingUart<8>                                                        UartHandlerType;
  typedef VoltageModulator<sizeof(calibration_data), 80,20, VoltageModulatorBsp> VoltageModulatorType;
  typedef TWIMaster<4> TWIMasterType;

protected:
  static CalibratorBsp s_Instance;

  volatile uint16_t m_IsrTicks;
  uint16_t          m_HandledSecondTicks;
  uint8_t           m_HandledMillisecondTicks;
  uint_fast8_t          m_KeyState;

  UartHandlerType       m_UartHandler;

  LCDType               m_Lcd;
  VoltageModulatorType  m_VoltageModulator;
  TWIMasterType         m_TWIMaster;
public:
  enum
  {
    SUPPLY_OFF = 4,
    SUPPLY_5V  = 2,
    SUPPLY_4V  = 1,
    SUPPLY_3V  = 0,
  };

  static CalibratorBsp & getBsp() {return s_Instance;}

  LCDType & getLCD()                           {return m_Lcd;}
  UartHandlerType & getUartHandler()           {return m_UartHandler;}
  VoltageModulatorType & getVoltageModulator() {return m_VoltageModulator;}
  TWIMasterType &        getTWIHandler()       {return m_TWIMaster;}

  static void init();


  static uint_fast16_t getDivisor() {return (UBRRH << 8) | UBRRL;}

  static void setSensorVoltage(uint8_t state);

  void cycle();
  uint_fast8_t getKeyState() const {return m_KeyState & 0x0F;}

  friend void TIMER1_COMPA_vect (void);
  friend void USART_RXC_vect (void);
  friend void TWI_vect (void);
};



#endif /* FIRMWARE_SRC_CALIBRATOR_BSP_HPP_ */
