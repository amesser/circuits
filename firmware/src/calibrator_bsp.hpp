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

using namespace ecpp;
using namespace ecpp::Peripherals;

/* Forward declarations of ISRs to define C linkage */
extern "C"
{
  void TIMER1_COMPA_vect (void) __attribute__ ((signal,__INTR_ATTRS));
  void USART_RXC_vect (void) __attribute__ ((signal,__INTR_ATTRS));
};

class CalibratorGlobals
{
protected:
  typedef SimpleTimer<uint16_t> MillisecondTimer;

  MillisecondTimer m_Timers[1];

  friend class CalibratorBsp;
};

class CalibratorBsp
{
public:
  typedef CalibratorGlobals::MillisecondTimer MillisecondTimer;
  typedef uint8_t UartBuffer[8];

protected:
  enum
  {
    UART_STATE_OFF   = 0,
    UART_STATE_RECV  = 1,
    UART_STATE_DONE  = 2,
  };

  static CalibratorBsp s_Instance;

  uint8_t              m_IsrTicks1ms;
  uint8_t              m_HandledTicks1ms;

  UartBuffer          *m_pUartBuffer;
  volatile uint8_t     m_UartRecvLen;
  volatile uint8_t     m_UartState;
  uint8_t              m_UartStatus0;

  CalibratorGlobals    m_Globals;

  MillisecondTimer & getUARTTimer() {return m_Globals.m_Timers[0];};

  static IOPin<AVR_IO_PD1> PinE;
  static IOPin<AVR_IO_PD2> PinRw;
  static IOPin<AVR_IO_PD3> PinRs;

  static IOPort<AVR_IO_PC> PortNibble;
  static IOPort<AVR_IO_PD> PortLcdControl;

  class LcdBsp
  {
  protected:
    static const FlashVariable<HD44780_CMD, 5> InitSequence PROGMEM;

  protected:
    static void delay(uint16_t us) { while(us--) _delay_us(1);}
    static void setRW()       {PortNibble.updateDirection(0x0F,0x00); PinRw = 1; }
    static void clearRW()     {PinRw = 0; PortNibble.updateDirection(0x0F,0x0F);}
    static void setRS()       {PinRs = 1;}
    static void clearRS()     {PinRs = 0;}
    static void clearEnable() {PinE  = 0;}
    static void setEnable()   {PinE  = 1;}
    static void setNibble(uint8_t data) {PortNibble.updateOutputs(data, 0x0F);}
  };
public:
  static CalibratorBsp & getBsp() {return s_Instance;}
  typedef LCD_HD44780<HD44780_MODE_4BIT, LcdBsp> LCDType;

  static void init();

  void handleTimers();

  void handleUART();

  void handle()
  {
    handleTimers();
    handleUART();
  }


  friend void TIMER1_COMPA_vect (void);
  friend void USART_RXC_vect (void);
};



#endif /* FIRMWARE_SRC_CALIBRATOR_BSP_HPP_ */
