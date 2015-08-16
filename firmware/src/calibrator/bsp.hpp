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
};


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

protected:
  static CalibratorBsp s_Instance;

  volatile uint_fast8_t m_IsrTicks1ms;
  uint_fast8_t          m_HandledTicks1ms;
  uint_fast8_t          m_KeyState;

  UartHandlerType       m_UartHandler;

  LCDType               m_Lcd;
  VoltageModulatorType  m_VoltageModulator;
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

  static void init();


  static uint_fast16_t getDivisor() {return (UBRRH << 8) | UBRRL;}

  static void setSensorVoltage(uint8_t state);

  void cycle();
  uint_fast8_t getKeyState() const {return m_KeyState & 0x0F;}

  friend void TIMER1_COMPA_vect (void);
  friend void USART_RXC_vect (void);
};



#endif /* FIRMWARE_SRC_CALIBRATOR_BSP_HPP_ */
