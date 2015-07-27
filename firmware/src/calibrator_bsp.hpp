/*
 * calibrator_bsp.hpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#ifndef CALIBRATOR_BSP_HPP_
#define CALIBRATOR_BSP_HPP_

#include "ecpp/Target.hpp"
#include "ecpp/Peripherals/LCD_HD44780.hpp"

using namespace ecpp;
using namespace ecpp::Peripherals;

class CalibratorBsp
{
protected:
    static IOPin<AVR_IO_PD1> PinE;
    static IOPin<AVR_IO_PD2> PinRw;
    static IOPin<AVR_IO_PD3> PinRs;

    static IOPort<AVR_IO_PC> PortNibble;
    static IOPort<AVR_IO_PD> PortLcdControl;

  class LcdBsp
  {
  protected:
    static const FlashVariable<HD44780_CMD, 5> HD44780InitSequence PROGMEM;


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

  typedef LCD_HD44780<HD44780_MODE_4BIT, LcdBsp> LCDType;


  static void init()
  {
    PortLcdControl = PinE.OutHigh | PinRw.OutHigh | PinRs.OutHigh;
    LCDType().init();
  }
};

const FlashVariable<HD44780_CMD, 5> CalibratorBsp::LcdBsp::HD44780InitSequence PROGMEM =
{
   HD44780_CMD_FUNCTIONSET_4BIT | HD44780_CMD_FUNCTIONSET_2LINE | HD44780_CMD_FUNCTIONSET_5x7FONT,
   HD44780_CMD_DISPLAYCONTROL_DISPLAYON,
   HD44780_CMD_ENTRYMODE_INCREMENT | HD44780_CMD_ENTRYMODE_NOSHIFT,
   HD44780_CMD_HOME,
   HD44780_CMD_CLEAR,
};



#endif /* FIRMWARE_SRC_CALIBRATOR_BSP_HPP_ */
