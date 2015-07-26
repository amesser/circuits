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
private:
  class LcdBsp {
  private:
    static IOPin<AVR_IO_PD1> PinE;
    static IOPin<AVR_IO_PD2> PinRw;
    static IOPin<AVR_IO_PD3> PinRs;
    static IOPort<AVR_IO_PC> PortNibble;
  protected:
    static void delay(uint16_t us) { while(us--) _delay_us(1);}
    static void clearRW()     {PinRw = 0;}
    static void clearRS()     {PinRs = 0;}
    static void clearEnable() {PinE  = 0;}
    static void setEnable()   {PinE  = 1;}
    static void setNibble(uint8_t data) {PortNibble.updateOutputs(data, 0x0F);}
  };
public:
  typedef LCD_HD44780<HD44780_MODE_4BIT, LcdBsp> LCDType;
};




#endif /* FIRMWARE_SRC_CALIBRATOR_BSP_HPP_ */
