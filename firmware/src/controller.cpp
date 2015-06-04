/*
 * controller.cpp
 *
 *  Created on: 23.05.2015
 *      Author: andi
 */

#include "controller_bsp.hpp"

static BSP                BoardSupportPackage;

class HumidityController
{
  Timer<uint16_t> LedTimer;

  uint8_t cnt;
public:

  void cycle()
  {
    if (LedTimer.hasTimedOut(BoardSupportPackage.get1MsClock()))
    {
      LedTimer.updateTimeout(6000);
      BoardSupportPackage.toggleLed();

      BoardSupportPackage.enableOutputs(cnt & 0x0C);
      cnt += 0x04;
    }

    BoardSupportPackage.handleLCD();
  }
};

static HumidityController Application;

int main()
{
  BoardSupportPackage.initialize();

  while(1)
  {
    Sys_AVR8::enableSleep();

    BoardSupportPackage.cycle();
    Application.cycle();
  }
}

ISR(USART_RXC_vect)
{
  BoardSupportPackage.isrUartRecv();
  Sys_AVR8::disableSleep();
}

ISR(TIMER0_OVF_vect)
{
  BoardSupportPackage.isrTimer0Ovf();
  Sys_AVR8::disableSleep();

}

