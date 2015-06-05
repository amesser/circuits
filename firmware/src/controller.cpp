/*
 * controller.cpp
 *
 *  Created on: 23.05.2015
 *      Author: andi
 */

#include "controller_bsp.hpp"

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

      auto line = BoardSupportPackage.getDisplayLine(0);
      line[0] = 'a';
      line[1] = 'b';
      line[2] = 'c';
      line[3] = 'd';
      line[4] = 'e';
      line[5] = 'f';
      line[6] = 'g';
      line[7] = 'h';
      line[8] = 'j';
      line[9] = 'k';
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



