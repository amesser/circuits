/*
 * calibrator.cpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#include "calibrator_bsp.hpp"

class CalibratorApp
{
public:
  void readSensor();
};



void
CalibratorApp::readSensor()
{

}

int main(void) __attribute__ ((OS_main));
int main(void)
{
  auto & bsp = CalibratorBsp::getBsp();

  bsp.init();


  while(1)
  {
    bsp.handle();
    /* lcd.moveCursor(0);
     * lcd.writeData(abTest, 8); */
  }
}


