/*
 * calibrator.cpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#include "calibrator_bsp.hpp"


int main(void) __attribute__ ((OS_main));
int main(void)
{
  CalibratorBsp::LCDType lcd;


  while(1)
  {
    lcd.init();

  }
}


