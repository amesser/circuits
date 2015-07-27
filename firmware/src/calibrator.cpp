/*
 * calibrator.cpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#include "calibrator_bsp.hpp"

static CalibratorBsp g_bsp;
static const char abTest[] = "ABCDEFGH";

int main(void) __attribute__ ((OS_main));
int main(void)
{
  CalibratorBsp::LCDType lcd;

  g_bsp.init();


  while(1)
  {
    lcd.moveCursor(0);
    lcd.writeData(abTest, 8);
  }
}


