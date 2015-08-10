/*
 * globals.hpp
 *
 *  Created on: 10.08.2015
 *      Author: andi
 */

#ifndef CALIBRATOR_GLOBALS_HPP_
#define CALIBRATOR_GLOBALS_HPP_

#include <ecpp/Time.hpp>

using namespace ecpp;

class Globals
{
public:
  typedef SimpleTimer<uint16_t> MillisecondTimer;

private:
  MillisecondTimer m_Timers[1];

  static Globals s_Instance;
public:
  static Globals & getGlobals()
  {
    return s_Instance;
  }

  void handleTimers(uint8_t TicksPassed);
  MillisecondTimer & getUartTimer() {return m_Timers[0];}
};



#endif /* CALIBRATOR_GLOBALS_HPP_ */
