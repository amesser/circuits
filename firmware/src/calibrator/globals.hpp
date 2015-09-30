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
  typedef SimpleTimer<uint16_t,1>    MillisecondTimer;
  typedef SimpleTimer<uint16_t,1000> SecondTimer;

private:
  MillisecondTimer m_MillisecondTimers[4];
  SecondTimer      m_SecondTimers[2];

  static Globals s_Instance;
public:
  static Globals & getGlobals()
  {
    return s_Instance;
  }

  void handleTimers(uint8_t MillisecondsPassed, uint8_t SecondsPassed);

  MillisecondTimer & getUartTimer()              {return m_MillisecondTimers[0];}
  MillisecondTimer & getKeyTimer()               {return m_MillisecondTimers[1];}
  MillisecondTimer & getVoltageModulatorTimer()  {return m_MillisecondTimers[2];}
  SecondTimer      & getTemperatureTimer()       {return m_SecondTimers[0];}
  SecondTimer      & getAutomaticTimer()         {return m_SecondTimers[1];}
};



#endif /* CALIBRATOR_GLOBALS_HPP_ */
