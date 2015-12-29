/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor Calibrator firmware.
 *
 *  This software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *  */
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
