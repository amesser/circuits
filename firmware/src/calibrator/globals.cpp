/*
 * globals.cpp
 *
 *  Created on: 10.08.2015
 *      Author: andi
 */

#include "globals.hpp"

Globals Globals::s_Instance;

void
Globals::handleTimers(uint8_t TicksPassed)
{
  for(uint_fast8_t i = 0; i < ElementCount(m_Timers); ++i)
  {
    m_Timers[i].update(TicksPassed);
  }
}

