/*
 * globals.cpp
 *
 *  Created on: 10.08.2015
 *      Author: andi
 */

#include "globals.hpp"

Globals Globals::s_Instance;

void
Globals::handleTimers(uint8_t MillisecondsPassed, uint8_t SecondsPassed)
{
  for(uint_fast8_t i = 0; i < ElementCount(m_MillisecondTimers); ++i)
  {
    m_MillisecondTimers[i].handleMillisecondsPassed(MillisecondsPassed);
  }

  for(uint_fast8_t i = 0; i < ElementCount(m_SecondTimers); ++i)
  {
    m_SecondTimers[i].handleSecondsPassed(SecondsPassed);
  }

}

