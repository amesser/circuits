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

