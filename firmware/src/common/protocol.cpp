/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor firmwares.
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
#include "protocol.hpp"

static uint32_t mult32x16 (uint32_t lhs, uint16_t rhs) __attribute__((always_inline));
static uint32_t mult32x16 (uint32_t lhs, uint16_t rhs)
{
  uint32_t result = 0;

  while(rhs)
  {
    if(rhs & 0x0001)
    {
      result += lhs;
    }

    rhs >>= 1;
    lhs <<= 1;
  }

  return result;
}

uint16_t Evaluator::scale(uint16_t value) const
{

  if (value < Min)
  {
    value = 0;
  }
  else if(value <= Max)
  {
    uint16_t value_shift = Offset + value;
    uint32_t result     = mult32x16(Mult, value_shift);

    value = result >> 16U;
  }
  else
  {
    value = 0xFFFF;
  }

  return value;
}




