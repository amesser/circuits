/*
 * protocol.cpp
 *
 *  Created on: 24.08.2015
 *      Author: andi
 */
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




