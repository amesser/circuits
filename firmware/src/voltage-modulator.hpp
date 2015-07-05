/*
 * voltage-modulator.hpp
 *
 *  Created on: 26.06.2015
 *      Author: andi
 */

#ifndef VOLTAGE_MODULATOR_HPP_
#define VOLTAGE_MODULATOR_HPP_

#include "ecpp/Datatypes.hpp"
#include "ecpp/Operators.hpp"
#include "ecpp/Target.hpp"

using namespace ecpp;

template<typename T, int MAX>
class WrappedInteger
{
};

const FlashVariable<uint8_t, 6> s_WrappedInteger2 PROGMEM = {0,1,2,0,1, 2};

template<typename T>
class WrappedInteger<T, 2>
{
private:
  T m_value;
public:
  constexpr WrappedInteger(const T init) : m_value(init) {};

  WrappedInteger operator - (const WrappedInteger & rhs)
  {
    return s_WrappedInteger2[m_value - rhs.m_value];
  }

  WrappedInteger & operator ++ ()
  {
    m_value = s_WrappedInteger2[m_value + 1];
    return *this;
  }

  WrappedInteger operator ++ (int)
  {
    T backup = m_value;
    ++(*this);
    return backup;
  }

  WrappedInteger & operator -- ()
  {
    m_value = s_WrappedInteger2[m_value + 3 - 1];
    return *this;
  }

  WrappedInteger operator -- (int)
  {
    T backup = m_value;
    --(*this);
    return backup;
  }

  bool operator == (const WrappedInteger & rhs) const
  {
    return m_value == rhs.m_value;
  }

  bool operator != (const WrappedInteger & rhs) const
  {
    return m_value == rhs.m_value;
  }

  constexpr T asInteger() {return m_value;}
};


template<int BITTIME, int GUARD, class BASE>
class VoltageModulator : public BASE
{
public:
  enum VoltageState
  {
    VOLTAGE_A = 0,
    VOLTAGE_B = 1,
    VOLTAGE_C = 2,
  };

  typedef WrappedInteger<uint8_t, VOLTAGE_C> StateType;

  static void transmitt(const void* ptr, size_t len);

  template<typename SIZE>
  static bool receive(void* ptr, SIZE maxlen);

};

template<int BITTIME, int GUARD, class BASE>
void VoltageModulator<BITTIME, GUARD, BASE>::transmitt(const void* ptr, size_t len)
{
  const uint8_t *buf = ptr;
  StateType state = VOLTAGE_A;

  BASE::setState(state);
  BASE::delay(BITTIME + 2 * GUARD);

  while(len--)
  {
    const uint8_t value = *(buf++);

    for(uint8_t mask = 0x80; mask; mask >>= 1)
    {
      if(mask & value)
      {
        state += StateType(2);
      }
      else
      {
        state += StateType(1);
      }

      BASE::setState(state);
      BASE::delay(BITTIME + GUARD);
    }
  }
}

template<int BITTIME, int GUARD, class BASE>
template<typename SIZE>
bool
VoltageModulator<BITTIME, GUARD, BASE>::receive(void* ptr, SIZE maxlen)
{
  uint8_t *buf = static_cast<uint8_t*>(ptr);

  SIZE    offset;
  uint8_t value, count;

  decltype(BASE::startTimer(30000)) timer;

  StateType state     = BASE::getState();
  uint8_t   laststate = 4; /* force initial reset of timer */

  offset = 0;
  count  = 0;

  while(maxlen > 0)
  {
    StateType curstate = BASE::getState();

    if(curstate.asInteger() != laststate)
    {
      laststate = curstate.asInteger();
      timer = BASE::startTimer(30000);
    }
    else if (curstate != state)
    {
      if (timer.getElapsedTime(30000) > BITTIME)
      {
        const auto delta = (curstate - state).asInteger();
        state = curstate;

        value  = (value << 1) | (delta - 1);
        count += (256 / 8);

        if (0 == count)
        {
          buf[offset] = value;
          offset++;
          maxlen--;
        }
      }
    }

    if(BASE::checkTimeout(timer))
    {
      maxlen = 0;
    }
  }

  return offset;
}

#endif /* FIRMWARE_SRC_VOLTAGE_MODULATOR_HPP_ */
