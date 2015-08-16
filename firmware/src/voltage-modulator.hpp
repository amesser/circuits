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
#include "ecpp/WrappedNumbers.hpp"

using namespace ecpp;

template<int LENGTH, int BITTIME, int GUARD, class BASE>
class VoltageModulator : public BASE
{
public:
  typedef uint8_t BuffertType[LENGTH];
  enum VoltageState
  {
    VOLTAGE_A = 0,
    VOLTAGE_B = 1,
    VOLTAGE_C = 2,
  };
private:
  typedef WrappedInteger<uint8_t, VOLTAGE_C > StateType;


  StateType      m_State;
  uint_least8_t  m_Offset;
  uint_least8_t  m_BitCnt;
  uint_least8_t  m_Value;
  BuffertType    m_Buffer;
public:
  template<typename T = BuffertType>
  T & getBuffer() {return *reinterpret_cast<T*>(&m_Buffer);}

  uint_least8_t getNumTransferred() const {return (m_BitCnt > 0) ? (m_Offset - 1) : (m_Offset);}
  uint_least8_t getTransferring()    const {return m_Offset;}
  bool          hasFinished()       const {return (m_BitCnt == 0) && (m_Offset == LENGTH);}

  /* event based interface */
  void startTransmission();
  void handleTransmission();

  static void transmitt(const void* ptr);
  static bool receive(void* ptr);
};


template<int LENGTH, int BITTIME, int GUARD, class BASE>
void VoltageModulator<LENGTH, BITTIME, GUARD, BASE>::transmitt(const void* ptr)
{
  const uint8_t *buf = ptr;
  uint8_t len = LENGTH;
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

template<int LENGTH, int BITTIME, int GUARD, class BASE>
void VoltageModulator<LENGTH, BITTIME, GUARD, BASE>::startTransmission()
{
  StateType State = VOLTAGE_A;

  m_State   = State;
  m_Offset  = 0;
  m_BitCnt  = 1;

  BASE::setVoltageState(State.asInteger());
}

template<int LENGTH, int BITTIME, int GUARD, class BASE>
void VoltageModulator<LENGTH, BITTIME, GUARD, BASE>::handleTransmission()
{
  if (!hasFinished())
  {
    auto & Timer = BASE::getTimer();

    if(Timer.hasTimedOut())
    {
      StateType State = m_State;

      m_BitCnt -= 1;

      if (m_Offset < LENGTH || m_BitCnt > 0)
      {

        if(m_BitCnt == 0)
        {
          m_BitCnt = 8;
          m_Value  = m_Buffer[m_Offset++];
        }

        if(m_Value & 0x80)
        {
          State += StateType(2);
        }
        else
        {
          State += StateType(1);
        }

        m_Value <<= 1;

        Timer.start(BITTIME + GUARD);
      }
      else
      {
        State = StateType(VOLTAGE_A);
      }

      m_State = State;
      BASE::setVoltageState(State.asInteger());
    }
  }
}

template<int LENGTH, int BITTIME, int GUARD, class BASE>
bool
VoltageModulator<LENGTH, BITTIME, GUARD, BASE>::receive(void* ptr)
{
  uint8_t *buf = static_cast<uint8_t*>(ptr);
  uint8_t maxlen = LENGTH;

  uint8_t offset;
  uint8_t value, count;

  decltype(BASE::startTimer(30000)) timer;

  StateType State     = BASE::getState();
  uint8_t   laststate = 4; /* force initial reset of timer */

  offset = 0;
  count  = 0;

  while(maxlen > 0)
  {
    StateType NextState = BASE::getState();

    if(NextState.asInteger() != laststate)
    {
      laststate = NextState.asInteger();
      timer = BASE::startTimer(30000);
    }
    else if (NextState != State)
    {
      if (timer.getElapsedTime(30000) > BITTIME)
      {
        const auto delta = (NextState - State).asInteger();
        State = NextState;

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
