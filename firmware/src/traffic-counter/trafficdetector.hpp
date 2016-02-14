/*
 * trafficdetector.hpp
 *
 *  Created on: 14.02.2016
 *      Author: andi
 */

#ifndef TRAFFIC_COUNTER_TRAFFICDETECTOR_HPP_
#define TRAFFIC_COUNTER_TRAFFICDETECTOR_HPP_
#include "ecpp/Math/Statistics.hpp"

using namespace ecpp;

class TrafficDetector
{
public:
  typedef uint8_t             SpeedType;
  typedef uint8_t             SpeedCntType;
  typedef int8_t              PhaseCntType;

  enum State {
    STATE_W_ENTER = 0,
    STATE_W_LEAVE,
    STATE_DETECTED,
    STATE_ERROR,
  };

  static constexpr SpeedType    s_MaxSpeed        = 250;
  static constexpr SpeedCntType s_MinSpeedSamples = 2;
  static constexpr PhaseCntType s_MinPhaseCnt     = 10;

private:
  uint8_t                     m_state;
  Averager<uint16_t, SpeedCntType> m_SpeedAvg;

  SpeedType                   m_Speed;
  int8_t                      m_Direction;

  void setState(enum State State)
  {
    m_state = State;
  }

public:
  constexpr enum State getState()
  {
    return (enum State)(m_state);
  }

  void init()
  {
    m_SpeedAvg.init();
    setState(STATE_W_ENTER);
  }

  constexpr SpeedType getSpeed(void)
  {
    return m_Speed;
  }

  constexpr int8_t    getDirection(void)
  {
    return m_Direction;
  }

  template<typename SPEEDTYPE>
  void sampleMeasurement(SPEEDTYPE Speed);

  template<typename PHASETYPE>
  void timeout(PHASETYPE PhaseCnt);
};


template<typename SPEEDTYPE>
void TrafficDetector::sampleMeasurement(SPEEDTYPE Speed)
{
  auto const State = getState();

  if(Speed > s_MaxSpeed)
  {
    setState(STATE_ERROR);
  }
  else if(State == STATE_W_ENTER)
  {
    m_SpeedAvg.sample(Speed);

    setState(STATE_W_LEAVE);
  }
  else if(State == STATE_W_LEAVE)
  {
    m_SpeedAvg.sample(Speed);
  }
}

template<typename PHASETYPE>
void TrafficDetector::timeout(PHASETYPE PhaseCnt)
{
  auto const State = getState();

  if(State == STATE_W_LEAVE)
  {
    const auto SpeedCnt = m_SpeedAvg.counts();

    if(PhaseCnt > 0)
    {
      m_Direction = 1;
    }
    else
    {
      PhaseCnt = -PhaseCnt;
      m_Direction = -1;
    }

    if(s_MinSpeedSamples > SpeedCnt)
    {
      setState(STATE_ERROR);
    }
    else if (s_MinPhaseCnt > PhaseCnt)
    {
      setState(STATE_ERROR);
    }
    else
    {
      m_Speed = m_SpeedAvg.value();
      setState(STATE_DETECTED);
    }
  }
}


#endif /* TRAFFIC_COUNTER_TRAFFICDETECTOR_HPP_ */
