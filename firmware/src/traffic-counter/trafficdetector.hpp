/*
 *  Copyright 2016 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of the radar based traffic counting device firmware.
 *
 *  The Radar based traffic counting device firmware is free software: you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation,
 *  either version 3 of the License, or (at your option) any later
 *  version.
 *
 *  Embedded C++ Platform Project is distributed in the hope that it
 *  will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ECPP.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of ECPP give you
 *  permission to link ECPP with independent modules to produce an
 *  executable, regardless of the license terms of these independent
 *  modules, and to copy and distribute the resulting executable under
 *  terms of your choice, provided that you also meet, for each linked
 *  independent module, the terms and conditions of the license of that
 *  module.  An independent module is a module which is not derived from
 *  or based on ECPP.  If you modify ECPP, you may extend this exception
 *  to your version of ECPP, but you are not obligated to do so.  If you
 *  do not wish to do so, delete this exception statement from your
 *  version.
 *  */
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
