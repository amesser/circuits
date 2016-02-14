/*
 * freqcnt.hpp
 *
 *  Created on: 14.02.2016
 *      Author: andi
 */

#ifndef TRAFFIC_COUNTER_FREQCNT_HPP_
#define TRAFFIC_COUNTER_FREQCNT_HPP_

#include <stdint.h>

class ReciprocalCounter
{
public:
  typedef uint16_t RefCntType;
  typedef uint8_t  SigCntType;
  typedef  int8_t  PhaseCntType;

private:
  static constexpr PhaseCntType s_MaxPhaseCnt = 127;

  volatile uint8_t      m_state;
  volatile PhaseCntType m_phasecnt;

  volatile RefCntType   m_refcntstart;
  volatile RefCntType   m_refcntend;

  volatile SigCntType   m_sigcnt;

public:
  enum State
  {
    STATE_WSTART    = 0,
    STATE_MEASURING = 1,
    STATE_FINISH    = 2,
    STATE_TIMEOUT   = 3,
  };

  enum State getState() const {return (enum State)(m_state);}

  void nextMeasurement()
  {
    m_state       = STATE_WSTART;
  }

  void init()
  {
    m_phasecnt = 0;
  }

  void startCounting(RefCntType refcntstart)
  {
    m_refcntstart = refcntstart;
    m_state       = STATE_MEASURING;
  }

  void stopCounting(RefCntType refcntend, SigCntType sigcnt)
  {
    m_refcntend   = refcntend;
    m_sigcnt      = sigcnt;

    m_state       = STATE_FINISH;
  }

  void countPhase(bool phase)
  {
    auto phasecnt = m_phasecnt;

    if(phase)
    {
      if (phasecnt < s_MaxPhaseCnt)
      {
        m_phasecnt = phasecnt + 1;
      }
    }
    else
    {
      if (phasecnt > (-s_MaxPhaseCnt))
      {
        m_phasecnt = phasecnt - 1;
      }
    }
  }

  void timeout()
  {
    m_state       = STATE_TIMEOUT;
  }

  SigCntType getSignalCnt() const
  {
    return m_sigcnt;
  }

  PhaseCntType getPhaseCnt() const
  {
    return m_phasecnt;
  }

  RefCntType getReferenceCnt() const
  {
    return m_refcntend - m_refcntstart;
  }
};




#endif /* TRAFFIC_COUNTER_FREQCNT_HPP_ */
