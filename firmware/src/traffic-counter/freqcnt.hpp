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
