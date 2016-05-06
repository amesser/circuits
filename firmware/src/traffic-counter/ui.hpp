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
#ifndef TRAFFIC_COUNTER_UI_HPP_
#define TRAFFIC_COUNTER_UI_HPP_

#include <stdint.h>
#include <ecpp/Time.hpp>
#include <ecpp/Graphics/TextFramebuffer.hpp>

using namespace ecpp;

class Ui
{
public:
  typedef uint8_t                       KeyMaskType;
  typedef TextFramebuffer<16,2>         FramebufferType;
  /* TODO: 256Hz != 4ms */
  typedef SimpleTimer<uint16_t,4>       UiTimerType;

  enum State
  {
    STATE_STARTUP     = 0,
    STATE_STARTREC,
    STATE_CHECKSD,
    STATE_FORMATSD,
    STATE_FORMATINGSD,
    STATE_ENABLERADAR,
    STATE_ADJLENGTHL,
    STATE_ADJLENGTHR,
    STATE_RECORDING,
    STATE_CNFSTOP,
    STATE_STOPPING,
    STATE_CONFIGCLOCK,
    STATE_CONFIGYEAR,
    STATE_CONFIGMONTH,
    STATE_CONFIGDAY,
    STATE_CONFIGHOUR,
    STATE_CONFIGMINUTE,
    STATE_CANCEL,
    STATE_BATEMPTY,
    STATE_REMOVEDISK,
  };
private:
  uint8_t                        m_State;
  uint8_t                        m_StateHint;

  uint8_t                        m_KeyState;
  uint8_t                        m_KeyLongMask;
  uint8_t                        m_KeyAdd;

  uint8_t                        m_LastSpeed;
  uint8_t                        m_LastDirection;
  uint16_t                       m_LastDuration;

  UiTimerType                    m_Timer;
  FramebufferType                m_Framebuffer;
  FramebufferType::RowBufferType m_Row;

  void setState(enum State State)
  {
    m_State = static_cast<uint8_t>(State);
  }

  void setStateHint(enum State State)
  {
    m_StateHint = static_cast<uint8_t>(State);
  }

  enum State getStateHint() const
  {
    return static_cast<enum State>(m_StateHint);
  }

  static void formatTime(FramebufferType::RowBufferType & Buffer);

  uint8_t calculateLength();
public:
  enum State getState() const
  {
    return static_cast<enum State>(m_State);
  }

  FramebufferType &
  getFrameBuffer()
  {
    return m_Framebuffer;
  }

  void poll(uint8_t TicksPassed);

  void updateDisplay();
};



#endif /* TRAFFIC_COUNTER_UI_HPP_ */
