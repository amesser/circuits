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
#ifndef TRAFFIC_COUNTER_BSP_HPP_
#define TRAFFIC_COUNTER_BSP_HPP_

#include <stdint.h>
#include <ecpp/Time.hpp>
#include <ecpp/Peripherals/Keyboard.hpp>

#include "app.hpp"
#include "ui.hpp"

using namespace ecpp;

ISR(TIMER1_CAPT_vect);
ISR(TIMER1_COMPB_vect);
ISR(TIMER2_COMPA_vect);

class Bsp
{
public:
  typedef ecpp::DateTime<ecpp::FixedCenturyDate<20>, ecpp::Time > DateTimeType;
  typedef ecpp::Clock<DateTimeType>      ClockType;
  typedef uint8_t                        KeyStateType;
  /* TODO: 256Hz != 4ms */
  typedef SimpleTimer<uint8_t,4>         KeyTimerType;

  enum
  {
    FREQCNT_IDLE = 0,
    FREQCNT_RUN  = 1,
  };
private:
  ClockType             m_Clock;

  volatile uint8_t      Ticks1s;
  uint8_t               Ticks1sHandled;

  uint8_t               m_TicksHandled256Hz;
  uint8_t               m_TicksFat256Hz;

  uint8_t               FreqRefTCNT2;
  uint16_t              FreqRefTCNT1;

  volatile uint8_t      FreqRefCnt2;
  volatile uint16_t     FreqRefCnt1;

  uint16_t                   CalibrateTimer1Cnt;

  KeyDebouncer<KeyStateType> m_KeyDebouncer;
  KeyTimerType               m_KeyTimer;
  uint8_t                    m_BatteryVoltage;



  uint8_t                    m_FreqCntState;
  uint8_t                    m_FreqTCNT0Start;
  uint16_t                   m_FreqICR1Start;
  int8_t                     m_FreqPhaseCnt;

  static Bsp s_Instance;
public:
  static Bsp &
  getInstance()
  {
    return s_Instance;
  }

  uint8_t getFreqCntState() const {return m_FreqCntState;}

  uint8_t getTicks1s() const
  {
    return Ticks1s;
  }

  uint8_t getClockTicksHandles1s() const
  {
    return Ticks1sHandled;
  }

  enum Key
  {
    KEY_NEXT = 0x01,
    KEY_OK   = 0x02,
  };

  KeyStateType getKeyState()
  {
    return m_KeyDebouncer.getKeyState();
  }

  ClockType & getClock()
  {
    return m_Clock;
  }

  uint8_t getCal() const;

  void setDate(const DateTimeType::DateType & Date);
  void setTime(const DateTimeType::TimeType & Time);

  void init(void);

  uint8_t poll();

  void enableRadar(void);
  void disableRadar(void);

  uint16_t calcFrequency(uint8_t sigcnt, uint16_t refcnt);

  void updateFrameBuffer(Ui::FramebufferType& FrameBuffer);

  uint16_t getFreqRefCnt1() const {return FreqRefCnt1;}
  uint8_t  getFreqRefCnt2() const {return FreqRefCnt2;}


  friend void TIMER1_CAPT_vect(void);
  friend void TIMER1_COMPB_vect(void);
  friend void TIMER2_COMPA_vect(void);
};


#endif /* TRAFFIC_COUNTER_BSP_HPP_ */
