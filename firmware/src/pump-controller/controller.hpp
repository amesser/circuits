/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of Soil Moisture Pump Controllerr firmwares.
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
#ifndef CONTROLLER_APP_HPP_
#define CONTROLLER_APP_HPP_

#include "ecpp/Datatypes.hpp"
#include "ecpp/Time.hpp"

using namespace ecpp;


class HumidityController
{
public:
  struct Parameters
  {
    WeekTime   m_WettingTime;
    WeekTime   m_WettingTimeout;
    uint16_t   m_WettingPeriod;
  };
private:
  enum ChannelState
  {
    CHANNEL_IDLE    = 0,
    CHANNEL_WETTING = 1,
  };

  struct Channel
  {
    uint8_t state;
    SimpleTimer<uint16_t,1000> TimeEnabled;
    WeekTime                   TimeLastWetting;
  };

  Parameters       m_Parameters;
  WeekTime         m_Time;
  Channel          m_ChannelA;

public:
  const WeekTime & getTime() const {return m_Time;};
  void             changeTime(int8_t dMinutes);

  void eventSecondTick();

  void changeWettingTime(int8_t delta);
  void changeWettingTimeout(int8_t dMinutes);
  void changeWettingPeriod(int8_t delta);

  const WeekTime &   getWettingTime()    const {return m_Parameters.m_WettingTime;}
  const WeekTime &   getWettingTimeout() const {return m_Parameters.m_WettingTimeout;}
  const uint16_t     getWettingPeriod() const {return m_Parameters.m_WettingPeriod;}

  const WeekTime &   getLastWettingTimeChA() const {return m_ChannelA.TimeLastWetting;}
  bool               isWettingChA() const {return m_ChannelA.state == CHANNEL_WETTING;}
  uint16_t           getActualWettingTimeChA() const { return m_ChannelA.TimeEnabled.getElapsedSeconds((uint32_t)0xFFFF);}

  void init();
  void cycle();
};

extern HumidityController Application;

#endif /* FIRMWARE_SRC_CONTROLLER_APP_HPP_ */
