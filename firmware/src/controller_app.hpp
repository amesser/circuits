/*
 * controller_app.hpp
 *
 *  Created on: 06.06.2015
 *      Author: andi
 */

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
    SimpleTimer<uint16_t> TimeEnabled;
    WeekTime              TimeLastWetting;
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
  uint16_t           getActualWettingTimeChA() const { return m_ChannelA.TimeEnabled.getElapsedTime(0xFFFF);}

  void init();
  void cycle();
};

extern HumidityController Application;

#endif /* FIRMWARE_SRC_CONTROLLER_APP_HPP_ */
