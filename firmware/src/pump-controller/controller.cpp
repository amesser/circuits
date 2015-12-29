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
#include "controller.hpp"

#include "bsp.hpp"
#include "ui.hpp"

#define MAXWETTINGDURATION (60 * 60 * 4)

static EEVariable<HumidityController::Parameters> s_ApplicationParameters EEMEM;

HumidityController Application;

void HumidityController::init()
{
  s_ApplicationParameters.read(m_Parameters);

  if(m_Parameters.m_WettingPeriod > MAXWETTINGDURATION)
  {
    m_Parameters.m_WettingPeriod = 0;
  }

  m_Parameters.m_WettingTime.incSecond(0);
  m_Parameters.m_WettingTime.setWeekDay(m_Time.getWeekDay());

  m_Parameters.m_WettingTimeout.incSecond(0);

  if(m_Time - m_Parameters.m_WettingTime > (60UL * 60UL * 24UL))
    m_Parameters.m_WettingTime.incHour(24);
}

void HumidityController::eventSecondTick()
{
  m_Time.incSecond(1);

  if(bsp.getRealOutputState() & bsp.OUTPUT_CHA)
    m_ChannelA.TimeEnabled.handleSecondsPassed(1);
}

void HumidityController::changeTime(int8_t dMinutes)
{
  auto & Time = m_Time;;

  if(dMinutes > 0)
  {
    Time.setSecond(0);
    Time.incMinute(dMinutes);
  }
  else if(dMinutes < 0)
  {
    Time.setSecond(0);
    Time.decMinute(-dMinutes);
  }
  else
  {
    auto & LastWettingTime = m_ChannelA.TimeLastWetting;

    LastWettingTime = Time;

    LastWettingTime.decWeekday(m_Parameters.m_WettingTimeout.getWeekDay());
    LastWettingTime.decHour(m_Parameters.m_WettingTimeout.getHour());

    auto & WettingTime = m_Parameters.m_WettingTime;
    WettingTime.setWeekDay(Time.getWeekDay());

    if((WettingTime - Time) > (24UL * 60UL * 60UL))
      WettingTime.incHour(24);

  }
}

void HumidityController::changeWettingTime(int8_t dMinutes)
{
  auto & Time = m_Parameters.m_WettingTime;

  if(dMinutes > 0)
  {
    Time.setSecond(0);
    Time.incMinute(dMinutes);
  }
  else if(dMinutes < 0)
  {
    Time.setSecond(0);
    Time.decMinute(-dMinutes);
  }
  else
  { /* store parameters remanent */
    s_ApplicationParameters = m_Parameters;
  }
}

void HumidityController::changeWettingTimeout(int8_t dMinutes)
{
  auto & Time = m_Parameters.m_WettingTimeout;

  if(dMinutes > 0)
  {
    Time.setSecond(0);
    Time.incMinute(dMinutes);
  }
  else if(dMinutes < 0)
  {
    Time.setSecond(0);
    Time.decMinute(-dMinutes);
  }
  else
  { /* store parameters remanent */
    s_ApplicationParameters = m_Parameters;
  }
}

void HumidityController::changeWettingPeriod(int8_t delta)
{

  if(delta > 0)
  {
    if ((m_Parameters.m_WettingPeriod + delta) > MAXWETTINGDURATION)
      m_Parameters.m_WettingPeriod = MAXWETTINGDURATION;
    else
      m_Parameters.m_WettingPeriod += delta;
  }
  else if (delta < 0)
  {
    delta = -delta;

    if(m_Parameters.m_WettingPeriod > (uint8_t)delta)
      m_Parameters.m_WettingPeriod -= delta;
    else
      m_Parameters.m_WettingPeriod = 0;
  }
  else
  { /* store parameters remanent */
    s_ApplicationParameters = m_Parameters;
  }
}

void HumidityController::cycle()
{
  auto & Time = getTime();
  auto & WettingTime = m_Parameters.m_WettingTime;

  {
    auto & channel = m_ChannelA;
    uint8_t state  = channel.state;

    if(!ui.isEditing() && state == CHANNEL_IDLE)
    {
      if((Time - WettingTime) < (5 * 60))
      {
        /* we should start wetting the plants */
        WettingTime.incHour(24);

        auto x = Time - channel.TimeLastWetting;

        if(x > m_Parameters.m_WettingTimeout.getMonotonic())
        {
          state = CHANNEL_WETTING;
          channel.TimeEnabled.startSeconds(0xFFFF);

          bsp.enableOutputs(bsp.OUTPUT_CHA);
          bsp.enableOutputs(bsp.OUTPUT_CHB); /* only for first implementation */
        }
      }
    }

    if(state == CHANNEL_WETTING)
    {
      if (0 == (bsp.getOutputState() & bsp.OUTPUT_CHA))
      {
        /* for some reason the wetting was aborted */
        state = CHANNEL_IDLE;
      }
      else if (channel.TimeEnabled.getElapsedSeconds(0xFFFF) >= m_Parameters.m_WettingPeriod)
      {
        bsp.disableOutputs(bsp.OUTPUT_CHA);
        bsp.disableOutputs(bsp.OUTPUT_CHB); /* only for first implementation */
        channel.TimeLastWetting = Time;
        state = CHANNEL_IDLE;
      }
    }

    channel.state = state;
  }
}


int main()
{
  bsp.initialize();
  Application.init();

  while(1)
  {
    Sys_AVR8::enableSleep();

    bsp.cycle();
    ui.cycle();
    Application.cycle();
  }
}



