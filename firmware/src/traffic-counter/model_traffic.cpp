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
#include "ecpp/Target.hpp"
#include "ecpp/String.hpp"

#include "model_traffic.hpp"
#include "app.hpp"
#include "bsp.hpp"

using namespace ecpp;

static constexpr FlashVariable<char,12> s_Filename  PROGMEM = "YYMMDDXX.tc";
static constexpr FlashVariable<char,3>  s_Direction PROGMEM = "U<>";
static constexpr FlashVariable<char,40> s_TrafficRecordFormat    PROGMEM =  "TC0;XXX;YYYY-MM-DDTHH:MM:SSZ;X;XXX;XXXX\n";
static constexpr FlashVariable<char,40> s_ParametersRecordFormat PROGMEM =  "TC1;XX;XX;                             \n";

uint8_t
TrafficRecord::getLength() const
{
  const uint8_t  Speed       = getSpeed();
  const uint16_t Duration1ms = getDuration();

  int8_t Length1m;
  uint32_t Calculate;

  //Length1m = ((uint32_t)Duration1ms * Speed) / 3600;

  Calculate = Duration1ms;
  Calculate = Calculate * Speed;
  Calculate = Calculate / 3600;

  if(Calculate > 64)
  {
    Length1m = 64;
  }
  else
  {
    int8_t Correction;

    if (getDirection() == LEFT)
    {
      Correction = g_Parameters.LengthCorrectionL;
    }
    else if (getDirection() == RIGHT)
    {
      Correction = g_Parameters.LengthCorrectionR;
    }
    else
    {
      Correction = 0;
    }

    Length1m = static_cast<int8_t>(Calculate) + Correction;

    if (Length1m < 0)
    {
      Length1m = 0;
    }
  }

  return Length1m;
}

void TrafficRecorder::activate()
{
  if(STATE_DISABLED == getState())
  {
    RecordBuffer.reset();
    Count   = 0;
    LastDay = 0xFF;
    FileRecorder::activate();
  }
}

void TrafficRecorder::poll(void)
{
  FileRecorder::poll();

  switch(getState())
  {
  case STATE_OPENING:
    {
      if (nextFilename())
      {
        open(FilenameBuffer);
      }
      else
      {
        error(FR_DENIED);
      }
    }
    break;
  case STATE_RUN:
    {
      if(RecordBuffer.getCount() > 0)
      {
        formatRecord(RecordBuffer.front());
        RecordBuffer.popForced();

        write(&RecordString, sizeof(RecordString));

        if(RecordBuffer.getCount() == 0)
        {
          sync();
        }
      }
      else if(STATE_RUN != getStateHint())
      {
        close();
      }
    }
    break;
  default:
    break;
  }
}

void
TrafficRecorder::initFilename()
{
  auto & Clock = Bsp::getInstance().getClock();
  FilenameType & Buffer = FilenameBuffer;

  s_Filename.read(Buffer);

  LastDay = Clock.getDay();

  String::convertToDecimal(&(Buffer[0]), 2, Clock.getYear());
  String::convertToDecimal(&(Buffer[2]), 2, Clock.getMonth() + 1);
  String::convertToDecimal(&(Buffer[4]), 2, LastDay + 1);
}

bool
TrafficRecorder::nextFilename()
{
  auto & Clock = Bsp::getInstance().getClock();
  FilenameType & Buffer = FilenameBuffer;

  if(Clock.getDay() != LastDay)
  {
    initFilename();
  }

  if (Buffer[6] == 'X')
  {
    Buffer[6] = '0';
    Buffer[7] = '0';
  }
  else if (Buffer[7] == '9')
  {
    if (Buffer[6] == '9')
    {
      Buffer[0] = 0;
    }
    else
    {
      Buffer[6] += 1;
      Buffer[7]  = '0';
    }
  }
  else
  {
    Buffer[7] += 1;
  }

  return Buffer[0] != 0;
}

void
TrafficRecorder::start()
{
  if(STATE_READY == getState())
  {
    auto & record = RecordBuffer.back();

    record.m_Type = record.TYPE_PARAMETERS;
    record.m_Data.Parameters.LengthCorrectionL = g_Parameters.LengthCorrectionL;
    record.m_Data.Parameters.LengthCorrectionR = g_Parameters.LengthCorrectionR;

    RecordBuffer.pushForced();

    FileRecorder::start();
  }

}

void
TrafficRecorder::formatRecord(const RecordType & Record)
{
  RecordStringType & Buffer = RecordString;


  if(Record.m_Type == Record.TYPE_TRAFFIC)
  {
    auto & Traffic = reinterpret_cast<const TrafficRecord &>(Record);
    /* The following format will be used for an event
     * TC0;INDEX;YYYY-MM-DDTHH:MM:SSZ;DIRECTION;SPEED;DURATION; */

    s_TrafficRecordFormat.read(Buffer);

    String::formatUnsigned(&(Buffer[4]), 3, Traffic.getIndex());

    formatDateTime(Traffic.getTimestamp(), *reinterpret_cast<char(*)[19]>(&(Buffer[8])));

    Buffer[29] = s_Direction[Traffic.getDirection()];

    uint32_t duration = Traffic.getDuration();

    duration = (duration * 25 + 32) / 64;

    String::formatUnsigned(&(Buffer[31]), 3,  Traffic.getSpeed());
    String::formatUnsigned(&(Buffer[35]), 4, (uint16_t)duration);

  }
  else if (Record.m_Type == Record.TYPE_PARAMETERS)
  {
    auto & Parameters = Record.m_Data.Parameters;
    /* The following format will be used for an event
     * TC1;ADJLENGTHL;ADJLENGTHR;*/

    s_ParametersRecordFormat.read(Buffer);

    String::formatSigned(&(Buffer[4]), 2,  Parameters.LengthCorrectionL);
    String::formatSigned(&(Buffer[7]), 2,  Parameters.LengthCorrectionR);
  }
  else
  {
    s_ParametersRecordFormat.read(Buffer);
    memset(&(Buffer[0]),' ', 39);
  }
}

