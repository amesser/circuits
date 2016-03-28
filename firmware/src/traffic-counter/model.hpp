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
#ifndef TRAFFIC_COUNTER_MODEL_HPP_
#define TRAFFIC_COUNTER_MODEL_HPP_

#include <ecpp/Datatypes.hpp>
#include <ecpp/Time.hpp>

#include "recorder.hpp"

class   TrafficRecorderBase;
typedef EventRecorder<TrafficRecorderBase> TrafficRecorder;

class TcEvent
{
public:
  typedef ecpp::DateTime TimestampType;

  enum Type
  {
    TYPE_TRAFFIC     = 0,
    TYPE_PARAMETERS  = 1,
  };

protected:
  uint8_t            m_Type;

  union
  {
    struct {
      uint16_t       Index;
      uint8_t        Speed;
      uint8_t        Direction;
      uint16_t       Duration;

      TimestampType  Timestamp;
    } Traffic;

    struct {
      int8_t         LengthCorrectionL;
      int8_t         LengthCorrectionR;
    } Parameters;
  } m_Data;

  void create(uint8_t Type)
  {
    m_Type = Type;
  }
public:
  friend class TrafficRecorderBase;
};


class TrafficRecord : public TcEvent
{
public:
  enum Direction
  {
    UNKOWN = 0,
    LEFT   = 1,
    RIGHT  = 2,
  };

  void setTime(const TimestampType & Timestamp)
  {
    m_Data.Traffic.Timestamp = Timestamp;
  }

  void setSpeed(uint8_t Speed, enum Direction Direction, uint16_t Duration)
  {
    m_Data.Traffic.Speed     = Speed;
    m_Data.Traffic.Direction = Direction;
    m_Data.Traffic.Duration  = Duration;
  }

  uint16_t getIndex() const
  {
    return m_Data.Traffic.Index;
  }

  uint8_t getSpeed() const
  {
    return m_Data.Traffic.Speed;
  }

  uint16_t getDuration() const
  {
    return m_Data.Traffic.Duration;
  }

  enum Direction getDirection() const
  {
    return static_cast<enum Direction>(m_Data.Traffic.Direction);
  }

  const TimestampType &
  getTimestamp() const
  {
    return m_Data.Traffic.Timestamp;
  }

  uint8_t getLength() const;

  friend class TrafficRecorderBase;
};

class ParametersRecord : public TcEvent
{
public:
  friend class TrafficRecorderBase;
};

class TrafficRecorderBase
{
protected:
  typedef TcEvent RecordType;
  typedef char    RecordStringType[41];

  typedef char    FilenameType[8 + 1 + 3 + 1];

private:
  union {
    RecordStringType m_RecordString;

    struct {
      FilenameType   m_FilenameBuffer;
      uint8_t        m_LastDay;
    };
  };

  TrafficRecord      m_LastTrafficEvent;

  uint16_t           m_Count;

protected:
  const RecordStringType & formatRecord(const RecordType& record);

  void                     initFilename();
  const FilenameType     & nextFilename();

public:
  const TrafficRecord & getLastTrafficRecord() const
  {
    return m_LastTrafficEvent;
  }

  void startRecording();

  TrafficRecord & createTrafficRecord()
  {
    auto & Record = m_LastTrafficEvent;

    Record.m_Type = Record.TYPE_TRAFFIC;

    if (m_Count < 999)
    {
      m_Count += 1;
    }
    else
    {
      m_Count = 0;
    }

    Record.m_Data.Traffic.Index = m_Count;

    return Record;
  }
};


#endif /* TRAFFIC_COUNTER_MODEL_HPP_ */
