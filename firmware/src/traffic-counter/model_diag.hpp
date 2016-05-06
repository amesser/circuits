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
#ifndef TRAFFIC_COUNTER_MODEL_DIAG_HPP_
#define TRAFFIC_COUNTER_MODEL_DIAG_HPP_

#include <ecpp/Datatypes.hpp>
#include <ecpp/Time.hpp>

#include "recorder.hpp"

class   DiagRecorderBase;
typedef EventRecorder<DiagRecorderBase> DiagRecorder;

class DiagEvent
{
public:
  typedef ecpp::DateTime TimestampType;

public:
  TimestampType  Timestamp;

  uint16_t       m_Idx;

  uint8_t        m_Cnt0;
  uint8_t        m_Cnt2;
  uint16_t       m_Cnt1;
  uint8_t        m_Speed;
};

class DiagRecorderBase
{
protected:
  typedef DiagEvent RecordType;
  typedef char      RecordStringType[41];

  typedef char      FilenameType[8 + 1 + 3 + 1];

private:
  union {
    RecordStringType m_RecordString;

    struct {
      FilenameType   m_FilenameBuffer;
      uint8_t        m_LastDay;
    };
  };

  DiagEvent          m_LastEvent;

  uint16_t           m_Count;

protected:
  const RecordStringType & formatRecord(const RecordType& record);

  void                     initFilename();
  const FilenameType     & nextFilename();

public:
  void startRecording();

  DiagEvent & createDiagRecord()
  {
    auto & Record = m_LastEvent;

    if (m_Count < 999)
    {
      m_Count += 1;
    }
    else
    {
      m_Count = 0;
    }

    Record.m_Idx = m_Count;
    return Record;
  }
};


#endif /* TRAFFIC_COUNTER_MODEL_DIAG_HPP_ */
