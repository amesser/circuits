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


class DiagEvent
{
public:
  uint16_t       Id;
  uint8_t        Ticks1s;
  uint8_t        Ticks256Hz;
  uint8_t        NSig;
  uint16_t       NRef;
  int8_t         NPhase;
};

struct __attribute__((packed)) DiagRecord0Fmt
{
  char Tag[3];
  char Sep0;
  char Id[3];
  char Sep1;
  char Time[8];
  char Sep2;
  char Ticks256Hz[3];
  char Sep3;
  char NSig[2];
  char Sep4;
  char NRef[5];
  char Sep5;
  char NFCnt2[2];
  char Sep6;
  char NFCnt1[5];
  char Sep7;
  char Phase;
  char Newline;
};

class DiagRecorder : public FileRecorder
{
protected:
  typedef DiagEvent RecordType;
  typedef char      RecordStringType[sizeof(DiagRecord0Fmt)];

private:
  Ringbuffer<RecordType, 16> RecordBuffer;

  union {
    DiagRecord0Fmt   LogString0;

    struct {
      FilenameType   FilenameBuffer;
      uint8_t        LastDay;
    };
  };

  uint16_t     Count;

  void initFilename(void);
  bool nextFilename(void);
protected:
  void formatRecord(const RecordType& record);

public:
  char     LastPhase;
  uint16_t LastFreq;

  void activate(void);
  void poll(void);

  const RecordType & getLastRecord() const
  {
    return RecordBuffer.back();
  }

  RecordType* createRecord()
  {
    RecordType* p = 0;

    if(STATE_RUN == getStateHint() &&
       STATE_RUN == getState())
    {
      p = RecordBuffer.getInsertElem();

      if(0 != p)
      {
        p->Id = Count;
      }

      if (Count < 999)
      {
        Count += 1;
      }
      else
      {
        Count = 0;
      }
    }

    return p;
  }

  void storeRecord(RecordType* p)
  {
    if(p == RecordBuffer.getInsertElem())
    {
      RecordBuffer.pushForced();
    }
  }
};


#endif /* TRAFFIC_COUNTER_MODEL_DIAG_HPP_ */
