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

#include "model_diag.hpp"
#include "app.hpp"
#include "bsp.hpp"

using namespace ecpp;

static constexpr FlashVariable<char,12> s_Filename  PROGMEM = "YYMMDDXX.csv";
static constexpr FlashVariable<char,sizeof(DiagRecord0Fmt)> s_DiagRecordFormat    PROGMEM =  "DI0;XXX;HH:MM:SS;XXX;XX;XXXXX;XX;XXXXX;X\n";

void DiagRecorder::activate(void)
{
  if(STATE_DISABLED == getState())
  {
    RecordBuffer.reset();
    Count   = 0;
    LastDay = 0xFF;
    FileRecorder::activate();
  }
}

void DiagRecorder::poll(void)
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

      Count = 0;
    }
    break;
  case STATE_RUN:
    {
      if(STATE_ERROR <= getStateHint())
      {
        close();
      }
      else if(RecordBuffer.getCount() > 0)
      {
        formatRecord(RecordBuffer.front());
        RecordBuffer.popForced();

        write(&LogString0, sizeof(LogString0));

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
DiagRecorder::initFilename(void)
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
DiagRecorder::nextFilename(void)
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
DiagRecorder::formatRecord(const RecordType & Record)
{
  auto & Buffer = LogString0;
  auto & bsp = Bsp::getInstance();
  auto time = bsp.getClock().getTime();

  uint8_t tmp;

  tmp = Record.Ticks1s - bsp.getClockTicksHandles1s();

  /* we need to adjust the time according the record time
   * and the ticks handled in the clock */
  if (tmp < 0x80)
  {
    time.sub({0,0, (uint8_t)(- tmp)});
  }
  else
  {
    time.add({0,0,tmp});
  }

  /* The following format will be used for an event
   * DI0;XXX;HH:MM:SS;XXX;XX;XXX;XXXXX */
  s_DiagRecordFormat.read(Buffer);

  Buffer.Newline = '\n';

  String::formatUnsigned(Buffer.Id, 3, Record.Id);

  formatTime(time, Buffer.Time);

  String::formatUnsigned(Buffer.Ticks256Hz, 3, Record.Ticks256Hz);
  String::formatUnsigned(Buffer.NSig, 2, Record.NSig);
  String::formatUnsigned(Buffer.NRef, 5, Record.NRef);

  /* consistent read of ref frequency state */
  uint16_t cnt1;
  uint8_t  cnt2;
  do
  {
    cnt2 = bsp.getFreqRefCnt2();
    cnt1 = bsp.getFreqRefCnt1();
  } while(cnt2 != bsp.getFreqRefCnt2());

  String::formatUnsigned(Buffer.NFCnt2, 2, cnt2);
  String::formatUnsigned(Buffer.NFCnt1, 5, cnt1);

  if (Record.NPhase > 0)
  {
    Buffer.Phase = '+';
  }
  else if (Record.NPhase < 0)
  {
    Buffer.Phase = '-';
  }

  LastPhase = Buffer.Phase;
  LastFreq  = bsp.calcFrequency(Record.NSig, Record.NRef);
}


