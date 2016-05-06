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
static constexpr FlashVariable<char,40> s_DiagRecordFormat    PROGMEM =  "DI0;XXX;YYYY-MM-DDTHH:MM:SSZ;XXX;XXX;  \n";

void
DiagRecorderBase::initFilename()
{
  auto & Clock = Bsp::getInstance().getClock();
  FilenameType & Buffer = m_FilenameBuffer;

  s_Filename.read(Buffer);

  m_LastDay = Clock.getDay();

  String::convertToDecimal(&(Buffer[0]), 2, Clock.getYear());
  String::convertToDecimal(&(Buffer[2]), 2, Clock.getMonth());
  String::convertToDecimal(&(Buffer[4]), 2, m_LastDay);
}

const DiagRecorderBase::FilenameType &
DiagRecorderBase::nextFilename()
{
  auto & Clock = Bsp::getInstance().getClock();
  FilenameType & Buffer = m_FilenameBuffer;

  if(Clock.getDay() != m_LastDay)
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

  return Buffer;
}

void
DiagRecorderBase::startRecording()
{
  m_Count = 0;
}

const DiagRecorderBase::RecordStringType &
DiagRecorderBase::formatRecord(const RecordType & Record)
{
  RecordStringType & Buffer = m_RecordString;

  /* The following format will be used for an event
   * DI0;XXX; XXX;XXXXX;HH:MM:SSZ;XXX;XXX;
   * TC0;XXX;YYYY-MM-DDTHH:MM:SSZ;X;XXX;XXXX */
  s_DiagRecordFormat.read(Buffer);

  String::formatUnsigned(&(Buffer[4]), 3, Record.m_Idx);

  Record.Timestamp.formatUTCTime(&(Buffer[8]));

  Buffer[8] = ' ';
  String::formatUnsigned(&(Buffer[9]),  3, Record.m_Cnt0);
  Buffer[12] = ';';
  String::formatUnsigned(&(Buffer[13]), 5, Record.m_Cnt1);
  Buffer[18] = ';';

  String::formatUnsigned(&(Buffer[29]), 3, Record.m_Cnt2);
  String::formatUnsigned(&(Buffer[33]), 3, Record.m_Speed);

  return Buffer;
}

