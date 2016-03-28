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
#ifndef TRAFFIC_COUNTER_RECORDER_HPP_
#define TRAFFIC_COUNTER_RECORDER_HPP_

#include <stdint.h>
#include <string.h>

#include <ecpp/Ringbuffer.hpp>

#include "diskio.h"
#include "ff.h"

using namespace ecpp;

template<typename MODEL>
class EventRecorder : public MODEL
{
public:
  typedef typename MODEL::RecordType RecordType;

  enum State
  {
    STATE_DISABLED = 0,
    STATE_NODISK,
    STATE_NOFS,
    STATE_FORMAT,
    STATE_MOUNTED,
    STATE_OPENING,
    STATE_READY,
    STATE_RUN,
    STATE_CLOSING,
    STATE_UNMOUNTING,
    STATE_DISKFULL,
    STATE_DISKERROR,

    /** first error state */
    STATE_ERROR = STATE_DISKFULL
  };

private:
  uint8_t  m_State;
  uint8_t  m_StateHint;

  FATFS    m_Volume;
  FIL      m_File;

  /** Small ringbuffer to buffer events */
  Ringbuffer<RecordType, 8> m_EventRing;

  void setState(enum State State)
  {
    m_State = State;
  }

  enum State getStateHint()
  {
    return static_cast<enum State>(m_StateHint);
  }

  void setStateHint(enum State State)
  {
    m_StateHint = State;
  }
public:

  enum State getState() const
  {
    return static_cast<enum State>(m_State);
  }

  enum State getErrorState()
  {
    enum State State;

    State = getState();

    if(State == STATE_DISABLED)
    {
      State = getStateHint();
    }

    return State;
  }

  void poll();

  bool isBufferFull() const
  {
    return (m_EventRing.getCount() >= m_EventRing.getSize());
  }

  RecordType &
  getFirstRingEvent()
  {
    return m_EventRing.getBuffer()[0];
  }

  void recordEventForced(const RecordType & Event)
  {
    auto & Ring = m_EventRing;

    if(Ring.getCount() <= Ring.getSize())
    {
      Ring.pushForced(Event);
    }
  }

  void recordEvent(const RecordType & Event)
  {
    if(STATE_RUN == getState() &&
       STATE_RUN == getStateHint())
    {
      recordEventForced(Event);
    }
  }


  void activate()
  {
    if(STATE_DISABLED == getState())
    {
      m_EventRing.reset();
      setState(STATE_NODISK);
    }
  }

  void close();

  void formatDisk()
  {
    if(STATE_NOFS    == getState() ||
       STATE_MOUNTED == getState())
    {
      setState(STATE_FORMAT);
    }
  }

  void openLogfile()
  {
    if(STATE_MOUNTED == getState())
    {
      setState(STATE_OPENING);
    }
  }

  void startRecording()
  {
    if(STATE_READY == getState())
    {
      MODEL::startRecording();

      setStateHint(STATE_RUN);
      setState(STATE_RUN);
    }
  }
};

template<typename MODEL>
void
EventRecorder<MODEL>::close()
{
  auto State = getState();

  switch(State)
  {
  case STATE_DISABLED:
    break;
  case STATE_NODISK:
  case STATE_NOFS:
  case STATE_FORMAT:
    setState(STATE_DISABLED);
    setStateHint(STATE_DISABLED);
    break;
  case STATE_MOUNTED:
  case STATE_OPENING:
    setState(STATE_UNMOUNTING);
    setStateHint(STATE_DISABLED);
    break;
  case STATE_READY:
    setState(STATE_CLOSING);
    setStateHint(STATE_DISABLED);
    break;
  case STATE_RUN:
  case STATE_CLOSING:
  case STATE_UNMOUNTING:
    setStateHint(STATE_DISABLED);
    break;
  case STATE_DISKFULL:
  case STATE_DISKERROR:
    setState(STATE_DISABLED);
    break;
  }
}

template<typename MODEL>
void
EventRecorder<MODEL>::poll()
{
  auto State = getState();
  FRESULT result;

  switch(State)
  {
  case STATE_DISABLED:
    { /* nothing to do */
      /* ensure that power of sd is switched to off */
      disk_ioctl(0,CTRL_POWER_OFF, 0);
    }
    break;
  case STATE_NODISK:
    { /* poll for disk */
      result = f_mount(&m_Volume, "", 1);

      if(FR_OK == result)
      {
        State = STATE_MOUNTED;
        MODEL::initFilename();
      }
      else if(FR_NO_FILESYSTEM == result)
      {
        State  = STATE_NOFS;
      }
    }
    break;
  case STATE_NOFS:
    { /* nothing to do, wait for next state */

    }
    break;
  case STATE_FORMAT:
    { /* formatting of disk requested */

      /* make sure fs is not mounted */
      f_mount(0, "", 0);

      result = f_mkfs("",0,0);

      if(FR_OK == result)
      {
        result = f_mount(&m_Volume, "", 1);
      }

      if(FR_OK == result)
      {
        State = STATE_MOUNTED;
        MODEL::initFilename();
      }
      else
      {
        State = STATE_DISKERROR;
      }
    }
    break;
  case STATE_MOUNTED:
    { /* nothing to do, wait for application to request opening of files */

    }
    break;
  case STATE_OPENING:
    { /* Try to open a file */
      auto & Filename = MODEL::nextFilename();

      if(Filename[0] != 0)
      {
        result = f_open(&m_File, Filename, FA_WRITE | FA_CREATE_NEW);

        if(FR_OK == result)
        {
          State = STATE_READY;
          f_sync(&m_File);
        }
        else if (FR_EXIST == result)
        {
          /* nothing to do, repeat with next filename */
        }
        else
        {
          State = STATE_UNMOUNTING;
          setStateHint(STATE_DISKERROR);
        }
      }
      else
      {
        State = STATE_UNMOUNTING;
        setStateHint(STATE_DISKFULL);
      }
    }
    break;
  case STATE_READY:
    { /* waiting for user to continue */

    }
    break;
  case STATE_RUN:
    { /* recording */
      if(m_EventRing.getCount() > 0)
      {
        auto & buffer = MODEL::formatRecord(m_EventRing.front());
        UINT lentowrite, lenwritten;

        lentowrite = strnlen(buffer, sizeof(buffer));
        result = f_write(&m_File, buffer,lentowrite, &lenwritten);

        if (FR_OK == result)
        {
          if(lentowrite == lenwritten)
          {
            f_sync(&m_File);
            m_EventRing.popForced();
          }
          else
          {
            State = STATE_CLOSING;
            setStateHint(STATE_DISKFULL);
          }
        }
        else
        {
          State = STATE_CLOSING;
          setStateHint(STATE_DISKERROR);
        }
      }
      else if (STATE_DISABLED == getStateHint())
      {
        State = STATE_CLOSING;
      }
    }
    break;
  case STATE_CLOSING:
    {
      f_close(&m_File);
      State = STATE_UNMOUNTING;
    }
    break;
  case STATE_UNMOUNTING:
    {
      f_mount(0, "", 0);
      State = getStateHint();
    }
    break;
  case STATE_DISKFULL:
  case STATE_DISKERROR:
    { /* poll for removal of disk */
      result = f_mount(&m_Volume, "", 1);

      if(FR_OK == result)
      {
        f_mount(0, "", 0);
      }
      else if(FR_NO_FILESYSTEM != result)
      {
        setStateHint(State);
        State = STATE_DISABLED;
      }
    }
    break;
  }

  setState(State);
}

#endif /* TRAFFIC_COUNTER_RECORDER_HPP_ */
