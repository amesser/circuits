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

class FileRecorder
{
public:
  typedef char FilenameType[8 + 1 + 3 + 1];

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

  void setState(enum State State)
  {
    m_State = State;
  }


  void setStateHint(enum State State)
  {
    m_StateHint = State;
  }

  void enterState(enum State state);

protected:
  enum State getStateHint()
  {
    return static_cast<enum State>(m_StateHint);
  }

  void          open(const char *name);
  uint_fast16_t write(const void* buf, uint_fast16_t len);
  void          sync(void);
  void          close(void);
  void          error(FRESULT result);
public:

  uint_fast32_t getFilesize() const
  {
    if(STATE_READY <= getState() &&
       STATE_CLOSING >= getState())
    {
      return m_File.fsize;
    }
    else
    {
      return 0;
    }
  }
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


  void activate();
  void formatDisk();
  void openFile();
  void start();
  void finish();


};



#endif /* TRAFFIC_COUNTER_RECORDER_HPP_ */
