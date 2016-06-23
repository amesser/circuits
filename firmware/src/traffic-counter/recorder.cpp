/*
 * recorder.cpp
 *
 *  Created on: 15.05.2016
 *      Author: andi
 */
#include "recorder.hpp"


void FileRecorder::activate()
{
  if(STATE_DISABLED == getState())
  {
    setState(STATE_NODISK);
    setStateHint(STATE_RUN);
  }
}

void FileRecorder::formatDisk()
{
  const auto state = getState();

  if((STATE_NOFS == state || STATE_MOUNTED == state))
  {
    setState(STATE_FORMAT);
  }
}

void FileRecorder::openFile()
{
  if(STATE_MOUNTED  == getState())
  {
    setState(STATE_OPENING);
  }
}

void FileRecorder::start()
{
  if(STATE_READY  == getState())
  {
    setState(STATE_RUN);
  }
}

void FileRecorder::finish()
{
  setStateHint(STATE_DISABLED);
}


void FileRecorder::open(const char *name)
{
  if(STATE_OPENING == getState())
  {
    FRESULT result;

    result = f_open(&m_File, name, FA_WRITE | FA_CREATE_NEW);

    if(FR_OK == result)
    {
      setState(STATE_READY);
      sync();
    }
    else if (FR_EXIST == result)
    {
      /* nothing to do, repeat with next filename */
    }
    else
    {
      setState(STATE_DISKERROR);
    }
  }
}

uint_fast16_t FileRecorder::write(const void* buf, uint_fast16_t len)
{ /* recording */
  FRESULT result;
  UINT len_written = 0;

  if(((STATE_RUN == getState()) || (STATE_READY == getState())) &&
     STATE_ERROR > getStateHint())
  {
    result = f_write(&m_File, buf, len, &len_written);
  }
  else
  {
    result = FR_INT_ERR;
  }

  if((len_written < len) || FR_OK != result)
  {
    error(result);
  }

  return len_written;
}

void FileRecorder::sync()
{
  if(((STATE_RUN == getState()) || (STATE_READY == getState())) &&
     STATE_ERROR > getStateHint())
  {
    FRESULT result = f_sync(&m_File);

    if(FR_OK != result)
    {
      error(result);
    }
  }
}

void FileRecorder::close()
{
  if((STATE_READY == getState()) ||
     (STATE_RUN   == getState()))
  {
    setState(STATE_CLOSING);
  }
}

/** Special meaning: result = 0 means out of space */
void FileRecorder::error(FRESULT result)
{
  if(STATE_DISABLED != getState())
  {
    if (STATE_ERROR > getStateHint())
    {
      m_StateHint = STATE_ERROR + result;
    }
  }
}


void FileRecorder::poll()
{
  auto state     = getState();
  auto statehint = getStateHint();
  FRESULT result;

  if (statehint != STATE_RUN)
  {
    switch(state)
    {
    case STATE_DISABLED:
      break;
    case STATE_NODISK:
    case STATE_NOFS:
    case STATE_FORMAT:
      state = statehint;
      break;
    case STATE_MOUNTED:
    case STATE_OPENING:
      state = STATE_UNMOUNTING;
      break;
    case STATE_READY:
      state = STATE_CLOSING;
      break;
    case STATE_RUN:
      /* This case is to be handled by the subclass of FileRecorder()
       * The subclass is expected to write out all pending data
       * and then call close()
       */
      break;
    default:
      /* in all other states, statehint is ignored */
      break;
    }
  }

  switch(state)
  {
  case STATE_DISABLED:
    break;
  case STATE_NODISK:
    { /* poll for disk */
      result = f_mount(&m_Volume, "", 1);

      if(FR_OK == result)
      {
        state = STATE_MOUNTED;
      }
      else if(FR_NO_FILESYSTEM == result)
      {
        state  = STATE_NOFS;
      }
    }
    break;
  case STATE_NOFS: /* nothing to do, wait for format request */
    break;
  case STATE_FORMAT:
    { /* formatting of disk requested */

      result = f_mkfs("",0,0);

      if(FR_OK == result)
      {
        state = STATE_NODISK;
      }
      else
      {
        error(result);
      }
    }
    break;
  case STATE_MOUNTED: /* nothing to do, wait for next state */
    break;
  case STATE_OPENING: /* in this state we're trying to open the record file
                       * it s in the scope of the subclass to invoke the open() function */
    break;
  case STATE_READY: /* wait for start of recording */
    break;
  case STATE_RUN: /* In this state we're recording to the file.
                   * it is in the scope of the subclass to invoke the write() function */
    break;
  case STATE_CLOSING:
    {
      result = f_close(&m_File);

      if(FR_OK != result)
      {
        error(result);
      }

      state = STATE_UNMOUNTING;
    }
    break;
  case STATE_UNMOUNTING:
    {
      f_mount(0, "", 0);
      state = getStateHint();
    }
    break;
  default:
    if(state >= STATE_ERROR)
    { /* poll for removal of disk */
      result = f_mount(&m_Volume, "", 1);

      if((FR_OK != result) &&
         (FR_NO_FILESYSTEM != result))
      { /* disk was removed */
        statehint = state;
        state     = STATE_DISABLED;
      }
      else
      { /* in case of error state == statehint,
         * but when cancel requested: statehint == STATE_DISABLED, so
         * the polling will be terminated */
        state = statehint;
      }
    }
    break;
  }

  if (STATE_DISABLED == state &&
      STATE_DISABLED != getState())
  { /* unmount and power off disk*/
    f_mount(0, "", 0);
    disk_ioctl(0,CTRL_POWER_OFF, 0);
  }

  setState(state);
}
