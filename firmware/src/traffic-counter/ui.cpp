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
#include <ecpp/Target.hpp>
#include <ecpp/String.hpp>

#include "app.hpp"
#include "bsp.hpp"
#include "ui.hpp"

uint16_t
Ui::calculateLength()
{
  const uint16_t Duration1ms = m_LastDuration;

#if defined(DIAGFIRMWARE)
  return Duration1ms;
#else
  int8_t Length1m;
  const uint8_t  Speed        = m_LastSpeed;

  uint32_t Calculate;

  //Length1m = ((uint32_t)Duration1ms * Speed) / 3600;

  Calculate = Duration1ms;
  Calculate = (Calculate * 125 + 16) / 32;
  Calculate = Calculate * Speed;
  Calculate = Calculate / 3600;

  if(Calculate > 64)
  {
    Length1m = 64;
  }
  else
  {
    int8_t Correction;

    if (m_LastDirection == TrafficRecord::LEFT)
    {
      Correction = g_Parameters.LengthCorrectionL;
    }
    else if (m_LastDirection == TrafficRecord::RIGHT)
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
#endif
}

void Ui::poll(uint8_t Ticks)
{
  auto & Bsp = Bsp::getInstance();
  enum State State = getState();
  uint8_t KeyState = Bsp.getKeyState();

  const auto KEY_OK   = Bsp.KEY_OK;
  const auto KEY_NEXT = Bsp.KEY_NEXT;

  auto & Timer = m_Timer;

  uint8_t KeyDown, KeyLong;
  uint8_t KeyAdd;

  Timer.handleTicksPassed(Ticks);

  /* check which keys have been pressed since last check */
  KeyDown = KeyState & (~m_KeyState);
  m_KeyState = KeyState;

  /* handle long key presses by simulating
   * additional key presses */
  KeyLong = KeyDown;

  if(0 != KeyDown)
  { /* restart ui timeout on keypress */
    Timer.startMilliseconds(500);
    KeyAdd = 1;
  }
  else if(0 != KeyState)
  { /* key has been hold down for some
     * time, simulate key press */
    KeyAdd = m_KeyAdd;

    if(Timer.hasTimedOut())
    {
      KeyLong = KeyState & m_KeyLongMask;
      Timer.startMilliseconds(50);

      if(KeyAdd < 10)
      {
        KeyAdd += 1;
      }
    }
  }
  else
  {
    m_KeyLongMask = 0xFF;
    KeyAdd = 1;
  }

  /* This var is used to speed up stepping when holding a key
   * a key */
  m_KeyAdd = KeyAdd;
  if(KeyAdd < 10)
  {
    KeyAdd = 1;
  }

  if (State == STATE_RECORDING || State == STATE_CNFSTOP)
  {
#if defined(DIAGFIRMWARE)
    const auto & Event = g_Globals.Recorder.getLastRecord();

    if (Event.NPhase > 0)
    {
      m_LastDirection = TrafficRecord::RIGHT;
    }
    else if (Event.NPhase < 0)
    {
      m_LastDirection = TrafficRecord::LEFT;
    }
    else
    {
      m_LastDirection = TrafficRecord::UNKOWN;
    }

    m_LastSpeed = Event.Id;
    m_LastDuration = g_Globals.Recorder.getFilesize() % 512;

#else
    const auto & Event = g_Globals.Recorder.getLastTrafficRecord();

    m_LastSpeed     = Event.getSpeed();
    m_LastDirection = Event.getDirection();
    m_LastDuration  = Event.getDuration();
#endif
  }
  else if(Timer.hasTimedOut())
  {
#if !defined(DIAGFIRMWARE)
    const auto & Event = g_Globals.Recorder.getLastTrafficRecord();

    if ((State == STATE_ADJLENGTHL && Event.getDirection() == Event.LEFT)  ||
        (State == STATE_ADJLENGTHR && Event.getDirection() == Event.RIGHT))
    {
      m_LastSpeed     = Event.getSpeed();
      m_LastDirection = Event.getDirection();
      m_LastDuration  = Event.getDuration();
    }
#endif
  }

  switch(State)
  {
  case STATE_STARTUP:
    if(KeyDown & KEY_NEXT)
    {
      State = STATE_CONFIGCLOCK;
    }
    break;
  case STATE_CONFIGCLOCK:
    if(KeyDown == KEY_NEXT)
    {
      if(Bsp.getClock().isValid())
      {
        State = STATE_STARTREC;
      }
    }
    else if(KeyDown == KEY_OK)
    {
      State = STATE_CONFIGYEAR;
    }
    break;
  case STATE_CONFIGYEAR:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CONFIGMONTH;
    }
    else if(KeyLong == KEY_OK)
    {
      Bsp::DateTimeType::DateType d = Bsp.getClock();

      if (d.add({1,0,0}))
      {
        d = {2000,d.getMonth(),d.getDay()};
      }

      Bsp.setDate(d);
    }
    break;
  case STATE_CONFIGMONTH:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CONFIGDAY;
    }
    else if(KeyLong == KEY_OK)
    {
      Bsp::DateTimeType::DateType d = Bsp.getClock();

      if(d.getMonth() < 11)
      {
        d.add({0,1,0});
      }
      else
      {
        d = {d.getYear(), 0, d.getDay()};
      }

      Bsp.setDate(d);
    }
    break;
  case STATE_CONFIGDAY:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CONFIGHOUR;
    }
    else if(KeyLong == KEY_OK)
    {
      Bsp::DateTimeType::DateType d = Bsp.getClock();

      if(d.getDay() < (d.getDaysInMonth() - 1))
      {
        d.add({0,0,1});
      }
      else
      {
        d = {d.getYear(), d.getMonth(), 0};
      }

      Bsp.setDate(d);
    }
    break;
  case STATE_CONFIGHOUR:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CONFIGMINUTE;
    }
    else if(KeyLong == KEY_OK)
    {
      Bsp::DateTimeType::TimeType t = Bsp.getClock();

      t.add({1,0,0});
      Bsp.setTime(t);
    }
    break;
  case STATE_CONFIGMINUTE:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CONFIGCLOCK;
    }
    else if(KeyLong == KEY_OK)
    {
      Bsp::DateTimeType::TimeType t = Bsp.getClock();

      if(t.getMinute() < 59)
      {
        t.add({0,1,0});
      }
      else
      {
        t = {t.getHour(), 0, 0};
      }

      Bsp.setTime(t);
    }
    break;
  case STATE_STARTREC:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CONFIGCLOCK;
    }
    else if(KeyDown == KEY_OK)
    {
      m_LastSpeed     = 0;
      m_LastDirection = 0;
      m_LastDuration  = 0;

      State = STATE_CHECKSD;
    }
    break;
  case STATE_CHECKSD:
    {
      auto & Recorder = g_Globals.Recorder;
      const auto RecState = Recorder.getState();
      Recorder.activate();

      if(Recorder.STATE_NODISK == RecState)
      {
        if(KeyDown == KEY_NEXT)
        {
          State = STATE_CANCEL;
        }
      }
      else if(Recorder.STATE_NOFS == RecState)
      {
        if(KeyDown == KEY_NEXT)
        {
          State = STATE_CANCEL;
        }
        else if(KeyDown == KEY_OK)
        {
          State = STATE_FORMATSD;
        }
      }
      else if(Recorder.STATE_MOUNTED == RecState)
      {
        if(KeyDown == KEY_NEXT)
        {
          Recorder.openFile();
        }
        else if(KeyDown == KEY_OK)
        {
          State = STATE_FORMATSD;
        }
      }
      else if(Recorder.STATE_READY == RecState)
      {
        State = STATE_ENABLERADAR;
      }
      else if((Recorder.STATE_DISKFULL  == RecState) ||
              (Recorder.STATE_DISKERROR == RecState))
      {
        State = STATE_REMOVEDISK;
        setStateHint(STATE_CHECKSD);
      }
    }
    break;
  case STATE_FORMATSD:
    {
      auto & Recorder = g_Globals.Recorder;

      if(KeyDown == KEY_NEXT)
      {
        State = STATE_CANCEL;
      }
      else if(KeyDown == KEY_OK)
      {
        Recorder.formatDisk();
        State = STATE_FORMATINGSD;
      }
    }
    break;
  case STATE_FORMATINGSD:
    {
      auto & Recorder    = g_Globals.Recorder;
      auto RecorderState = Recorder.getErrorState();

      if(Recorder.STATE_MOUNTED == RecorderState)
      {
        Recorder.openFile();
        State = STATE_CHECKSD;
      }
      else if (Recorder.STATE_FORMAT != RecorderState)
      {
        State = STATE_REMOVEDISK;
      }
    }
    break;
  case STATE_ENABLERADAR:
    {
      Bsp.enableRadar();
      State = STATE_ADJLENGTHL;
    }
    break;
  case STATE_ADJLENGTHL:
    {
      if(KeyDown == KEY_NEXT)
      {
        State = STATE_ADJLENGTHR;
      }
      else if(KeyDown == KEY_OK)
      {
        if (g_Parameters.LengthCorrectionL < 8)
        {
          g_Parameters.LengthCorrectionL += 1;
        }
        else
        {
          g_Parameters.LengthCorrectionL = -8;
        }
      }
    }
    break;
  case STATE_ADJLENGTHR:
    {
      auto & Recorder = g_Globals.Recorder;

      if(KeyDown == KEY_NEXT)
      {
        State = STATE_RECORDING;
        Recorder.start();
      }
      else if(KeyDown == KEY_OK)
      {
        if (g_Parameters.LengthCorrectionR < 8)
        {
          g_Parameters.LengthCorrectionR += 1;
        }
        else
        {
          g_Parameters.LengthCorrectionR = -8;
        }
      }
    }
    break;
  case STATE_RECORDING:
    if(KeyDown == KEY_NEXT)
    {
      State = STATE_CNFSTOP;
    }
    break;
  case STATE_CNFSTOP:
    {
      auto & Recorder = g_Globals.Recorder;

      if(KeyDown == KEY_NEXT)
      {
        State = STATE_RECORDING;
      }
      else if(KeyDown == KEY_OK)
      {
        State = STATE_STOPPING;
        Recorder.finish();
        Bsp.disableRadar();
      }
    }
    break;
  case STATE_STOPPING:
    {
      auto & Recorder    = g_Globals.Recorder;
      auto RecorderState = Recorder.getState();

      if(Recorder.STATE_DISABLED == RecorderState)
      {
        State = STATE_CONFIGCLOCK;
      }
    }
    break;
  case STATE_REMOVEDISK:
    {
      auto & Recorder = g_Globals.Recorder;

      if (Recorder.STATE_DISABLED == Recorder.getState())
      {
        State = STATE_CHECKSD;
      }
      else if(KeyDown == KEY_NEXT)
      {
        State = STATE_CANCEL;
      }
    }
    break;

  case STATE_CANCEL:
    {
      auto & Recorder = g_Globals.Recorder;

      Recorder.finish();
      Bsp.disableRadar();

      if(KeyDown == KEY_NEXT)
      {
        State = STATE_CONFIGCLOCK;
      }
    }
    break;
  }

  if (State != getState())
  {
    m_KeyLongMask = 0x00;
    setState(State);
  }
}

constexpr FlashVariable<char,16> s_StartRecord     PROGMEM = "Aufzeichnung    ";
constexpr FlashVariable<char,16> s_Cancel          PROGMEM = "Abgebrochen     ";
constexpr FlashVariable<char,16> s_NoSDCard        PROGMEM = "Keine SD-Karte  ";
constexpr FlashVariable<char,16> s_CheckingSDCard  PROGMEM = "Prüfe SD-Karte  ";
constexpr FlashVariable<char,16> s_FormatSD        PROGMEM = "SD formatieren? ";
constexpr FlashVariable<char,16> s_Confirm         PROGMEM = "Wirklich?       ";
constexpr FlashVariable<char,16> s_Formating       PROGMEM = "Formatiere...   ";
constexpr FlashVariable<char,16> s_SDError         PROGMEM = "  SD Fehler XX  ";
constexpr FlashVariable<char,16> s_SDFull          PROGMEM = "   SD voll      ";
constexpr FlashVariable<char,16> s_Event           PROGMEM = "X  XX m XXX km/h";

constexpr FlashVariable<char,16> s_Diag            PROGMEM = "X       XXXXX Hz";


constexpr FlashVariable<char,16> s_Next    PROGMEM = "[W]             ";

constexpr FlashVariable<char,16> s_NextOK    PROGMEM = "[W]         [OK]";
constexpr FlashVariable<char,16> s_NextYes   PROGMEM = "[W]         [JA]";
constexpr FlashVariable<char,16> s_CancelYes PROGMEM = "[ABBRUCH]   [JA]";

constexpr FlashVariable<char,16> s_NextInc   PROGMEM = "[WEITER]     [+]";

constexpr FlashVariable<char,16> s_SetTime    PROGMEM = "[W] [UHRSTELLEN]";
constexpr FlashVariable<char,16> s_SetYear    PROGMEM = "[W]       [JAHR]";
constexpr FlashVariable<char,16> s_SetMonth   PROGMEM = "[W]      [MONAT]";
constexpr FlashVariable<char,16> s_SetDay     PROGMEM = "[W]        [TAG]";
constexpr FlashVariable<char,16> s_SetHour    PROGMEM = "[W]     [STUNDE]";
constexpr FlashVariable<char,16> s_SetMinute  PROGMEM = "[W]     [MINUTE]";
constexpr FlashVariable<char,16> s_AdjLengthL PROGMEM = "[W]    [LAENGE<]";
constexpr FlashVariable<char,16> s_AdjLengthR PROGMEM = "[W]    [LAENGE>]";
constexpr FlashVariable<char,16> s_Finish     PROGMEM = "[BEENDEN]       ";

constexpr FlashVariable<char,16> s_KeysStartRecord PROGMEM = "[W]    [STARTEN]";

constexpr FlashVariable<char,16> s_ActInsertSD PROGMEM     = " <SD einlegen>  ";
constexpr FlashVariable<char,16> s_ActRemoveSD PROGMEM     = " <SD entfernen> ";

constexpr FlashVariable<char,3>  s_Direction PROGMEM       = " <>";

void
Ui::formatTime(FramebufferType::RowBufferType & Buffer)
{
  auto       & Bsp = Bsp::getInstance();
  auto const & Clock = Bsp.getClock();

  ecpp::formatDate(Clock, *reinterpret_cast<char (*)[10]>(&(Buffer[0])));
  Buffer[10] = ' ';

  String::convertToDecimal(&(Buffer[11]), 2, Clock.getHour());
  String::convertToDecimal(&(Buffer[14]), 2, Clock.getMinute());

  if(Clock.getSecond() % 2 == 1)
  {
    Buffer[13] = ' ';
  }
  else
  {
    Buffer[13] = ':';
  }
}

void
Ui::updateDisplay()
{
  auto State = getState();

  auto & Bsp = Bsp::getInstance();
  auto & Row = m_Row;

  memset(&Row, ' ', sizeof(Row));

  String::formatUnsigned(&(Row[0]), 2, State);
  String::formatUnsigned(&(Row[3]), 2, g_Globals.Recorder.getState());

  switch(State)
  {
  case STATE_STARTUP:
    formatTime(Row);
    break;
  case STATE_CONFIGCLOCK:
    formatTime(Row);
    break;
  case STATE_CONFIGYEAR:
  case STATE_CONFIGMONTH:
  case STATE_CONFIGDAY:
  case STATE_CONFIGHOUR:
  case STATE_CONFIGMINUTE:
    formatTime(Row);
    break;
  case STATE_STARTREC:
    s_StartRecord.read(Row);
    break;
  case STATE_CHECKSD:
    {
      const auto & Recorder = g_Globals.Recorder;
      const auto RecState = Recorder.getState();

      if(Recorder.STATE_NODISK == RecState)
      {
        s_NoSDCard.read(Row);
      }
      else if(Recorder.STATE_NOFS == RecState)
      {
        s_FormatSD.read(Row);
      }
      else if(Recorder.STATE_MOUNTED == RecState)
      {
        s_FormatSD.read(Row);
      }
    }
    break;
  case STATE_FORMATSD:
    s_Confirm.read(Row);
    break;
  case STATE_FORMATINGSD:
    s_Formating.read(Row);
    break;
  case STATE_ENABLERADAR:
    break;

  case STATE_ADJLENGTHL:
    {
      s_Event.read(Row);

      String::formatUnsigned(&(Row[0]), 2, g_Parameters.LengthCorrectionL);
      String::formatUnsigned(&(Row[2]), 3, calculateLength());
      String::formatUnsigned(&(Row[8]), 3, m_LastSpeed);
    }
    break;
  case STATE_ADJLENGTHR:
    {
      s_Event.read(Row);

      String::formatUnsigned(&(Row[0]), 2, g_Parameters.LengthCorrectionR);
      String::formatUnsigned(&(Row[2]), 3, calculateLength());
      String::formatUnsigned(&(Row[8]), 3, m_LastSpeed);
    }
    break;
  case STATE_RECORDING:
    {
#if defined(DIAGFIRMWARE)
      s_Diag.read(Row);

      if (Bsp.FREQCNT_RUN == Bsp.getFreqCntState())
      {
        Row[0] = g_Globals.Recorder.LastPhase;
        String::formatUnsigned(&(Row[8]), 5, g_Globals.Recorder.LastFreq);
      }
#else
      s_Event.read(Row);


      Row[0] = s_Direction[m_LastDirection];
      String::formatUnsigned(&(Row[2]), 3, calculateLength());
      String::formatUnsigned(&(Row[8]), 3, m_LastSpeed);
#endif
    }
    break;
  case STATE_CNFSTOP:
    s_Confirm.read(Row);
    break;
  case STATE_CANCEL:
    s_Cancel.read(Row);
    break;

  case STATE_REMOVEDISK:
    {
      const auto & Recorder = g_Globals.Recorder;
      const auto RecState = Recorder.getState();

      if(Recorder.STATE_DISKFULL == RecState)
      {
        s_SDFull.read(Row);
      }
      else if(Recorder.STATE_ERROR <= RecState)
      {
        s_SDError.read(Row);
        String::formatUnsigned(&(Row[12]), 2, RecState - Recorder.STATE_ERROR);
      }
    }
    break;
  }

  m_Framebuffer.setRow({0,0}, Row);

  memset(&Row, ' ', sizeof(Row));
  switch(getState())
  {
  case STATE_STARTUP:
    s_Next.read(Row);
    break;
  case STATE_CONFIGCLOCK:
    s_SetTime.read(Row);

    if(!Bsp.getClock().isValid())
    {
      memset(&(Row[0]), ' ', 3);
    }
    break;
  case STATE_CONFIGYEAR:
    s_SetYear.read(Row);
    break;
  case STATE_CONFIGMONTH:
    s_SetMonth.read(Row);
    break;
  case STATE_CONFIGDAY:
    s_SetDay.read(Row);
    break;
  case STATE_CONFIGHOUR:
    s_SetHour.read(Row);
    break;
  case STATE_CONFIGMINUTE:
    s_SetMinute.read(Row);
    break;
  case STATE_STARTREC:
    s_KeysStartRecord.read(Row);
    break;
  case STATE_CHECKSD:
    {
      const auto & Recorder = g_Globals.Recorder;
      const auto RecState = Recorder.getState();

      if (Recorder.STATE_NODISK == RecState)
      {
        s_ActInsertSD.read(Row);
      }
      else if(Recorder.STATE_NOFS == RecState)
      {
        s_CancelYes.read(Row);
      }
      else if(Recorder.STATE_MOUNTED == RecState)
      {
        s_NextYes.read(Row);
      }
    }
    break;
  case STATE_FORMATSD:
    s_CancelYes.read(Row);
    break;
  case STATE_ADJLENGTHL:
    s_AdjLengthL.read(Row);
    break;
  case STATE_ADJLENGTHR:
    s_AdjLengthR.read(Row);
    break;
  case STATE_RECORDING:
    s_Finish.read(Row);
    break;
  case STATE_CNFSTOP:
    s_CancelYes.read(Row);
    break;
  case STATE_CANCEL:
    s_Next.read(Row);
    break;
  case STATE_REMOVEDISK:
    {
      s_ActRemoveSD.read(Row);
    }
    break;
  }

  m_Framebuffer.setRow({0,1}, Row);
}

