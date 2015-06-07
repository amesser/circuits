/*
 * controller_ui.cpp
 *
 *  Created on: 05.06.2015
 *      Author: andi
 */

#include "controller_ui.hpp"
#include "controller_app.hpp"
#include "controller_bsp.hpp"

#include "ecpp/Datatypes.hpp"
#include "ecpp/String.hpp"

using namespace ecpp;

UserInterface ui;

static const char s_TextKeySelect[] PROGMEM = "<   EE   >";
static const char s_TextKeyEdit[]   PROGMEM = "-   OK   +";

static const char s_TextTime[] PROGMEM           = " Uhrzeit ";
static const char s_TextWettingTime[] PROGMEM    = "Startzeit";
static const char s_TextWettingPeriod[] PROGMEM  = "  Dauer  ";
static const char s_TextWettingTimeout[] PROGMEM = "Abstand  ";
static const char s_TextChannelA[]      PROGMEM  = " Kanal A ";

static const char s_TextDay[]    PROGMEM = "SoMoDiMiDoFrSa";
static const char s_TextStatus[] PROGMEM = "BatAnAus";

typedef void (*MenuHandler)(uint8_t keydown, uint8_t keylong);

static void FormatTime(char *buf, const WeekTime & Time, uint8_t day = 1)
{
  if(day == 1)
  {
    memcpy_P(buf, &s_TextDay[Time.getWeekDay()*2], 2);
    buf += 3;
  }
  else if (day == 2)
  {
    String::formatDecimal(buf, 2, Time.getWeekDay(), '0');
    buf += 2;
    buf[0] = ':';
    buf += 1;
  }

  String::formatDecimal(buf, 2, Time.getHour(), '0');
  buf += 2;
  buf[0] = ':';
  buf += 1;

  String::formatDecimal(buf, 2, Time.getMinute(), '0');
}

static void MenuDebug()
{
#if 0
  auto UiTimer        = bsp.getUiTimer();
  auto UiRefreshTimer = bsp.getUiRefreshTimer();
  auto line = bsp.getDisplayLine(3);
  memset(line, 0x00, 10);

  String::formatDecimal(line + 0, 5, UiTimer.getRemainingTime());
  String::formatDecimal(line + 5, 5, UiRefreshTimer.getRemainingTime());
#endif
}

static void MenuHandler_Idle(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();


  if(bsp.getAccuState() == bsp.ACCU_EMPTY)
    memcpy_P(&display.Lines[0][7], &s_TextStatus[0], 3);

  display.Lines[1][0] = 'U';
  String::formatDecimal(&(display.Lines[1][4]), 5, bsp.getAccuVoltage(),' ', 2);

  auto & Time = Application.getTime();

  if(Application.isWettingChA())
  {
    if((Time.getSecond() % 2) == 0)
    {
      WeekTime Time;

      Time.setMonotonic(Application.getActualWettingTimeChA());

      display.Lines[2][0] = 'A';
      FormatTime(&(display.Lines[2][2]), Time, 2);
    }
  }
  else
  {
    display.Lines[2][0] = 'A';
    FormatTime(&(display.Lines[2][2]), Application.getLastWettingTimeChA());
  }

  MenuDebug();
}




static void MenuHandler_Time(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();

  memcpy_P(display.Lines[0], s_TextTime, 10);

  keydown |= keylong;

  if (keydown & 0x01)
  {
    Application.changeTime(+1);
  }
  else if (keydown & 0x02)
  {
    Application.changeTime(0);
  }
  else if (keydown & 0x04)
  {
    Application.changeTime(-1);
  }

  auto & Time = Application.getTime();
  FormatTime(&display.Lines[2][1], Time);

  MenuDebug();
}

static void MenuHandler_WettingTime(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();

  memcpy_P(display.Lines[0], s_TextWettingTime, 10);

  keydown |= keylong;

  if (keydown & 0x01)
  {
    Application.changeWettingTime(+1);
  }
  else if (keydown & 0x02)
  {
    Application.changeWettingTime(0);
  }
  else if (keydown & 0x04)
  {
    Application.changeWettingTime(-1);
  }

  auto & Time = Application.getWettingTime();
  FormatTime(&display.Lines[2][1], Time, 1);

  MenuDebug();
}

static void MenuHandler_WettingDuration(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();

  memcpy_P(display.Lines[0], s_TextWettingPeriod, 10);

  keydown |= keylong;

  if (keydown & 0x01)
  {
    Application.changeWettingPeriod(+60);
  }
  else if (keydown & 0x02)
  {
    Application.changeWettingPeriod(0);
  }
  else if (keydown & 0x04)
  {
    Application.changeWettingPeriod(-60);
  }


  auto Period = Application.getWettingPeriod();
  WeekTime Time;

  Time.setMonotonic(Period);

  FormatTime(&display.Lines[2][1], Time, 2);

  MenuDebug();
}

static void MenuHandler_WettingTimeout(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();

  memcpy_P(display.Lines[0], s_TextWettingTimeout, 10);

  keydown |= keylong;

  if (keydown & 0x01)
  {
    Application.changeWettingTimeout(+1);
  }
  else if (keydown & 0x02)
  {
    Application.changeWettingTimeout(0);
  }
  else if (keydown & 0x04)
  {
    Application.changeWettingTimeout(-1);
  }

  auto & Time = Application.getWettingTimeout();
  FormatTime(&display.Lines[2][1], Time, 2);

  MenuDebug();
}

static void MenuHandler_Channel(uint8_t mask, uint8_t keydown)
{
  auto & display = bsp.getDisplay();

  if (keydown & 0x01)
  {
    bsp.enableOutputs(mask);
  }
  else if (keydown & 0x04)
  {
    bsp.disableOutputs(mask);
  }

  if(bsp.getOutputState() & mask)
    memcpy_P(&display.Lines[2][3], &s_TextStatus[3], 2);
  else
    memcpy_P(&display.Lines[2][3], &s_TextStatus[5], 3);
}

static void MenuHandler_ChannelA(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();
  memcpy_P(display.Lines[0], s_TextChannelA, 10);
  MenuHandler_Channel(bsp.OUTPUT_CHA, keydown);
}

static void MenuHandler_ChannelB(uint8_t keydown, uint8_t keylong)
{
  auto & display = bsp.getDisplay();
  memcpy_P(display.Lines[0], s_TextChannelA, 10);
  display.Lines[0][7] = 'B';

  MenuHandler_Channel(bsp.OUTPUT_CHB, keydown);
}

static const FlashVariable<MenuHandler> s_MenuHandlers[] PROGMEM =
{
    MenuHandler_Idle,
    MenuHandler_Time,
    MenuHandler_WettingTime,
    MenuHandler_WettingDuration,
    MenuHandler_WettingTimeout,
    MenuHandler_ChannelA,
    MenuHandler_ChannelB,
};

void UserInterface::updateStatusLine(void)
{
  auto & display = bsp.getDisplay();

  if(m_CurrentMenuItem != 0)
  {
    if(m_Edit)
    {
      memcpy_P(display.Status, s_TextKeyEdit, 10);
    }
    else
    {
      memcpy_P(display.Status, s_TextKeySelect, 10);
    }
  }
  else
  {
    auto & Time = Application.getTime();

    memset(display.Status, 0x00, 10);
    FormatTime(&display.Status[2], Time);

    if(Time.getSecond() % 2)
      display.Status[7] = 0;
  }
}

void UserInterface::cycle(void)
{
  uint8_t keymask = bsp.getKeyState();
  uint8_t keydown = keymask & (keymask ^ m_Keymask);
  uint8_t keylong = 0;

  auto & UiTimer        = bsp.getUiTimer();
  auto & UiRefreshTimer = bsp.getUiRefreshTimer();

  auto MenuItem = m_CurrentMenuItem;

  bool refresh = false;

  if (keymask)
  {
    UiTimer.start(30000);
  }
  else if(UiTimer.hasTimedOut())
  {
    if (m_Edit)
      keydown = 0x02;
    else
      MenuItem = 0;

    refresh = true;
  }

  if(m_Keymask != keymask)
  {
    m_Keymask = keymask;
    refresh = true;
  }
  else if(UiRefreshTimer.hasTimedOut())
  {
    refresh = true;
  }

  auto keytime = bsp.getKeyTime();

  if(keytime > 500)
    keylong = keymask;

  if(refresh)
  {
    if(MenuItem != 0 || keymask)
    {
      if(keytime > 5000)
      {
        UiRefreshTimer.start(20);
      }
      else
      {
        UiRefreshTimer.start(100);
      }
    }
    else
    {
      UiRefreshTimer.start(1000);
    }
  }

  if (MenuItem == 0)
  {
    m_Edit = 0;

    if(keylong & 0x02)
    {
      MenuItem = 1;
      keylong  = 0;
      keydown  = 0;
    }
  }
  else if (MenuItem > 0 && m_Edit == 0)
  {
    if(keydown & 0x01)
    {
      if ((MenuItem + 1) < (uint8_t)(sizeof(s_MenuHandlers) / sizeof(s_MenuHandlers[0])))
      {
        MenuItem = MenuItem + 1;
      }
      else
      {
        MenuItem = 1;
      }
    }
    else if(keydown & 0x02)
    {
      m_Edit = 1;
    }
    else if(keydown & 0x04)
    {
      if ( MenuItem > 1)
      {
        MenuItem = MenuItem - 1;
      }
      else
      {
        MenuItem = sizeof(s_MenuHandlers) / sizeof(s_MenuHandlers[0]) - 1;
      }

    }
    keydown = 0;
    keylong = 0;
  }
  else if (MenuItem > 0 && m_Edit == 1)
  {
    if(keydown & 0x02)
    {
      m_Edit  = 0;
      keylong = 0;
    }
  }

  if(MenuItem >= sizeof(s_MenuHandlers) / sizeof(s_MenuHandlers[0]))
    MenuItem = 0;

  if(refresh)
  {
    auto & display = bsp.getDisplay();
    memset(&display.Lines[0][0], 0x00, 10 * 4);

    updateStatusLine();

    MenuHandler Handler = s_MenuHandlers[MenuItem];

    if(0 != Handler)
    {
      Handler(keydown, keylong);
    }

    bsp.handleLCD();
  }

  m_CurrentMenuItem = MenuItem;
}

