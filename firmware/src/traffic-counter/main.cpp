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
#include <string.h>

#include "ecpp/Target.hpp"

#include "app.hpp"
#include "bsp.hpp"
#include "ui.hpp"

using namespace ecpp;

Globals    g_Globals;
Parameters g_Parameters;
static Ui  s_Ui;


int main(void) __attribute__ ((OS_main));
int main()
{
  auto & Bsp = Bsp::getInstance();

  Bsp.init();

  Sys_AVR8::enableInterrupts();

  while(1)
  {
    uint8_t TicksPassed256Hz;

    TicksPassed256Hz = Bsp.poll();

    s_Ui.poll(TicksPassed256Hz);
    g_Globals.Recorder.poll();

    s_Ui.updateDisplay();
    Bsp.updateFrameBuffer(s_Ui.getFrameBuffer());
  };
}
