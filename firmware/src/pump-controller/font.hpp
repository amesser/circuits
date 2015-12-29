/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of Soil Moisture Pump Controller firmwares.
 *
 *  This software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *  */
#ifndef FIRMWARE_SRC_FONT_HPP_
#define FIRMWARE_SRC_FONT_HPP_

#include "ecpp/Target.hpp"

using namespace ecpp;

struct character
{
  const FlashVariable<uint8_t> bitmap[8];
};

extern const struct character g_Font[256] PROGMEM;


#endif /* FIRMWARE_SRC_FONT_HPP_ */
