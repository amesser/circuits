/*
 * font.hpp
 *
 *  Created on: 05.06.2015
 *      Author: andi
 */

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
