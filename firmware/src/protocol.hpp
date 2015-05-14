/*
 * protocol.hpp
 *
 *  Created on: 09.05.2015
 *      Author: andi
 */

#include <stdint.h>

#ifndef PROTOCOL_HPP_
#define PROTOCOL_HPP_

struct measurement_data
{
  uint8_t  sync;
  uint8_t  type;
  uint16_t humidity_counts;
  uint16_t led_counts;
  uint16_t temp;
};

#endif /* PROTOCOL_HPP_ */
