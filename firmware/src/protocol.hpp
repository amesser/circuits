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

struct calibration_param
{
  uint32_t Mult;
  uint16_t Offset;
  uint16_t Max;
};

struct calibration_data
{
  calibration_param Humidity;
  calibration_param Light;
  calibration_param Temperature;
};

class Evaluator : public calibration_param
{
public:
  uint16_t scale(uint16_t value) const;
};


#endif /* PROTOCOL_HPP_ */
