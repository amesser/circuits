/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor firmwares.
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
  uint16_t Min;
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
