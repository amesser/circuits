/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor Calibrator firmware.
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
#ifndef SRC_FIT_HPP_
#define SRC_FIT_HPP_

#include "ecpp/Datatypes.hpp"
#include "protocol.hpp"

using namespace ecpp;

class CalibratorFit
{
public:
  struct calibration_minmax
  {
    uint16_t Max;
    uint16_t Min;
  };

  /** Hold the data used for linear regression
   *
   * In order to avoid to strong influence of lots
   * of calibration points in one range, the x/y measurement
   * pair will be grouped in slots. each slot is 2 degrees
   * wide. When calculating the calibration factors, each slot
   * will be weighted with the inverse of its contents
   */
  struct calibration_linearregression
  {
    uint32_t SumX;
    uint32_t SumY;
    uint32_t SumXY;
    uint32_t SumXX;
    uint8_t  NumPoints[25];
  };

  static void resetMinMaxStatistics(struct calibration_minmax & stat);
  static void collectMinMaxStatistics(struct calibration_minmax &stat, uint16_t Counts);
  static void calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm);

  static void resetLinRegrStat(struct calibration_linearregression &stat);
  static void collectTempStatistics(struct calibration_linearregression &stat, uint16_t TempCounts, uint16_t Temp);
  static void calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm);

};


#endif /* SRC_FIT_HPP_ */
