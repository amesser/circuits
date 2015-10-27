/*
 * fit.hpp
 *
 *  Created on: 17.10.2015
 *      Author: andi
 */

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
  static void collectMinMaxStatistics(struct calibration_minmax &stat, uint16_t Counts, bool ForceCollect);
  static void calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm);

  static void resetLinRegrStat(struct calibration_linearregression &stat);
  static void collectTempStatistics(struct calibration_linearregression &stat, uint16_t TempCounts, uint16_t Temp);
  static void calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm);

};


#endif /* SRC_FIT_HPP_ */
