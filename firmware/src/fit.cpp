/*
 * fit.cpp
 *
 *  Created on: 17.10.2015
 *      Author: andi
 */
#include "fit.hpp"

#define MAX_MINMAX_JUMPWIDTH (25)

void
CalibratorFit::collectMinMaxStatistics(struct calibration_minmax &stat, uint_fast16_t Counts, bool ForceCollect)
{
  const auto MaxJumpwidth = ForceCollect ? TypeProperties<decltype(stat.Max)>::MaxUnsigned : MAX_MINMAX_JUMPWIDTH;

  if(stat.Min > stat.Max)
  { /* First value */
    stat.Min = stat.Max = Counts;
  }
  else if(Counts > stat.Max && (Counts - stat.Max) <= MaxJumpwidth)
  {
    stat.Max = Counts;
  }
  else if(Counts < stat.Min && (stat.Min - Counts) <= MaxJumpwidth)
  {
    stat.Min = Counts;
  }
}

void CalibratorFit::calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm)
{
  prm.Offset = -stat.Min;
  prm.Max    =  stat.Max - stat.Min;
  prm.Mult   = (0xFFFF * 0x10000ULL) / prm.Max;
}

void
CalibratorFit::collectTempStatistics(struct calibration_linearregression &stat, uint_fast16_t TempCounts, uint_fast16_t Temp)
{
  /* Temperature statistics is kept in seperate bins of 2 degrees.
   * This is used to improve the temperature calibration distribution */

  /* Bins are available from [-10, +30 [ */
  if(Temp >= (273 - 10) && Temp < (273 + 30))
  {
    uint_fast16_t UTemp = Temp - (273 - 10);
    uint_fast8_t Idx = static_cast<uint_fast8_t>(UTemp / 2);

    if(stat.NumPoints[Idx] < (min(stat.NumPoints) + 2))
    {
      stat.NumPoints[Idx] += 1;
      stat.SumX[Idx]      += TempCounts;
      stat.SumX[Idx]      += TempCounts * TempCounts;
      stat.SumY[Idx]      += Temp;
      stat.SumXY[Idx]     += Temp * TempCounts;
    }
  }
}

void CalibratorFit::calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm)
{
  /* y = Mult * (x + Offset) / 65536 */

  uint_fast8_t NumPoints;
  int_fast32_t Numerator, Denominator, SumX, SumY;

  NumPoints = sum(stat.NumPoints);

  SumX      = sum(stat.SumX);
  SumY      = sum(stat.SumY);

  Numerator   = sum(stat.SumXY) - ((SumX * SumY) + NumPoints / 2) / NumPoints;
  Denominator = sum(stat.SumXX) - ((SumX * SumX) + NumPoints / 2) / NumPoints;

  prm.Mult   = (((int_fast64_t)Numerator) * 65536ULL) / Denominator ;
  prm.Offset =    (SumY + NumPoints / 2) * 65536ULL / NumPoints / prm.Mult
                - (SumX + NumPoints / 2) / NumPoints;
  prm.Max    = 0xFFFF0000 / prm.Mult;
}

