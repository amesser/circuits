/*
 * fit.cpp
 *
 *  Created on: 17.10.2015
 *      Author: andi
 */
#include "fit.hpp"
#include <string.h>

void accumulate(uint32_t & accu, uint32_t value) __attribute__((noinline));
void accumulate(uint32_t & accu, uint32_t value)
{
  accu += value;
}

void accumulate(uint32_t & accu, uint16_t value) __attribute__((noinline));
void accumulate(uint32_t & accu, uint16_t value)
{
  accumulate(accu, static_cast<uint32_t>(value));
}

void
CalibratorFit::resetMinMaxStatistics(struct calibration_minmax & stat)
{
  stat.Min = ~0;
  stat.Max =  0;
}

void
CalibratorFit::collectMinMaxStatistics(struct calibration_minmax &stat, uint16_t Counts)
{
  if(stat.Min > stat.Max)
  { /* First value */
    stat.Min = stat.Max = Counts;
  }
  else if(Counts > stat.Max)
  {
    stat.Max = Counts;
  }
  else if(Counts < stat.Min )
  {
    stat.Min = Counts;
  }
}

void CalibratorFit::calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm)
{
  uint16_t Delta;

  prm.Max    =  stat.Max;
  prm.Min    =  stat.Min;
  prm.Offset = -stat.Min;

  Delta = stat.Max - stat.Min;

  prm.Mult   = (0xFFFF * (uint32_t)0x10000ULL + Delta - 1) / Delta;
}

void CalibratorFit::resetLinRegrStat(struct calibration_linearregression &stat)
{
  memset(&stat, 0x00, sizeof(stat));
}

#define LINREG_WINDOW (uint_fast16_t)5
void
CalibratorFit::collectTempStatistics(struct calibration_linearregression &stat, uint16_t TempCounts, uint16_t Temp)
{
  /* Temperature statistics is kept in seperate bins of 2 degrees.
   * This is used to improve the temperature calibration distribution */
  const auto     NumBins = ElementCount(stat.NumPoints);

  const uint16_t MinTemp = 273 - 10;
  const uint16_t MaxTemp = MinTemp + 2* NumBins;

  if(TempCounts <= 0x3FF && Temp >= MinTemp && Temp < MaxTemp)
  {
    uint_fast16_t ThisBin, MaxPointsPerBin, Bin;

    ThisBin = static_cast<uint_fast8_t>((Temp - MinTemp) / 2);

    /* We have maximum accumulated value from 10Bit * 10Bit = 20Bit value
     * Thus 12 Bits are left for accumulation. Furthermore we can only count up to 255
     */
    MaxPointsPerBin = min(0xFF, 0xFFF / NumBins);

    for(Bin=max(LINREG_WINDOW, ThisBin) - LINREG_WINDOW; Bin < min((uint_fast16_t)NumBins, ThisBin + LINREG_WINDOW); ++Bin)
    {
      if(Bin != ThisBin)
      {
        MaxPointsPerBin = min(MaxPointsPerBin, static_cast<uint_fast16_t>(stat.NumPoints[Bin] + 2));
      }
    }

    if(stat.NumPoints[ThisBin] < MaxPointsPerBin)
    {
      stat.NumPoints[ThisBin] += 1;
      accumulate(stat.SumY, Temp);
      accumulate(stat.SumXY, static_cast<uint32_t>(TempCounts) * Temp);
      accumulate(stat.SumX,TempCounts);
      accumulate(stat.SumXX, static_cast<uint32_t>(TempCounts) * TempCounts);
    }
  }
}

void CalibratorFit::calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm)
{
  /* y = Mult * (x + Offset) / 65536 */

  uint_fast8_t NumPoints;
  int_fast32_t Numerator, Denominator, SumX, SumY;

  NumPoints = sum(stat.NumPoints);

  SumX      = stat.SumX;
  SumY      = stat.SumY;

  Numerator   = stat.SumXY - ((SumX * SumY) + NumPoints / 2) / NumPoints;
  Denominator = stat.SumXX - ((SumX * SumX) + NumPoints / 2) / NumPoints;

  if(Denominator != 0)
  {
    prm.Mult   = (((int_fast64_t)Numerator) * 65536ULL) / Denominator ;
  }
  else
  {
    prm.Mult   = 0;
  }

  if(prm.Mult != 0)
  {
    int32_t Offset = (SumY + NumPoints / 2) * 65536ULL / NumPoints / prm.Mult
        - (SumX + NumPoints / 2) / NumPoints;

    prm.Offset = Offset;
    prm.Max    = 0xFFFF0000 / prm.Mult - Offset;
    prm.Min    = 0;
  }
  else
  {
    prm.Offset = 0;
    prm.Min    = 0xFFFF;
    prm.Max    = 0xFFFF;
  }

}

