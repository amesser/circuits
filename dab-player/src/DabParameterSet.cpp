/*
 * DabParameterSet.cpp
 *
 *  Created on: 16.04.2015
 *      Author: andi
 */
#include "DabParameterSet.hpp"
#include <stdio.h>


static const struct DabParameterSet DefaultDabParameterSets[4] =
{
  {
  },
  {
  },
  {
  },
  {
    .SampleRate             = 2048000,
    .SamplesPerSymbol       = 2048,
    .SamplesPerSymbolGuard  = 504,
    .Carriers               = 1536,
    .SymbolsPerFrame        = 76,
  },
};

DabParameterSetReference::DabParameterSetReference(enum DabMode mode)
{
  setDabMode(mode);
}

DabParameterSetReference::DabParameterSetReference(const DabParameterSetReference& rhs)
{
  setDabMode(rhs._Mode);
}

DabParameterSetReference & DabParameterSetReference::operator = (const DabParameterSetReference & rhs)
{
  setDabMode(rhs._Mode);
  return *this;
}

void DabParameterSetReference::setDabMode(enum DabMode Mode)
{
  _pCurrentParameters = &(DefaultDabParameterSets[Mode]);

  updateMappingTable();

}
void DabParameterSetReference::updateMappingTable()
{
  const auto SamplesPerSymbol = getSamplesPerSymbol();
  const auto Carriers          = getNumCarriers();

  const auto FirstCarrier = (SamplesPerSymbol - Carriers) / 2;
  const auto LastCarrier  = SamplesPerSymbol - FirstCarrier;

  const auto v1 = SamplesPerSymbol - Carriers - 1;
  unsigned int i,j;

  for(i = 0, j=0; i < Carriers; )
  {
    if (j != SamplesPerSymbol / 2 and j >= FirstCarrier and j <= LastCarrier)
    {
      /* fft has swapped halfs !! */
      if (j < (SamplesPerSymbol / 2))
        _ChannelMapping[i] = j + SamplesPerSymbol / 2;
      else
        _ChannelMapping[i] = j - SamplesPerSymbol / 2;

      i++;
    }

    j = (13 * j + v1) % SamplesPerSymbol;
  }
}
