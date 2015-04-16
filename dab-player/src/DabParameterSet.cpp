/*
 * DabParameterSet.cpp
 *
 *  Created on: 16.04.2015
 *      Author: andi
 */
#include "DabParameterSet.hpp"



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

DabParameterSetReference::DabParameterSetReference(enum DabMode mode) :
    _pCurrentParameters(&DefaultDabParameterSets[mode])
{

}

DabParameterSetReference::DabParameterSetReference(const DabParameterSetReference& rhs):
    _pCurrentParameters(rhs._pCurrentParameters)
{

}

DabParameterSetReference & DabParameterSetReference::operator = (const DabParameterSetReference & rhs)
{
  _pCurrentParameters = rhs._pCurrentParameters;
  return *this;
}


