/*
 * symbol.cpp
 *
 *  Created on: 09.04.2015
 *      Author: andi
 */
#include "DabSymbol.hpp"



DabSymbol::DabSymbol() :
  _pBuffer(static_cast<fftwf_complex*>(fftwf_malloc(sizeof(*_pBuffer) * DabParameterSetReference::getMaxSamples())))
{

}

DabSymbol::~DabSymbol()
{
  fftwf_free(_pBuffer);
}
