/*
 * symbol.hpp
 *
 *  Created on: 09.04.2015
 *      Author: andi
 */

#ifndef DABSYMBOL_HPP_
#define DABSYMBOL_HPP_

#include "DabParameterSet.hpp"

#include <iostream>

#include <stddef.h>
#include <fftw3.h>

class DabSymbol
{
public:
  typedef fftwf_complex SampleType;

private:
  SampleType * const _pBuffer;
  float              _Offset;

public:
  DabSymbol();
  ~DabSymbol();

  SampleType * getBuffer() { return _pBuffer;}

  void setOffset(float Offset) { _Offset = Offset;}
};



#endif /* DABSYMBOL_HPP_ */
