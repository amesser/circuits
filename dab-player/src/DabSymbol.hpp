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

  enum State
  {
    STATE_SAMPLE,
    STATE_DFT,
    STATE_FREQCOR,
    STATE_OFDM,
  };

private:
  SampleType * const _pBuffer;
  enum State         _State;

  float              _CoarseOffset;
  float              _FineOffset;

public:
  DabSymbol();
  ~DabSymbol();

  SampleType * getBuffer() { return _pBuffer;}

  void  setOffsets(float Coarse, float Fine) { _CoarseOffset = Coarse; _FineOffset = Fine;}
  float getCoarseOffset() const { return _CoarseOffset;}
  float getFineOffset()   const { return _FineOffset;}

  void setState(enum State State) {_State = State;}
  enum State getState() const { return _State;}
};



#endif /* DABSYMBOL_HPP_ */
