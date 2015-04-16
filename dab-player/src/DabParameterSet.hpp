/*
 * DabParameterSet.hpp
 *
 *  Created on: 16.04.2015
 *      Author: andi
 */

#ifndef DABPARAMETERSET_HPP_
#define DABPARAMETERSET_HPP_

#include <stdint.h>

struct DabParameterSet
{
  uint32_t SampleRate;
  uint16_t SamplesPerSymbol;
  uint16_t SamplesPerSymbolGuard;
  uint16_t Carriers;
  int8_t   SymbolsPerFrame; /* excluding the phase reference symbol*/
};

class DabParameterSetReference
{
private:
  const struct DabParameterSet *_pCurrentParameters;
public:

  enum DabMode
  {
    DAB_MODE0 = 0,
    DAB_MODE1 = 1,
    DAB_MODE2 = 2,
    DAB_MODE3 = 3
  };

  DabParameterSetReference(enum DabMode mode);
  DabParameterSetReference(const DabParameterSetReference& rhs);

  DabParameterSetReference & operator = (const DabParameterSetReference & rhs);

  static constexpr unsigned int getMaxSamples() { return 2048 + 504; }
  unsigned int getSamplesPerSymbol()      const {return _pCurrentParameters->SamplesPerSymbol;}
  unsigned int getSamplesPerSymbolGuard() const {return _pCurrentParameters->SamplesPerSymbolGuard;}
           int getSymbolsPerFrame()       const {return _pCurrentParameters->SymbolsPerFrame;}
  unsigned int getNumCarriers()           const {return _pCurrentParameters->Carriers;}
  unsigned int getSampleRate()            const {return _pCurrentParameters->SampleRate;}
};


#endif /* DABPARAMETERSET_HPP_ */
