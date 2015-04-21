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
  uint8_t  SymbolsPerFrame; /* excluding the phase reference symbol*/
};

class DabParameterSetReference
{
public:
  enum DabMode
  {
    DAB_MODE0 = 0,
    DAB_MODE1 = 1,
    DAB_MODE2 = 2,
    DAB_MODE3 = 3
  };

  typedef uint16_t MappingTableType[1536];

private:
  struct DabParameterSet const *_pCurrentParameters;
  enum DabMode            _Mode;
  MappingTableType        _ChannelMapping;

  void updateMappingTable();
  void setDabMode(enum DabMode mode);
public:

  DabParameterSetReference(enum DabMode mode);
  DabParameterSetReference(const DabParameterSetReference& rhs);

  DabParameterSetReference & operator = (const DabParameterSetReference & rhs);

  static constexpr unsigned int getMaxSamples()    { return 2048 + 504; }
  static constexpr unsigned int getMaxSampleRate() { return 2048000;    }

  unsigned int getSamplesPerSymbol()      const {return _pCurrentParameters->SamplesPerSymbol;}
  unsigned int getSamplesPerSymbolGuard() const {return _pCurrentParameters->SamplesPerSymbolGuard;}
  unsigned int getSymbolsPerFrame()       const {return _pCurrentParameters->SymbolsPerFrame;}
  unsigned int getNumCarriers()           const {return _pCurrentParameters->Carriers;}
  unsigned int getSampleRate()            const {return _pCurrentParameters->SampleRate;}
  const MappingTableType & getMappingTable() const {return _ChannelMapping;};
};


#endif /* DABPARAMETERSET_HPP_ */
