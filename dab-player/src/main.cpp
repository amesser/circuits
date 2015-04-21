/*
 * main.cpp
 *
 *  Created on: 17.03.2015
 *      Author: andi
 */

#include "DabParameterSet.hpp"
#include "worker.hpp"
#include "DabSymbol.hpp"

#include <complex>
#include <cmath>

#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fftw3.h>

#include "ecpp/Filter.hpp"

using namespace ecpp;

class Event
{
public:
  virtual void signal() = 0;
  virtual ~Event() {}
};

class InputFeeder : public Job
{
private:
  DabParameterSetReference    & _Prm;

  static constexpr unsigned int _nelements = 8192;
  static constexpr unsigned int _navgshort = 512;

  Queue & _queue;
  Event & _event;

  int     _fh;

  int8_t   _data[_nelements][2];
  uint16_t _abs [_nelements];

  unsigned long _avg_long;
  unsigned long _avg_short;

  volatile unsigned int _writeOffset;
  volatile unsigned int _readOffset;

  enum State {
    SOF_STATE_PREFILL     = 0,
    SOF_STATE_INIT        = 1,
    SOF_STATE_PAUSE       = 2,
    SOF_STATE_FRAME       = 3,
  };

  int  _symbolcnt;
  State _sofState;

  unsigned int  _sofOffset;
  int            _corPhase;


  volatile bool _done;
  PT1<float,16>    _OffsetCorrection;

  inline void updateSignalLevels(unsigned long index)
  {
    index = index % _nelements;

    const int16_t i = _data[index][0];
    const int16_t q = _data[index][1];

    const unsigned long abs = (unsigned long)(i * i) + (unsigned long)(q * q);

    _avg_long  = _avg_long  + abs - _abs[index];
    _avg_short = _avg_short + abs - _abs[((unsigned int)(index - _navgshort)) % _nelements];

    _abs[index] = abs;
  }

  static ::std::complex<float> * getSineTable();

public:

  int getSymbolCnt() const {return _symbolcnt;};

  InputFeeder(DabParameterSetReference & Prm, Queue & queue, Event & event, const char *path);

  void run();

  bool done() {return _done;}

  void start();

  int  findNextSymbol();

  void getSymbol(DabSymbol & symbol);
  void updateOffset(const DabSymbol & Symbol);
};

InputFeeder::InputFeeder(DabParameterSetReference & Prm, Queue & queue, Event & event, const char *path) :
    _Prm(Prm),
    _queue(queue), _event(event), _avg_long(0) , _avg_short(0),
    _writeOffset(0), _readOffset(0), _symbolcnt(-1),
    _sofState(SOF_STATE_PREFILL), _done(false), _corPhase(0)
{
  _fh = open(path, O_RDONLY);

  memset(_abs, 0x00, sizeof(_abs));
}

void InputFeeder::run()
{
  unsigned int fill = _writeOffset - _readOffset;

  if(fill < _nelements)
  {
    unsigned int index   = _writeOffset % _nelements;
    unsigned int lenread = _nelements - index;
    unsigned int indexend;

    if (lenread > _nelements - fill)
      lenread = _nelements - fill;

    lenread = read(_fh, &_data[index][0], lenread * 2) / 2;

    indexend = index + lenread;

    for(;index < indexend; ++index)
    {
      const int8_t i = _data[index][0] - 128;
      const int8_t q = _data[index][1] - 128;

      _data[index][0] = i;
      _data[index][1] = q;
    }


    _writeOffset += lenread;

    if(lenread == 0)
      _done = true;

    if(lenread)
      _queue.queue(this);

    if(_writeOffset != _readOffset)
      _event.signal();
  }
}

void InputFeeder::start()
{
  _queue.queue(this);
}

int InputFeeder::findNextSymbol()
{
  auto  readOffset  = _readOffset;
  auto writeOffset  = _writeOffset;
  auto  sofSymbol   = _sofOffset;

  enum State state = _sofState;

  if(SOF_STATE_PREFILL == state)
  {
    /* prefill the average with enough values to
     * get a reasonable median */
    while(readOffset != writeOffset &&
          readOffset < _nelements)
    {
      updateSignalLevels(readOffset);
      readOffset++;
    }

    if(readOffset >= _navgshort)
    {
      state = SOF_STATE_INIT;
    }
  }

  if(state == SOF_STATE_INIT)
  {
    /* wait for first pause in data stream */

    while(readOffset != writeOffset)
    {
      const unsigned long med_long  = _avg_long  / _nelements;
      const unsigned long med_short = _avg_short / _navgshort;

      if((med_short * 4) < med_long)
      {
        break;
      }
      else
      {
        updateSignalLevels(readOffset);
        readOffset++;
      }
    }

    if(readOffset != writeOffset)
      state = SOF_STATE_PAUSE;
  }

  if(SOF_STATE_PAUSE == state)
  {
    /* wait for start of next frame */
    while(readOffset != writeOffset)
    {
      const unsigned long med_long  = _avg_long  / _nelements;
      const unsigned long med_short = _avg_short / _navgshort;

      if(med_short > med_long)
      {
        break;
      }
      else
      {
        updateSignalLevels(readOffset);
        readOffset++;
      }
    }

    /* check if start of frame was found */
    if(readOffset != writeOffset)
    {
      state      = SOF_STATE_FRAME;
      sofSymbol  = readOffset;
      _symbolcnt = 0;
      //printf("SOF_STATE_FRAME %d\n", readOffset);
    }
  }

  int nextsymbol = -1;

  if(state == SOF_STATE_FRAME)
  {
    /* process frame */
    const auto samples = _Prm.getSamplesPerSymbol() + _Prm.getSamplesPerSymbolGuard();

    if (((unsigned int)(writeOffset - sofSymbol)) >= samples)
    {
      nextsymbol = _symbolcnt;
    }
  }

  _sofState   = state;
  _sofOffset  = sofSymbol;
  _readOffset = readOffset;

  return nextsymbol;
}

void InputFeeder::getSymbol(DabSymbol & symbol)
{
  auto  readOffset   = _readOffset;
  auto writeOffset   = _writeOffset;
  auto  sofSymbol    = _sofOffset;
  auto  corPhase     = _corPhase;


  //printf("CorFreq: %d\n", corFreq);


  enum State state = _sofState;

  if(state == SOF_STATE_FRAME)
  {
    const int MaxSampleRate = DabParameterSetReference::getMaxSampleRate();
    int corFreq = _OffsetCorrection.getValue();

    while (corFreq >= MaxSampleRate)
      corFreq -= MaxSampleRate;

    while (corFreq < -MaxSampleRate)
      corFreq += MaxSampleRate;

    /* process frame */
    const auto samples = _Prm.getSamplesPerSymbol() + _Prm.getSamplesPerSymbolGuard();

    if (((unsigned int)(writeOffset - sofSymbol)) >= samples)
    {
      auto pBuffer = symbol.getBuffer();
      auto pCor    = getSineTable();

      if (_symbolcnt == 0)
        corPhase = 0;

      /* a full symbol is available in ringbuffer */
      while(((unsigned int)(readOffset - sofSymbol)) < samples)
      {
        const unsigned long med_long  = _avg_long  / _nelements;
        const unsigned long med_short = _avg_short / _navgshort;

        if((med_short * 4) < med_long)
        {
          state = SOF_STATE_PAUSE;
          break;
        }
        else
        {
          const auto index = readOffset - sofSymbol;

          if(index < samples)
          {
            const int8_t i = _data[readOffset % _nelements][0];
            const int8_t q = _data[readOffset % _nelements][1];

            ::std::complex<float> sample(i,q);

            sample = sample * pCor[corPhase];

            pBuffer[index][0] = sample.real();
            pBuffer[index][1] = sample.imag();
          }

          updateSignalLevels(readOffset);
          readOffset++;

          if ((corPhase + corFreq) >= MaxSampleRate)
            corPhase  = corPhase  + corFreq - MaxSampleRate;
          else if ((corPhase + corFreq) < 0)
            corPhase  = corPhase  + corFreq + MaxSampleRate;
          else
            corPhase  = corPhase  + corFreq;
        }
      }

      symbol.setState(symbol.STATE_SAMPLE);

      _symbolcnt += 1;
      sofSymbol += samples;
    }
  }

  if(_symbolcnt >= _Prm.getSymbolsPerFrame() &&
      state == SOF_STATE_FRAME)
  {
    state = SOF_STATE_INIT;
  }

  _sofState   = state;
  _sofOffset  = sofSymbol;
  _readOffset = readOffset;
}

void InputFeeder::updateOffset(const DabSymbol & Symbol)
{
  float Offset = 0;

  Offset += Symbol.getCoarseOffset();
  Offset += Symbol.getFineOffset();

  _OffsetCorrection.addValue(_OffsetCorrection.getValue() - Offset);
};

::std::complex<float> * InputFeeder::getSineTable()
{
  static ::std::complex<float> * pTable = 0;

  if(!pTable)
  {
    const auto  MaxSampleRate = DabParameterSetReference::getMaxSampleRate();
    const float Factor = 2 * ::std::acos((float)-1) / MaxSampleRate;
    unsigned int i;

    pTable = new ::std::complex<float>[MaxSampleRate];

    for(i = 0; i < MaxSampleRate; ++i)
    {
      pTable[i] = ::std::exp(::std::complex<float>(0, i * Factor));
    }
  }

  return pTable;
}

class FFT : public Job
{
public:
  typedef DabSymbol SymbolType;
private:
  static constexpr float _pi = ::std::acos((float)-1);

  DabParameterSetReference & _Prm;
  fftwf_complex  * const _pOutBuffer;
  SymbolType     *       _pSymbol;

  fftwf_plan     _plan;

  volatile bool  _running;

  Queue & _queue;
  Event & _event;

  static Mutex & getFFTWMutex();
public:
  FFT(DabParameterSetReference & Prm, Queue & queue, Event & event);
  virtual ~FFT();

  virtual void run();

  void start(SymbolType * pSymbol);
  bool running();
};

Mutex & FFT::getFFTWMutex()
{
  static Mutex mutex;
  return mutex;
}

FFT::FFT(DabParameterSetReference & Prm, Queue & queue, Event & event):
    _Prm(Prm),
    _pOutBuffer(static_cast<fftwf_complex*>(fftwf_malloc(sizeof(fftwf_complex[_Prm.getMaxSamples()])))),
    _plan(0),
    _running(false),
    _queue(queue),
    _event(event)
{
}

FFT::~FFT()
{
  if(_plan)
  {
    MutexLocker lock(getFFTWMutex());
    fftwf_destroy_plan(_plan);
  }
}

void FFT::run()
{
  const auto SamplesPerSymbolGuard = _Prm.getSamplesPerSymbolGuard();
  const auto SamplesPerSymbol      = _Prm.getSamplesPerSymbol();
  const auto NumCarriers           = _Prm.getNumCarriers();

  if(!_plan)
  {
    MutexLocker lock(getFFTWMutex());

    fftwf_complex *pDummy = static_cast<fftwf_complex*>(fftwf_malloc(sizeof(fftwf_complex[_Prm.getMaxSamples()])));

    _plan = fftwf_plan_dft_1d(SamplesPerSymbol, pDummy, _pOutBuffer,-1,0);
    fftwf_free(pDummy);
  }

  const float FreqPerSlot = (float)_Prm.getSampleRate() / SamplesPerSymbol;
  float CoarseOffset = 0;
  float FineOffset = 0;

  {
    const auto *pInBuffer = _pSymbol->getBuffer();
    unsigned int i,j;

    for(i=0, j = SamplesPerSymbol; i < SamplesPerSymbolGuard; ++i, ++j)
    {
      ::std::complex<float> a = {pInBuffer[i][0] , pInBuffer[i][1]};
      ::std::complex<float> b = {pInBuffer[j][0] , pInBuffer[j][1]};

      FineOffset += ::std::arg(a*b);
    }

    FineOffset = FreqPerSlot * FineOffset / i / _pi / 2;
  }

  fftwf_execute_dft(_plan, _pSymbol->getBuffer(), _pOutBuffer);

  { /* mapp fft data to carriers */
    auto pBuffer      = _pSymbol->getBuffer();
    auto MappingTable = _Prm.getMappingTable();

    unsigned int i;

    for(i = 0; i < NumCarriers; ++i)
    {
      pBuffer[i][0] = _pOutBuffer[MappingTable[i]][0];
      pBuffer[i][1] = _pOutBuffer[MappingTable[i]][1];
    }
  }

  { /* calculate channel offset */
    float sum = 0, maxsum = 0;
    unsigned int i,j, maxj;

    maxsum = sum;
    maxj   = (SamplesPerSymbol / 2);

    for(i = SamplesPerSymbol / 2; i < SamplesPerSymbol; ++i)
    {
      sum += _pOutBuffer[i][0] * _pOutBuffer[i][0] + _pOutBuffer[i][1] * _pOutBuffer[i][1];
    }

    for(i = 1; (SamplesPerSymbol / 2 + i - 1) < NumCarriers; ++i)
    {
      sum += _pOutBuffer[i][0] * _pOutBuffer[i][0] + _pOutBuffer[i][1] * _pOutBuffer[i][1];
    }


    for(j = (SamplesPerSymbol / 2); i < (SamplesPerSymbol / 2); ++i, ++j)
    {
      sum -= _pOutBuffer[j][0] * _pOutBuffer[j][0] + _pOutBuffer[j][1] * _pOutBuffer[j][1];
      sum += _pOutBuffer[i][0] * _pOutBuffer[i][0] + _pOutBuffer[i][1] * _pOutBuffer[i][1];

      if ( sum > maxsum)
      {
        maxsum = sum;
        maxj   = j;
      }
    }

    int ChannelOffset = ((int)maxj - (int)(SamplesPerSymbol) + NumCarriers / 2);

    CoarseOffset = FreqPerSlot * ChannelOffset;
  }


  _pSymbol->setOffsets(CoarseOffset, FineOffset);
  _pSymbol->setState(_pSymbol->STATE_DFT);

  _running = false;
  _event.signal();
}

void FFT::start(SymbolType * pSymbol)
{
  if(!_running)
  {
    _running = true;
    _pSymbol    = pSymbol;
    _queue.queue(this);
  }
}

bool FFT::running()
{
  return _running;
}

class DabManager : Job
{
private:
  DabParameterSetReference _Prm;

  typedef DabSymbol SymbolType;

  SymbolType *_pSymbols;

  class FeederDoneEvent : public Event
  {
  private:
    DabManager & _mgr;
  public:
    FeederDoneEvent(DabManager & mgr) : _mgr(mgr) {}
    virtual ~FeederDoneEvent() {};

    virtual void signal() {_mgr.checkInput(); }
  };

  class FFTDoneEvent : public Event
  {
  private:
    DabManager & _mgr;
  public:
    FFTDoneEvent(DabManager & mgr) : _mgr(mgr) {}
    virtual ~FFTDoneEvent() {};

    virtual void signal() {_mgr.checkFFT(); }
  };

  FeederDoneEvent _evFeed;
  FFTDoneEvent    _evFFT;

  Queue      & _queue;
  InputFeeder  _feeder;
  FFT          _fft[2];

  Semaphore    _done;
  Mutex        _mutex;
public:
  DabManager(Queue & queue, const char* path);
  virtual ~DabManager();

  virtual void run();

  void checkInput();
  void checkFFT();

  void start();
  bool done();
};

DabManager::DabManager(Queue & queue, const char *path) :
    _Prm(_Prm.DAB_MODE3),
    _evFeed(*this),_evFFT(*this),_queue(queue),
    _feeder(_Prm,queue, _evFeed, path),
    _fft{{_Prm, queue,_evFFT}, {_Prm,queue, _evFFT}}
{
  _pSymbols = new SymbolType[_Prm.getSymbolsPerFrame() + 1];
}

DabManager::~DabManager()
{

}

void DabManager::start()
{
  _feeder.start();
}

bool DabManager::done()
{
  return _done.wait(10);
}

void DabManager::checkInput()
{
  _queue.queue(this);
}

void DabManager::checkFFT()
{
  _queue.queue(this);
}

void DabManager::run()
{
  const auto SymbolsPerFrame = _Prm.getSymbolsPerFrame();
  unsigned int i = sizeof(_fft) / sizeof(_fft[0]);

  {
    MutexLocker lock(_mutex);
    unsigned int j;

    for(j = 0; j < SymbolsPerFrame; ++j)
    {
      auto & Symbol = _pSymbols[j];

      if(Symbol.getState() == Symbol.STATE_DFT)
      {
        _feeder.updateOffset(Symbol);
        Symbol.setState(Symbol.STATE_FREQCOR);
      }
    }

    if(_feeder.getSymbolCnt() >= SymbolsPerFrame)
    {
      for(i = 0; i < sizeof(_fft) / sizeof(_fft[0]); ++i)
      {
        if(_fft[i].running())
          break;
      }
    }

    if(i == sizeof(_fft) / sizeof(_fft[0]))
    {
      for(i = 0; i < sizeof(_fft) / sizeof(_fft[0]); ++i)
      {
        if(!_fft[i].running())
        {
          int j = _feeder.findNextSymbol();

          if(j >= 0 && j < SymbolsPerFrame)
          {
            _feeder.getSymbol(_pSymbols[j]);
            _fft[i].start(&_pSymbols[j]);
          }
        }
      }

      _feeder.start();
    }


    if (_feeder.done() && !_fft[0].running() && !_fft[1].running())
      _done.put();
  }
}

int main(int argc, char * argv[])
{
  Queue  queue;
  Worker workers[2] = { queue, queue};


  if(argc > 1)
  {
    DabManager mgr(queue, argv[1]);

    workers[0].start();
    workers[1].start();

    mgr.start();

    while(!mgr.done())
      ;

    workers[0].terminate();
    workers[1].terminate();
  }

  return 0;
}


