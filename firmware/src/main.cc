#include "target/atmega8.hpp"
#include "util/guard.hpp"
#include "util/datatypes.hpp"
#include "algorithm/dft.hpp"
#include "algorithm/cordic.hpp"
#include "util/delay.h"

#include <stdint.h>

class SPI
{
public:
  enum SpiDir {
    SPI_MASTER = _BV(MSTR),
    SPI_SLAVE  = 0,
  };

  enum SpiMode {
    SPI_MODE0 = 0,
    SPI_MODE1 = _BV(CPHA),
    SPI_MODE2 = _BV(CPOL),
    SPI_MODE3 = _BV(CPHA) | _BV(CPOL),
  };

  static void initialize(const enum SpiDir dir, const enum SpiMode mode, const uint16_t usSpeed_kHz)
  {
    uint8_t rate = _BV(SPR1);

    SPCR = dir | mode | _BV(SPE) | rate;
  }

  uint8_t transferByte(uint8_t byte)
  {
    SPDR = byte;

    while(0 == (SPSR & _BV(SPIF)))
      ;

    return SPDR;
  }
};

class SPIMaster : public SPI
{
public:
  static void initialize(const uint8_t mode, uint16_t usSpeed_kHz)
  {
    switch(mode)
    {
    case 0: SPI::initialize(SPI_MASTER, SPI::SPI_MODE0, usSpeed_kHz); break;
    case 1: SPI::initialize(SPI_MASTER, SPI::SPI_MODE1, usSpeed_kHz); break;
    case 2: SPI::initialize(SPI_MASTER, SPI::SPI_MODE2, usSpeed_kHz); break;
    case 3: SPI::initialize(SPI_MASTER, SPI::SPI_MODE3, usSpeed_kHz); break;
    }
  }
};

class SPISlaveA : public SPIMaster
{
public:
  __attribute__((always_inline)) static void    assertChipSelect()     {DDRD |=  _BV(PIN3);_delay_us(10);}
  __attribute__((always_inline)) static void    deassertChipSelect()   {DDRD &= ~_BV(PIN3);_delay_us(10);}

  __attribute__((always_inline)) static void    assertReset()          {DDRD |=  _BV(PIN4);_delay_us(10);}
  __attribute__((always_inline)) static void    deassertReset()        {DDRD &= ~_BV(PIN4);_delay_us(10);}

  __attribute__((always_inline)) static void    assertData_Command()   {DDRD &= ~_BV(PIN2);_delay_us(10);}
  __attribute__((always_inline)) static void    deassertData_Command() {DDRD |= _BV(PIN2);_delay_us(10);}
};

using Platform::Guards::ChipSelectGuard;
using namespace Platform::Util::Datatypes;
using namespace Platform::Algorithm::DFT;
using namespace Platform::Architecture::AVR8;
using ::Platform::Buffer::RamBuffer;
using ::Platform::Algorithm::Cordic;
using ::std::size_t;

template<class Device>
class PCD8544 : public Device
{
private:
  /* test */
  static constexpr uint32_t m_MaxSpeed_Mhz = 400000;

  typedef ChipSelectGuard<Device> CsGuard;

public:
  void initialize()
  {
    Device::initialize(3,m_MaxSpeed_Mhz / 1000);

    this->assertReset();
    _delay_ms(100);
    this->deassertReset();

    this->writeCommand(0x21);
    this->writeCommand(0x14);
    this->writeCommand(0xb0);
    this->writeCommand(0x20);
    this->writeCommand(0x0c);
  }

  void writeData(uint8_t data)
  {
    CsGuard guard(*this);

    this->assertData_Command();
    this->transferByte(data);
  }

  void writeCommand(uint8_t cmd)
  {
    CsGuard guard(*this);

    this->deassertData_Command();
    this->transferByte(cmd);
  }
};

static PCD8544<SPISlaveA> display;

typedef FixedPoint<int16_t, 0x1000> CalculationType;

CalculationType afInputArray[128]; /*  input data array */

typedef Radix2DFT<CalculationType,7> DFT_Type;

static DFT_Type::t_Type dft_buffer[128];

static uint8_t          display_oscilloscope[84];
static uint8_t          display_dft[ElementCount(dft_buffer) / 2];

static const FlashBuffer<DFT_Type::m_bins / 2, DFT_Type::t_Type> dft_factors PROGMEM    = DFT_Type::w<Cordic<> >().asArray();
static const FlashBuffer<DFT_Type::m_bins / 2, uint8_t>          dft_descramble PROGMEM = DFT_Type::descramble().asArray();

static const FlashBuffer<8, uint8_t>                             display_oscilloscopemask PROGMEM  {{0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01}};
static const FlashBuffer<8, uint8_t>                             display_dftmask          PROGMEM  {{0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF}};

template<typename TYPE, size_t ELEMENTS>
class AmplitudeTable
{
public:
  static constexpr size_t m_len = ELEMENTS;

  constexpr AmplitudeTable() {}

  constexpr TYPE operator [] (size_t index) const
  {
    return ((double)ELEMENTS) * Cordic<>::sqrt((double)index / (double)ELEMENTS);
  }
};

template<typename TYPE, size_t SAMPLES>
class HammingWindow
{
public:
  static constexpr size_t m_len = SAMPLES;

  static constexpr double m_alpha = 0.54;
  static constexpr double m_beta  = 1.0 - m_alpha;

  constexpr HammingWindow() {}


  constexpr TYPE operator [] (size_t index) const
  {
    return (m_alpha - m_beta * Cordic<>::cos((2 * Cordic<>::m_pi * index) / (SAMPLES - 1)));
  }
};

using ::Platform::Util::indices;
using ::Platform::Util::build_indices;

template<typename GENERATOR, size_t... Is>
constexpr auto
fromFunction_real(const GENERATOR &gen, indices<Is...>)
-> RamBuffer<indices<Is...>::count(), decltype(gen[0])>
{
  return {{gen[Is]...,}};
}

template<typename GENERATOR, size_t ELEMENTS>
constexpr RamBuffer<ELEMENTS, uint8_t>
fromFunction(const GENERATOR &gen)
{
  return fromFunction_real(gen, build_indices<ELEMENTS>());
}



static const FlashBuffer<256, uint8_t>          amplitude_table PROGMEM  = fromFunction_real(AmplitudeTable<uint8_t,256>(), build_indices<256>());
static const FlashBuffer<128, CalculationType>  hamming_window  PROGMEM  = fromFunction_real(HammingWindow<CalculationType, 128>(), build_indices<128>());

int main()
{
  PORTB = _BV(PIN0) /* | _BV(PIN1) */ /* | _BV(PIN2) */ | _BV(PIN3) | _BV(PIN5);
  PORTD = 0;
  DDRB  = _BV(PIN0) | _BV(PIN1) | _BV(PIN2) | _BV(PIN3) | _BV(PIN5);
  DDRD  = _BV(PIN5) | _BV(PIN6);

  display.initialize();

  static constexpr uint8_t reduction       = 32;
  static constexpr uint8_t decay_reduction = (8 + reduction - 1) / reduction;

  int16_t offset  = 512;
  uint8_t iAdc    = 0;
  uint8_t iDiv    = 0;
  uint8_t iDecay  = 0;
  uint8_t maximumidx = 0;

  ADMUX = _BV(REFS0) | 0x2;
  ADCSR = _BV(ADEN) | _BV(ADFR) | _BV(ADSC) | 0x7;


  while(1)
  {
    /* check if adc done */
    if(ADCSR & _BV(ADIF))
    {
      if (iDiv > 0)
      { /* skip this adc measurement */
        iDiv = iDiv - 1;
      }
      else if (iAdc < ElementCount(afInputArray))
      { /* store measurement and continue */
        iDiv = reduction - 1;

        auto value = CalculationType((int16_t)ADCW - offset, 256);

        afInputArray[iAdc] = value * hamming_window[iAdc];
        iAdc++;


      }

      ADCSR |= _BV(ADIF);
    }

    if (iAdc >= ElementCount(afInputArray))
    {
      uint8_t uIdx;
      uint8_t uCol = 0;

      for(uIdx = 0; uIdx < ElementCount(afInputArray); ++uIdx)
      {
        if(uIdx < ElementCount(display_oscilloscope))
        {
          display_oscilloscope[uIdx] = (FixedPoint<int8_t, 1>(8) * afInputArray[uIdx]) + FixedPoint<int8_t, 1>(8);
        }

        dft_buffer[uIdx] = afInputArray[uIdx];
      }

      while(uIdx < ElementCount(display_oscilloscope))
        display_oscilloscope[uIdx++] = 0x00;

      for(uCol = 0; uCol < 84; ++uCol)
      {
        uint8_t value = display_oscilloscope[uCol] - 8;

        if (value < 8)
          display.writeData(display_oscilloscopemask[value]);
        else
          display.writeData(0x00);
      }

      for(uCol = 0; uCol < 84; ++uCol)
      {
          uint8_t value = display_oscilloscope[uCol];

          if (value < 8)
            display.writeData(display_oscilloscopemask[value]);
          else
            display.writeData(0x00);
      }

      DFT_Type::decimation_in_f(dft_buffer, dft_factors);


      if (iDecay > 0)
        iDecay = iDecay - 1;
      else
        iDecay = decay_reduction;

      const auto offset_correction = dft_buffer[dft_descramble[0]].real();

      if(offset_correction > 0)
          offset++;
      else if (offset_correction < 0)
          offset--;

      uint16_t uAccumulator = 0;
      uint8_t max           = 0;

      maximumidx = maximumidx /2 + maximumidx /4;

      for(uIdx = 0; uIdx < ElementCount(display_dft); uIdx++)
      {
        auto value = dft_buffer[dft_descramble[uIdx]];
        value = value * value.conjugate();

        uint8_t amplitude = amplitude_table[FixedPoint<int16_t, 1>(256) * (value.real())];

        uAccumulator += ((uint16_t)amplitude * uIdx)/ 4;

        uint8_t peak = display_dft[uIdx];

        if (0 == iDecay && peak)
          peak = (peak / 2) + (peak / 4);

        if (amplitude > peak)
          peak = amplitude;

        display_dft[uIdx] = peak;

        if (amplitude > max && uIdx > maximumidx)
        {
          max = amplitude;
          maximumidx = uIdx;
        }
      }

      if (uAccumulator > 256)
        PORTD |= _BV(PIN6);
      else
        PORTD &= ~_BV(PIN6);

      uint8_t range = 32;

      while(range)
      {
        range -= 8;

        for(uCol = 0; uCol < ElementCount(display_oscilloscope); uCol++)
        {
          uint8_t display_data = 0x00;

          if(uCol < ElementCount(display_dft))
          {
            const uint8_t value = display_dft[uCol] >> 1;

            if (value >= range + 8)
              display_data = 0xFF;
            else if (value >= range)
              display_data = display_dftmask[value-range];
          }

          if(uCol <= maximumidx && range == 24)
            display_data |= 0x3;

          if(uCol > 80 && range == 24 && uAccumulator >= 256)
            display_data |= 0x3;

          display.writeData(display_data);
        }
      }

      /* s sdrestart adc */
      iAdc = 0;
    }
  }
}
