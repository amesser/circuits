/*
 *  Copyright 2013 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of USB Display firmware.
 *
 *  This software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *  */
#include <target/attiny2313.hpp>
#include <generic/buffer.hpp>
#include <util/datatypes.hpp>
#include <protocols/usb/usb.hpp>
#include <protocols/usb/descriptor.hpp>
#include <protocols/usb/hid.hpp>

#include <util/delay.h>

using namespace Platform::Architecture::AVR8;
using namespace Platform::Target::AVR8;
using namespace Platform::Buffer;
using namespace Platform::Util::Datatypes;
using namespace Platform::Protocols::USB;



static constexpr auto repDescriptor PROGMEM = join(
  UsagePage(0x14), /* alphanumeric display */
  Usage(AlphanumericDisplay),
  Collection(Application),

    ReportCount(1),

    Usage(DisplayAttributesReport),
    Collection(Logical),
      ReportID(1),

      LogicalMaximum(31),

      Usage(Columns),
      ReportSize(5),
      Feature(Constant, Variable, Absolute),

      Usage(Rows),
      Feature(Constant, Variable, Absolute),

      Feature(Constant, Variable, Absolute),

      LogicalMaximum(1),

      Usage(ASCIICharacterSet),
      ReportSize(1),
      Feature(Constant, Variable, Absolute),

    EndCollection(),

    Usage(CursorPositionReport),
    Collection(Logical),
      ReportID(2),

      LogicalMaximum(31),

      Usage(Column),
      ReportSize(6),
      Feature(Data, Variable, Absolute, NoPreferredState),

      Usage(Row),
      ReportSize(1),
      Feature(Data, Variable, Absolute, NoPreferredState),

      ReportSize(1),
      Feature(Constant, Variable, Absolute),
    EndCollection(),

    ReportSize(8),

    LogicalMaximum(1),
    Usage(DisplayStatus),
    Collection(Logical),
      Usage(StatNotReady),
      Usage(StatReady),
      Input(Data,Array, Absolute, NoNullPosition),
    EndCollection(),

    ReportCount(4),

    Usage(CharacterReport),
    Collection(Logical),
      ReportID(3),
      Usage(DisplayData),
      Feature(Data, Variable, Absolute, BufferedBytes),
    EndCollection(),

  EndCollection()
).in<FlashBuffer>();



const static auto devDescriptor PROGMEM =
    DeviceDescriptor(0x0110, 0, 0, 0, 8, 0xFFFF, 0xFFFF, 0, 0x1).in<FlashBuffer>();

const static auto cfgDescriptor PROGMEM =
    join(ConfigurationDescriptor(sizeof(ConfigurationDescriptor) +
        sizeof(InterfaceDescriptor) + sizeof(EndpointDescriptor) +
        sizeof(HIDDescriptor), 1, 1, 0,
          CONFIGURATION_ATTRIBUTE_USBB10_BUSPOWERED, 100),
         InterfaceDescriptor(0,0,1,0x3,0,0,0),
         HIDDescriptor(0x01b, 0x00, 0x01, 0x22,  sizeof(repDescriptor)),
         EndpointDescriptor(1 | ENDPOINT_DIRECTION_IN, ENDPOINT_ATTRIBUTE_TRANSFERTYPE_INTERRUPT,
             64, 100)
        ).in<FlashBuffer>();


static constexpr uint8_t registers[] =
{
    _SFR_IO_ADDR(PORTA),  0x01,
    _SFR_IO_ADDR(PORTB),  0x00,
    _SFR_IO_ADDR(PORTD),  0x00,
    _SFR_IO_ADDR(DDRA),   0x03,
    _SFR_IO_ADDR(DDRB),   0xDF,
    _SFR_IO_ADDR(DDRD),   0x0F,
    _SFR_IO_ADDR(OCR1BL), 0,
    _SFR_IO_ADDR(TCCR1A), (0x2 << COM1B0) | (0x1 << WGM10),
    _SFR_IO_ADDR(TCCR1B), (0x1 << WGM12) |  (0x3 << CS10),

};

static constexpr auto registerInit PROGMEM =
    FlashBuffer<sizeof(registers)>(registers);

static void
trigLed()
{
  uint8_t val = OCR1BL;

  if (val == 0xFF || 0 == val)
    val = 0xF;
  else
    val = val*2 + 1;

  OCR1BL = val;
}

static void lcd_delay_clock()
{
  _delay_us(1);
}

static void lcd_enable()
{
  PORTA |= (1 << PIN1);
  lcd_delay_clock();
  PORTA &= ~(1 << PIN1);
  lcd_delay_clock();
}

static uint8_t lcd_read()
{
  uint8_t work;

  DDRD  &= 0xF0;
  PORTD &= 0xF0;

  PORTB |= (1 << PIN1);

  lcd_delay_clock();

  PORTA |= (1 << PIN1);
  lcd_delay_clock();
  work   = PIND;
  PORTA &= ~(1 << PIN1);

  work = work << 4;

  lcd_delay_clock();
  PORTA |= (1 << PIN1);
  lcd_delay_clock();
  work  |= (PIND & 0xF);
  PORTA &= ~(1 << PIN1);
  lcd_delay_clock();

  return work;
}

static void lcd_write(uint8_t value)
{
  uint8_t work;

  PORTB &= ~(1 << PIN1);
  lcd_delay_clock();

  DDRD  |= 0x0F;

  work  = (PORTD & 0xF0) | (value >> 4);
  PORTD = work;

  lcd_enable();

  work  = (PORTD & 0xF0) | (value & 0x0F);
  PORTD = work;

  lcd_enable();
}

static void lcd_write_ins(uint8_t value)
{
  PORTB &= ~(1 << PIN0);
  lcd_write(value);
  while(lcd_read() & 0x80);
}

static void lcd_write_data(uint8_t value)
{
  PORTB |= (1 << PIN0);
  lcd_write(value);
}

static void lcd_init()
{
  _delay_ms(20);
  PORTD = (PORTD & 0xF0) | 0x3;
  lcd_enable();
  _delay_ms(5);
  lcd_enable();
  _delay_ms(5);
  lcd_enable();
  _delay_ms(5);
  PORTD = (PORTD & 0xF0) | 0x2;
  lcd_enable();
  _delay_ms(5);
  lcd_write_ins(0x28);
  lcd_write_ins(0x08);
  lcd_write_ins(0x06);
  lcd_write_ins(0x17);
  lcd_write_ins(0x01);
  lcd_write_ins(0x02);
  lcd_write_ins(0x0C);
}

/* blub */
class SPISlaveA
{
public:
  __attribute__((always_inline)) static void selectChip()         {PORTB  &= ~_BV(PIN2);}
  __attribute__((always_inline)) static void deselectChip()       {PORTB  |= _BV(PIN2);}

  static USISPIMaster<15000000> getTransport(const unsigned long speed = 1500000) { return {}; }
};

template<typename DEVICE>
class ChipSelectGuard
{
private:
  const DEVICE & _device;

public:
  ChipSelectGuard(DEVICE & device) : _device{device}
  {
    _device.selectChip();
  };

  ~ChipSelectGuard()
  {
    _device.deselectChip();
  }
};

template<typename DEVICE>
ChipSelectGuard<DEVICE> ChipSelect(DEVICE & device)
{
  return {device};
}

template<template <size_t, typename> class Buffer>
class RegisterAccess : public Buffer<2, uint8_t>
{
public:
  typedef Buffer<2, uint8_t> BufferType;
  constexpr RegisterAccess(uint8_t reg, uint8_t val) : Buffer<2, uint8_t>{{reg, val}} {}

  template<template <size_t, typename> class InitBuffer>
  constexpr RegisterAccess(RegisterAccess<InitBuffer> const & init) : Buffer<2, uint8_t>{{init.reg(), init.val()}} {}

  constexpr uint8_t reg() const {return this->at(0);}
  constexpr uint8_t val() const {return this->at(1);}
};

template<size_t Count, template <size_t, typename> class Buffer = RamBuffer>
struct RegisterSequence
{
  template<template<size_t, typename> class BufferType>
  using OtherBuffer = RegisterSequence<Count, BufferType>;

  typedef RegisterAccess<Buffer> ElementType;

  template<template<size_t, typename> class BufferType, size_t ...Is>
  constexpr RegisterSequence(RegisterAccess<BufferType> const (&init)[Count], indices<Is...>) : _sequence{init[Is]...} {}
  template<template<size_t, typename> class BufferType = RamBuffer>
  constexpr RegisterSequence(RegisterAccess<BufferType> const (&init)[Count]) : RegisterSequence(init, build_indices<Count>{}) {}

  template<template<size_t, typename> class BufferType>
  constexpr RegisterSequence(RegisterSequence<Count, BufferType> const & init) : RegisterSequence(init._sequence) {}

  static constexpr size_t count() {return Count;}
  ElementType _sequence[Count];

  const ElementType & operator [] (size_t idx) const {return this->_sequence[idx];}

  typename ElementType::const_iterator begin() const {return _sequence[0].begin();}
  typename ElementType::const_iterator end()   const {return _sequence[Count-1].end();}
};

enum {
  GET_STATUS        = 0,
  SET_ADDRESS       = 5,
  GET_DESCRIPTOR    = 6,
  SET_CONFIGURATION = 9,
  SET_INTERFACE     = 10,
};

enum {
  GET_REPORT        = 1,
  SET_REPORT        = 9,
};

template<typename DEVICE, template<size_t, typename> class Storage>
class USBN960X : public DEVICE
{
public:
  enum {
    SPI_CMD_READ         = 0 << 6,
    SPI_CMD_NOP          = 1 << 6,
    SPI_CMD_WRITE        = 2 << 6,
    SPI_CMD_BURST_WRITE  = 3 << 6,
  };

  enum {
    REG_MCNTRL = 0x00,
    REG_CCONF  = 0x01,
    REG_RID    = 0x03,
    REG_FAR    = 0x04,
    REG_NFSR   = 0x05,
    REG_MAEV   = 0x06,
    REG_MAMSK  = 0x07,
    REG_ALTEV  = 0x08,
    REG_ALTMSK = 0x09,
    REG_TXEV   = 0x0A,
    REG_TXMSK  = 0x0B,
    REG_RXEV   = 0x0C,
    REG_RXMSK  = 0x0D,
    REG_NAKEV  = 0x0E,
    REG_NAKMSK = 0x0F,
    REG_FWEV   = 0x10,
    REG_FWMSK  = 0x11,
    REG_FNH    = 0x12,
    REG_FNL    = 0x13,
    REG_DMACNTRL = 0x14,
    REG_DMAEV    = 0x15,
    REG_DMAMSK   = 0x16,
    REG_MIR      = 0x17,
    REG_DMACNT   = 0x18,
    REG_DMAERR   = 0x19,
    REG_WKUP     = 0x1B,
    REG_EPC0     = 0x20,
    REG_TXD0     = 0x21,
    REG_TXS0     = 0x22,
    REG_TXC0     = 0x23,
    REG_RXD0     = 0x25,
    REG_RXS0     = 0x26,
    REG_RXC0     = 0x27,
    REG_EPC1     = 0x28,
    REG_TXD1     = 0x29,
    REG_TXS1     = 0x2A,
    REG_TXC1     = 0x2B,
    REG_EPC2     = 0x2C,
    REG_RXD1     = 0x2D,
    REG_RXS1     = 0x2E,
    REG_RXC1     = 0x2F,
    REG_EPC3     = 0x30,
    REG_TXD2     = 0x31,
    REG_TXS2     = 0x32,
    REG_TXC2     = 0x33,
    REG_EPC4     = 0x34,
    REG_RXD2     = 0x35,
    REG_RXS2     = 0x36,
    REG_RXC2     = 0x37,
    REG_EPC5     = 0x38,
    REG_TXD3     = 0x39,
    REG_TXS3     = 0x3a,
    REG_TXC3     = 0x3b,
    REG_EPC6     = 0x3c,
    REG_RXD3     = 0x3d,
    REG_RXS3     = 0x3e,
    REG_RXC3     = 0x3f,
  };

  enum {
    REG_DELTA_EPC = 0,
    REG_DELTA_TXD = 1,
    REG_DELTA_TXS = 2,
    REG_DELTA_TXC = 3,
    REG_DELTA_RXD = 5,
    REG_DELTA_RXS = 6,
    REG_DELTA_RXC = 7,
  };

  enum {
    MCNTRL_SRST_SRT  = 0,
    MCNTRL_VGE_SRT   = 2,
    MCNTRL_NAT_SRT   = 3,
    MCNTRL_INTOC_SRT = 6,
  };

  enum {
    CCONF_CLKDIV_SRT  = 0,
    CCONF_CODIS_SRT   = 7,
  };

  enum {
    NFS_RESET         = 0x00,
    NFS_RESUME        = 0x01,
    NFS_OPERATIONAL   = 0x02,
    NFS_SUSPEND       = 0x03
  };

  enum {
    MAEV_WARN_SRT     = 0,
    MAEV_ALT_SRT      = 1,
    MAEV_TX_EV_SRT    = 2,
    MAEV_FRAME_SRT    = 3,
    MAEV_NAK_SRT      = 4,
    MAEV_ULD_SRT      = 5,
    MAEV_RX_EV_SRT        = 6,
    MAEV_INTR_SRT     = 7,
  };
  enum {
    EPC_EP_EN       = 4,
    EPC_ISO         = 5,
    EPC_DEF_SRT     = 6,
    EPC_STALL_SRT   = 7,
  };

  enum {
    FAR_ADEN_SRT    = 7,
  };

  enum {
    ALT_WKUP_SRT   = 1,
    ALT_DMA_SRT    = 2,
    ALT_EOP_SRT    = 3,
    ALT_SD3_SRT    = 4,
    ALT_SD5_SRT    = 5,
    ALT_RESET_SRT  = 6,
    ALT_RESUME_SRT = 7,
  };

  enum {
    NAKEV_IN0_SRT  = 0,
    NAKEV_IN1_SRT  = 1,
    NAKEV_IN2_SRT  = 2,
    NAKEV_IN3_SRT  = 3,
    NAKEV_OUT0_SRT = 4,
    NAKEV_OUT1_SRT = 5,
    NAKEV_OUT2_SRT = 6,
    NAKEV_OUT3_SRT = 7,
  };

  enum {
    TXC0_IGN_IN_SRT    = 4,
  };

  enum {
    TXC_TX_EN_SRT      = 0,
    TXC_LAST_SRT       = 1,
    TXC_TOGGLE_SRT     = 2,
    TXC_FLUSH_SRT      = 3,
    TXC_RFF_SRT        = 4,
    TXC_TWFL_SRT       = 5,
    TXC_IGN_ISOMSK_SRT = 7,
  };

  enum {
    RXC_RX_EN_SRT      = 0,
    RXC_IGN_OUT_SRT    = 1,
    RXC_IGN_SETUP      = 2,
    RXC_FLUSH_SRT      = 3,
    RXC_TWFL_SRT       = 5,
  };

  enum {
    RXEV_FIFO0_SRT     = 0,
    RXEV_RXFIFO1_SRT   = 1,
    RXEV_RXFIFO2_SRT   = 2,
    RXEV_RXFIFO3_SRT   = 3,
    RXEV_OVRRN0_SRT    = 4,
    RXEV_RXOVRRN1_SRT  = 5,
    RXEV_RXOVRRN2_SRT  = 6,
    RXEV_RXOVRRN3_SRT  = 7,
  };

  enum {
    TXEV_FIFO0_SRT     = 0,
    TXEV_TXFIFO1_SRT   = 1,
    TXEV_TXFIFO2_SRT   = 2,
    TXEV_TXFIFO3_SRT   = 3,
    TXEV_UDRRN0_SRT    = 4,
    TXEV_TXUDRRN1_SRT  = 5,
    TXEV_TXUDRRN2_SRT  = 6,
    TXEV_TXUDRRN3_SRT  = 7,
  };

  enum {
    RXS_RCOUNT_SRT     = 0,
    RXS_RX_LAST        = 4,
    RXS_TOGGLE_SRT     = 5,
    RXS_SETUP          = 6,
    RXS_ERR            = 7,
  };

  enum {
    TXS_TCOUNT_SRT     = 0,
    TXS_TX_DONE_SRT    = 5,
    TXS_ACK_STAT_SRT   = 6,
    TXS_URUN_SRT       = 7,
  };
private:
  struct USBN960X_Sequence_Reset : public RegisterSequence<5>
  {
    constexpr USBN960X_Sequence_Reset() : RegisterSequence<5> {{
      {REG_NFSR, NFS_RESET},
      {REG_FAR, 1 << FAR_ADEN_SRT},
      {REG_EPC0, 0},
      {REG_TXC0, 1 << TXC_FLUSH_SRT},
      {REG_RXC0, 1 << RXC_FLUSH_SRT},
    }}{}
  };

  struct USBN960X_Sequence_Suspend : public RegisterSequence<2>
  {
    constexpr USBN960X_Sequence_Suspend() : RegisterSequence<2> {{
      {REG_ALTMSK, (1 << ALT_RESUME_SRT) |
                   (1 << ALT_RESET_SRT)},
      {REG_NFSR, NFS_SUSPEND},
    }}{}
  };

  struct USBN960X_Sequence_Resume : public RegisterSequence<2>
  {
    constexpr USBN960X_Sequence_Resume() : RegisterSequence<2> {{
      {REG_ALTMSK, (1 << ALT_SD3_SRT) |
                   (1 << ALT_RESET_SRT)},
      {REG_NFSR, NFS_OPERATIONAL},
    }}{}
  };

  struct USBN960X_Sequence_Init : public RegisterSequence<7>
  {
    constexpr USBN960X_Sequence_Init() : RegisterSequence<7> {{
      {REG_NAKMSK, (1 << NAKEV_OUT0_SRT)},
      {REG_RXMSK, (1 << RXEV_FIFO0_SRT)   | (1 << RXEV_RXFIFO1_SRT) |
                  (1 << RXEV_RXFIFO2_SRT) | (1 << RXEV_RXFIFO3_SRT)},
      {REG_TXMSK, (1 << TXEV_FIFO0_SRT)   | (1 << TXEV_TXFIFO1_SRT) |
                  (1 << TXEV_TXFIFO2_SRT) | (1 << TXEV_TXFIFO3_SRT)},
      {REG_ALTMSK, (1 << ALT_SD3_SRT) |
                   (1 << ALT_RESET_SRT)},
      {REG_MAMSK, (1 << MAEV_NAK_SRT)     | (1 << MAEV_ALT_SRT) |
                  (1 << MAEV_TX_EV_SRT)   | (1 << MAEV_RX_EV_SRT) |
                  (1 << MAEV_INTR_SRT) },
      {REG_NFSR, NFS_OPERATIONAL},
      {REG_MCNTRL,(1 << MCNTRL_VGE_SRT) | (1 << MCNTRL_NAT_SRT) |
                  (3 << MCNTRL_INTOC_SRT)},
    }}{}
  };

  struct USBN960X_Sequence_Configuration : public RegisterSequence<2>
  {
    constexpr USBN960X_Sequence_Configuration() : RegisterSequence<2> {{
      {REG_TXC1, 1 << TXC_FLUSH_SRT},
      {REG_EPC1, (1 << EPC_EP_EN) + 1},
    }}{}
  };

  static constexpr RegisterSequence<USBN960X_Sequence_Reset::count(), Storage> const & reset() {return
      ConstantArrayBuffer<USBN960X_Sequence_Reset, Storage>::value;}

  static constexpr RegisterSequence<USBN960X_Sequence_Suspend::count(), Storage> const & suspend() {return
      ConstantArrayBuffer<USBN960X_Sequence_Suspend, Storage>::value;}

  static constexpr RegisterSequence<USBN960X_Sequence_Resume::count(), Storage> const & resume() {return
      ConstantArrayBuffer<USBN960X_Sequence_Resume, Storage>::value;}

  static constexpr RegisterSequence<USBN960X_Sequence_Init::count(), Storage> const & initSequence() {return
      ConstantArrayBuffer<USBN960X_Sequence_Init, Storage>::value;}

  static constexpr RegisterSequence<USBN960X_Sequence_Configuration::count(), Storage> const & configSequence() {return
      ConstantArrayBuffer<USBN960X_Sequence_Configuration, Storage>::value;}

  typedef decltype(devDescriptor)::const_iterator IteratorType;

  IteratorType      it;      /*blub */
  IteratorType      itend;   /*blub */
  uint8_t           txtoggle;

  template<typename Iterator>
  void writeRegisters(Iterator it, Iterator end)
  {
    while(it != end)
    {
        uint8_t reg = *(it++);
        uint8_t val = *(it++);
        write(reg, val);
    }
  }

  template<class Sequence>
  void writeRegisters(Sequence const & seq)
  {
    writeRegisters(seq.begin(), seq.end());
  }


public:

  uint8_t transfer_2bytes(uint8_t addr, uint8_t value)
  {
    auto transport = this->getTransport();

    transport.transferByte(addr);
    return transport.transferByte(value);
  }

  uint8_t read(uint8_t addr)
  {
    auto guard = ChipSelect(*this);
    return transfer_2bytes((addr & 0x3F) | SPI_CMD_READ, SPI_CMD_NOP);
  }

  uint8_t write(uint8_t addr, uint8_t value)
  {
    auto guard = ChipSelect(*this);
    return transfer_2bytes((addr & 0x3F) | SPI_CMD_WRITE, value);
  }

  void handleNodeReset()
  {
    writeRegisters(reset());
    writeRegisters(resume());
  }

  void handleNodeSuspend()
  {
    writeRegisters(suspend());
  }

  void handleNodeResume()
  {
    writeRegisters(resume());
  }

  void eventALT()
  {
    auto altev = read(REG_ALTEV);

    if(altev & (1 << ALT_RESET_SRT))
      handleNodeReset();
    else if(altev & (1 << ALT_SD3_SRT))
      handleNodeSuspend();
    else if (altev & (1<< ALT_RESUME_SRT))
      handleNodeResume();
  }

  void  eventNAK()
  {
    read(REG_NAKEV);
  }

  enum {
    STATE_WSETUP      = 0,
    STATE_CTRL_NODATA,
    STATE_CTRL_READ,
    STATE_CTRL_READ_DONE,
    STATE_CTRL_WRITE,
  };

  uint8_t state;
  void fillTX0()
  {
    /* check if there is data to send or if we shall queue
     * at least one packet (null packet) */
    if (state == STATE_CTRL_READ)
    {
      uint8_t count = 8;

      while(it != itend && count)
      {
        write(REG_TXD0, *(it++));
        count -= 1;
      }

      /* clear the force send bit on the last not full size packet
       * we sent. we need this if the host requests more data than we
       * can deliver */
      if (count)
        txtoggle &= ~(1 << TXC_TX_EN_SRT);

      /* last packet to send, go to next state */
      if (it == itend && (0 == (txtoggle & (1 << TXC_TX_EN_SRT))))
        state = STATE_CTRL_READ_DONE;

      /* activate transmitter */
      write(REG_TXC0, txtoggle | (1 << TXC_TX_EN_SRT));
      txtoggle ^= (1 << TXC_TOGGLE_SRT);
    }
    else if (state == STATE_CTRL_NODATA)
    { /* queue a zero length response */
      write(REG_TXC0, txtoggle | (1 << TXC_TX_EN_SRT));
      state = STATE_WSETUP;
    }
  };

  void stall(uint8_t epc)
  {
    uint8_t val = read(epc) | (1 << EPC_STALL_SRT);
    write(epc, val);
  }

  void unstall(uint8_t epc)
  {
    uint8_t val = read(epc) & (~(1 << EPC_STALL_SRT));
    write(epc, val);
  }

  void eventRX()
  {
    auto rxev = read(REG_RXEV);

    if (rxev & (1 << RXEV_FIFO0_SRT))
    {
      auto status = read(REG_RXS0);

      if (0 == (status & (1<< RXS_RX_LAST)))
      { /* should never happen */
      }
      else if (status & (1<< RXS_SETUP))
      {
          state = STATE_CTRL_NODATA;

          it       = itend;

          txtoggle = (1 << TXC_TOGGLE_SRT);

          uint8_t bmRequestType = read(REG_RXD0);
          uint8_t bRequest      = read(REG_RXD0);

          uint16_t wValue  = read(REG_RXD0) | (read(REG_RXD0) << 8);
          uint16_t wIndex  = read(REG_RXD0) | (read(REG_RXD0) << 8);
          uint16_t wLength = read(REG_RXD0) | (read(REG_RXD0) << 8);

          unstall(REG_EPC0);

          write(REG_TXC0, 1 << TXC_FLUSH_SRT);

          if (0 == (bmRequestType & 0x7F))
          { /* standard device request */
              switch(bRequest)
              {
              case GET_DESCRIPTOR:
                if (wValue == 0x0100)
                {
                  it       = devDescriptor.begin();
                  itend    = devDescriptor.end();
                }
                else if (wValue == 0x0200)
                {
                  it       = cfgDescriptor.begin();
                  itend    = cfgDescriptor.end();
                }
                else
                {
                  stall(REG_EPC0);
                }
                break;
              case SET_ADDRESS:
                write(REG_EPC0, 1 << EPC_DEF_SRT);
                write(REG_FAR,  (1 << FAR_ADEN_SRT) | (wValue & 0xFF));
                break;
              case SET_CONFIGURATION:
                writeRegisters(configSequence());
                break;
              default:
                stall(REG_EPC0);
                break;
              }
          }
          else if (1 == (bmRequestType & 0x7F))
          {  /* standard interface request */
              switch(bRequest)
              {
              case SET_INTERFACE:
                break;
              case GET_DESCRIPTOR:
                if (wValue == 0x2100)
                {
                  it       = cfgDescriptor.begin() + sizeof(ConfigurationDescriptor)
                      + sizeof(InterfaceDescriptor) + sizeof(EndpointDescriptor);
                  itend    = it + sizeof(HIDDescriptor);
                }
                else if (wValue == 0x2200)
                {
                  it       = repDescriptor.begin();
                  itend    = repDescriptor.end();
                }
                else
                {
                  stall(REG_EPC0);
                }
                break;
              default:
                stall(REG_EPC0);
                break;
              }
          }
          else if (2 == (bmRequestType & 0x7F))
          {  /* standard endpoint request */
              switch(bRequest)
              {
              case GET_STATUS:
                write(REG_TXD0, 0);
                write(REG_TXD0, 0);
                break;
              default:
                stall(REG_EPC0);
                break;
              }
          }
          else if (0x21 == (bmRequestType & 0x7F))
          { /* class specific interface request */
            switch(bRequest)
            {
            case GET_REPORT:
              if (0x0301 == wValue)
              { /* report feature id 1 */
                  const uint16_t val = 16 + (2 << 5) + (1 << 15);
                  write(REG_TXD0, 1);
                  write(REG_TXD0, val & 0xFF);
                  write(REG_TXD0, (val >> 8) & 0xFF);
                  state = STATE_CTRL_READ;
              }
              else
              {
                  stall(REG_EPC0);
              }
              break;
            case SET_REPORT:
              if (0x0300 == (wValue & 0xFF00))
              { /* set feature, disable in token, wait for next out tokens */
                  state = STATE_CTRL_WRITE;
              }
              else
              {
                  stall(REG_EPC0);
              }
              break; /* reenable receiver, report data will following */
            default:
              stall(REG_EPC0);
              break;
            }
          }

          if (itend == it)
          {
            ;
          }
          else if ((it + wLength) > itend)
          { /* generate short / null packet */
            txtoggle |= (1 << TXC_TX_EN_SRT);
            state = STATE_CTRL_READ;
          }
          else
          { /* limit amount of send data to requested amount */
            itend = it + wLength;
            state = STATE_CTRL_READ;
          }

          if (state == STATE_CTRL_WRITE)
          {
            write(REG_RXC0, (1 << RXC_RX_EN_SRT));
          }
          else
          {
            fillTX0();
          }
      }
      else if (state == STATE_CTRL_READ_DONE)
      { /* ack from host */
          state = STATE_WSETUP;
      }
      else if (state == STATE_CTRL_WRITE)
      { /* test *wurst */

        txtoggle = (1 << TXC_TOGGLE_SRT) |
                   (1 << TXC_TX_EN_SRT);

        if (status & 0xF)
        {
          uint8_t reportid = read(REG_RXD0);

          switch(reportid)
          {
          case 2:
            {
            uint8_t loc = read(REG_RXD0);
            lcd_write_ins(0x80 | loc);
            break;
            }
          case 3:
            {
              uint8_t count = 4;
              while(count--)
                lcd_write_data(read(REG_RXD0));
            }
            break;
          }
        }

        /* flush any data */
        write(REG_TXC0, 1 << RXC_FLUSH_SRT);

        /* reuse NODATA queue in token for ack */
        state = STATE_CTRL_NODATA;

        fillTX0();
      }


    }
  }

  void eventTX()
  {
    auto txev = read(REG_TXEV);

    if (txev & (1 << TXEV_FIFO0_SRT))
    {
      auto status = read(REG_TXS0);

      if (status & (1 << TXS_TX_DONE_SRT))
      {
        write(REG_TXC0, 1 << TXC_FLUSH_SRT);

        if (!(status & (1 << TXS_ACK_STAT_SRT)))
        { /* packet was not acked, reset state */
          state = STATE_WSETUP;
          trigLed();
        }
        else if (state == STATE_CTRL_READ_DONE)
        { /* enable receiver to get host response to read transfer */
          write(REG_RXC0, (1 << RXC_RX_EN_SRT));
        }
        else
        {
          fillTX0();
        }
      }
    }

  }

  void handleInterrupt()
  {
    auto maev = read(REG_MAEV);


    if (maev & (1 << MAEV_ALT_SRT))
      eventALT();
    if (maev & (1 << MAEV_NAK_SRT))
      eventNAK();
    if (maev & (1 << MAEV_RX_EV_SRT))
      eventRX();
    if (maev & (1 << MAEV_TX_EV_SRT))
      eventTX();
  }


  void init()
  {
    /* software reset */
    while(0 == (read(REG_RID) & 0x0F));

    write(REG_MCNTRL, (1 << MCNTRL_SRST_SRT));
    while(read(REG_MCNTRL) & (1 << MCNTRL_SRST_SRT));

    writeRegisters(reset());
    writeRegisters(initSequence());

    /* disable clock out, do this later we use this
     * to check if the crystal is working properly */
    // write(REG_CCONF, (1 << CCONF_CODIS_SRT));

  }
};


class UsbDisplayApplication
{
private:
  USBN960X<SPISlaveA, FlashBuffer> _usb;


public:
  void main() __attribute__((noreturn))
  {
    {
      auto it = registerInit.begin();

      while(it != registerInit.end())
      {
          uint8_t reg = *(it++);
          _SFR_IO8(reg) = *(it++);
      }

      USISPIMaster<0>::initialize(0);
    }

    lcd_init();

    _usb.init();

    while (true)
    {
      _usb.handleInterrupt();
    }
  }
};


int main(void)
{
  static UsbDisplayApplication app;
  app.main();
}




