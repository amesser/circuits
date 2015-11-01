/*
 * calibrator_bsp.hpp
 *
 *  Created on: 26.07.2015
 *      Author: andi
 */

#ifndef CALIBRATOR_BSP_HPP_
#define CALIBRATOR_BSP_HPP_

#include "ecpp/Target.hpp"
#include "ecpp/Time.hpp"

#include "ecpp/Peripherals/LCD_HD44780.hpp"
#include "uart_atmega.hpp"

#include "voltage-modulator.hpp"
#include "protocol.hpp"
#include "globals.hpp"

using namespace ecpp;
using namespace ecpp::Peripherals;

/* Forward declarations of ISRs to define C linkage */
extern "C"
{
  void TIMER1_COMPA_vect (void) __attribute__ ((signal,__INTR_ATTRS));
  void USART_RXC_vect (void) __attribute__ ((signal,__INTR_ATTRS));
  void TWI_vect (void) __attribute__ ((signal,__INTR_ATTRS));
};


template<class DEVICE, int COLUMNS, int ROWS>
class BufferedLCD : protected DEVICE
{
public:
  typedef char RowBufferType[COLUMNS];

private:
  uint8_t  m_Index;
  uint8_t m_RefreshIndex;

  union {
    char          m_Framebuffer[COLUMNS * ROWS];
    RowBufferType m_RowBuffers[ROWS];
  };

public:
  class LocationType
  {
  private:
    uint8_t m_Index;
  public:
    constexpr LocationType(uint8_t Column, uint8_t Row) : m_Index(Row * COLUMNS + Column) {}

    uint8_t   getBufferOffset() const {return m_Index;}
    uint8_t   getRow()    const {return m_Index / COLUMNS;}
    uint8_t   getColumn() const {return m_Index % COLUMNS;}
  };

  static LocationType Location(uint8_t Column, uint8_t Row) {return LocationType(Column,Row);}

  void init() {DEVICE::init();}
  void poll();

  template<typename IteratorType>
  void displayString(LocationType Loc, IteratorType Start, IteratorType End);

  RowBufferType & updateRow(LocationType Loc)
  {
    m_RefreshIndex = min(m_RefreshIndex, Loc.getBufferOffset());
    return m_RowBuffers[Loc.getRow()];
  }
};



template<class DEVICE, int COLUMNS, int ROWS>
void BufferedLCD<DEVICE, COLUMNS, ROWS>::poll()
{
  /* check if the frame buffer changed while we were doing our updates */
  if (this->m_Index == ElementCount(this->m_Framebuffer))
  {
    this->m_Index  = m_RefreshIndex;
    m_RefreshIndex = ElementCount(this->m_Framebuffer);
  }

  if (this->m_Index < ElementCount(this->m_Framebuffer))
  {
    if (this->isReady())
    { /* lcd is ready for the next command */
      auto CurrentLoc  = DEVICE::getCursorLocation();
      auto RequiredLoc = DEVICE::getLocation(m_Index % COLUMNS, m_Index / COLUMNS);

      if(RequiredLoc != CurrentLoc)
      {
        this->setCursor(RequiredLoc);
      }
      else
      {
        this->putChar(this->m_Framebuffer[m_Index]);
        m_Index += 1;
      }
    }
  }
}

template<class DEVICE, int COLUMNS, int ROWS>
template<typename IteratorType>
void BufferedLCD<DEVICE, COLUMNS, ROWS>::displayString(LocationType Loc, IteratorType Start, IteratorType End)
{
  IteratorType it = Start;
  uint8_t  Offset = Loc.getBufferOffset();

  m_RefreshIndex = min(m_RefreshIndex,Offset);

  while(Offset < ElementCount(m_Framebuffer) && it < End)
  {
    m_Framebuffer[Offset++] = *(it++);
  }

}


class CalibratorBsp
{
protected:
  static IOPin<AVR_IO_PD1> PinE;
  static IOPin<AVR_IO_PD2> PinRw;
  static IOPin<AVR_IO_PD3> PinRs;

  static IOPort<AVR_IO_PB> PortB;
  static IOPort<AVR_IO_PC> PortNibble;
  static IOPort<AVR_IO_PD> PortLcdControl;

  static IOPin<AVR_IO_PD6> Pin1k8;
  static IOPin<AVR_IO_PD7> Pin8k2;
  static IOPin<AVR_IO_PB0> Pin3k3;

  static IOPin<AVR_IO_PB2> PinKeyA;
  static IOPin<AVR_IO_PB1> PinKeyB;

  class VoltageModulatorBsp
  {
  protected:
    void                        setVoltageState(uint_fast8_t State) {CalibratorBsp::setSensorVoltage(State);}
    Globals::MillisecondTimer & getTimer() {return Globals::getGlobals().getVoltageModulatorTimer();}
  };


  class LcdBsp
  {
  protected:
    static const FlashVariable<HD44780_CMD, 5> InitSequence PROGMEM;

    bool isReady()
    {
      return (TIFR & _BV(OCF1B)) != 0;
    }

    void setBusy(uint16_t Delay1us)
    {
      OCR1B  = TCNT1 + Delay1us;
      TIFR  |= _BV(OCF1B);
    }

    void waitReady()
    {
      while(!isReady());
    }

    void waitReady(uint16_t Delay1us)
    {
      setBusy(Delay1us);
      waitReady();
    }


    static void setRW()       {PortNibble.updateDirection(0x0F,0x00); PinRw = 1; }
    static void clearRW()     {PinRw = 0; PortNibble.updateDirection(0x0F,0x0F);}
    static void setRS()       {PinRs = 1;}
    static void clearRS()     {PinRs = 0;}
    static void clearEnable() {PinE  = 0;}
    static void setEnable()   {PinE  = 1;}
    static void setNibble(uint8_t data) {PortNibble.updateOutputs(data, 0x0F);}
  };

public:
  typedef BufferedLCD<LCD_HD44780<HD44780_MODE_4BIT, LcdBsp>, 16, 2>             LCDType;
  typedef AdaptingUart<8>                                                        UartHandlerType;
  typedef VoltageModulator<sizeof(calibration_data), 80,20, VoltageModulatorBsp> VoltageModulatorType;
  typedef TWIMaster<4> TWIMasterType;

  typedef LCDType::RowBufferType RowBufferType;
protected:
  static CalibratorBsp s_Instance;

  volatile uint16_t m_IsrTicks;
  uint16_t          m_HandledSecondTicks;
  uint8_t           m_HandledMillisecondTicks;
  uint_fast8_t          m_KeyState;

  UartHandlerType       m_UartHandler;

  LCDType               m_Lcd;
  VoltageModulatorType  m_VoltageModulator;
  TWIMasterType         m_TWIMaster;
public:
  enum
  {
    SUPPLY_OFF = 4,
    SUPPLY_5V  = 2,
    SUPPLY_4V  = 1,
    SUPPLY_3V  = 0,
  };

  static CalibratorBsp & getBsp() {return s_Instance;}

  LCDType         & getLCD()                           {return m_Lcd;}
  UartHandlerType & getUartHandler()           {return m_UartHandler;}
  VoltageModulatorType & getVoltageModulator() {return m_VoltageModulator;}
  TWIMasterType &        getTWIHandler()       {return m_TWIMaster;}

  LCDType::RowBufferType & getDisplayRow(uint8_t Row) {return m_Lcd.updateRow(m_Lcd.Location(0,Row));}

  static void init();


  static uint_fast16_t getDivisor() {return (UBRRH << 8) | UBRRL;}

  static void setSensorVoltage(uint8_t state);

  void cycle();
  uint_fast8_t getKeyState() const {return m_KeyState & 0x0F;}

  friend void TIMER1_COMPA_vect (void);
  friend void USART_RXC_vect (void);
  friend void TWI_vect (void);
};



#endif /* FIRMWARE_SRC_CALIBRATOR_BSP_HPP_ */
