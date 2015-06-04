/*
 * controller_bsp.c
 *
 *  Created on: 04.06.2015
 *      Author: andi
 */
#include "controller_bsp.hpp"

struct BSP::Uart      BSP::s_Uart;
struct BSP::Time      BSP::s_Time;
struct BSP::Timers1Ms BSP::s_Timers1Ms;
struct BSP::Outputs   BSP::s_Outputs;

void BSP::initialize()
{
  const uint8_t DDRPortB =  PinLCD_Reset.MASK |
                            PinLCD_A0.MASK |
                            PinBoostConverter.MASK |
                            PinSPI_MOSI.MASK |
                            PinSPI_SCK.MASK;

  const uint8_t InitPortB = PinLCD_A0.MASK;

  const uint8_t DDRPortC  = PinLed.MASK |
                            PinSD_CS.MASK;

  const uint8_t InitPortC = PinLed.MASK |
                            PinSD_CS.MASK;

  const uint8_t DDRPortD  = PinCAN_CS.MASK  |
                            PinOut_ChA.MASK |
                            PinOut_ChB.MASK |
                            PinSD_PwrE.MASK |
                            PinLCD_CS.MASK |
                            PinSens_ChA.MASK |
                            PinSens_ChB.MASK;

  const uint8_t InitPortD  = PinLCD_CS.MASK |
                             PinCAN_CS.MASK  |
                             PinSD_PwrE.MASK |
                             PinSens_ChA.MASK |
                             PinSens_ChB.MASK ;

  PortB = InitPortB;
  PortC = InitPortC;
  PortD = InitPortD;

  PortB.setDirection(DDRPortB);
  PortC.setDirection(DDRPortC);
  PortD.setDirection(DDRPortD);



  initLCD();

  /* Setup Timer/Counter 0 to generate 125 Hz Overflow */
  TCNT0 = 256 - 250;
  TCCR0 = _BV(CS02);

  /* Prepare Timer 1 to generate a 20khz square on OC1A */
  OCR1A  = 200;

  /* Enable Timer 0 IRQ */
  TIMSK = _BV(TOIE0);

  Sys_AVR8::enableInterrupts();
}

void BSP::cycle()
{
  /* calculate number of milliseconds passewd since last update */
  uint8_t dClock1Ms = s_Time.Clock1Ms.delta(s_Time.IsrTickCnt);

  if(dClock1Ms)
  {
    s_Time.Clock1Ms.advance(dClock1Ms);

    SimpleTimer<uint16_t> *pTimer1Ms    = reinterpret_cast<SimpleTimer<uint16_t> *>(&s_Timers1Ms);
    SimpleTimer<uint16_t> *pTimer1MsEnd = pTimer1Ms + sizeof(s_Timers1Ms) / sizeof(*pTimer1Ms);

    while(pTimer1Ms < pTimer1MsEnd)
    {
      pTimer1Ms->update(dClock1Ms);
      pTimer1Ms++;
    }
  }

  handleOutputs();
}

void BSP::handleOutputs()
{
  uint8_t State = s_Outputs.State;

  uint8_t OutputsState = State & (OUTPUT_CHA | OUTPUT_CHB);
  uint8_t BoostState   = State & 0x3;

  if(OutputsState)
  {
    if(BoostState == BOOST_OFF)
    {
      BoostState = BOOST_STARTUP;
      s_Timers1Ms.Boost.start(1000);

      TCCR1A = _BV(COM1A0);
      TCCR1B = _BV(WGM12) | _BV(CS10);
    }
    else if(BoostState == BOOST_STARTUP)
    {
      if (s_Timers1Ms.Boost.hasTimedOut())
        BoostState = BOOST_ACTIVE;
    }
  }
  else
  {
    TCCR1A = 0;
    TCCR1B = 0;

    BoostState = BOOST_OFF;
  }

  s_Outputs.State = OutputsState | BoostState;

  if(BoostState != BOOST_ACTIVE)
    OutputsState = 0;

  PortD.updateOutputs(OutputsState,PinOut_ChA.MASK | PinOut_ChB.MASK);

}
