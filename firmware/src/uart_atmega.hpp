/*
 * uart_atmega.hpp
 *
 *  Created on: 10.08.2015
 *      Author: andi
 */

#ifndef UART_ATMEGA_HPP_
#define UART_ATMEGA_HPP_

#include <ecpp/Datatypes.hpp>
#include <ecpp/Target.hpp>
#include "globals.hpp"

template<int LENGTH>
class AdaptingUart
{
public:
  typedef typename ::ecpp::IntTypeEstimator<LENGTH>::Type IndexType;
  typedef uint8_t BufferType[LENGTH];
  static constexpr IndexType BufferSize = LENGTH;

private:
  enum
  {
    STATE_OFF   = 0,
    STATE_RECV  = 1,
    STATE_DONE  = 2,
  };

  volatile uint8_t m_State;
  uint8_t    m_RecvStatus;
  IndexType  m_RecvLength;
  BufferType m_Buffer;
public:
  BufferType & getBuffer() {return m_Buffer;}

  template<typename B>
  B & getBufferAs()
  {
    union {
      BufferType *pBuffer;
      B          *pType;
    } u;

    u.pBuffer = &m_Buffer;
    return *(u.pType);
  }

  IndexType getReceivedLength() {return m_RecvLength;}

  void recvIrq()
  {
    uint8_t status;
    uint8_t resh;
    uint8_t resl;

    status = UCSRA & (_BV(PE) | _BV(FE)); /* parity & frame error bits */
    resh   = UCSRB & _BV(RXB8); /* 9th bit */
    resl   = UDR;

    if(m_State == STATE_RECV)
    {
      auto offset = m_RecvLength;

      /* combine error bits and 9th bit */
      status = (status | resh);
      /* avr-gcc problem? bitshift operation generates 16 bit shift,
       * divide by two only 8 bit shift */
      status = (status / 2) /* status >> 1 */;

      if(offset == 0)
      {
        m_RecvStatus = status;
      }

      if((offset < TypeProperties<IndexType>::MaxUnsigned) &&
         (offset < BufferSize))
      {
        status = status << 4;

        m_RecvStatus = status | m_RecvStatus;
        m_Buffer[offset] = resl;

        m_RecvLength = offset + 1;
      }

      Globals::getGlobals().getUartTimer().start(100);
    }
  }

  void handleCyclic();

  static void init()
  {
    UBRRH = (0x7F & (416 >> 8));
    UBRRL = (0xFF &  416);
  }

  void activate();
  bool finished() const {return m_State == STATE_DONE;}
};


template<int LENGTH>
void AdaptingUart<LENGTH>::handleCyclic()
{
  auto & Timer = Globals::getGlobals().getUartTimer();

  uint8_t  len  = m_RecvLength;

  if((len > 0) && (STATE_RECV == m_State) && Timer.hasTimedOut())
  {
    /* the first byte transmitted by the sensor is a sync byte
     * with value 0x0C0. (9 bits). Together with the parity bit
     * and the two stop bits this can be used to figure out if
     * we need to adapt the baudrate
     */

    /* prevent further receiving */
    UCSRB &= ~_BV(RXEN);

    uint8_t sync = m_Buffer[0];
    int8_t delta = 0;

    if (len >= 1)
    {
      const uint8_t mask_parity = _BV(PE) >> 1;
      const uint8_t mask_fe     = _BV(FE) >> 1;

      const uint8_t status = m_RecvStatus;

      /* the received sync byte will be evaluated bit by bit
       * according the transmission direction. The farer
       * we get away from the start bit with the receive error
       * the closer we get to the correct divisor
       *
       * Parity and 9th bit is always zero, so the pattern to match is
       *
       * 0b0 0000 0011 00 11
       * Bit counting starts with 0 (including the start bit)
       */
      if(sync & 0x20)
      { /* Bit 7 received as Bit 6 -> too slow */
        delta = -1;
      }
      else if (0 == (sync & 0x40))
      { /* Bit 6 received as Bit 7 -> too fast */
        delta = +1;
      }
      else if(0 == (sync & 0x80))
      { /* bit 9 received as bit 8 -> too slow */
        delta = -1;
      }
      else if (status & 0x01)
      { /* bit 8 received as bit 9 -> too fast */
        delta = +1;
      }
      else if (status & mask_parity)
      { /* bit 11 (sb 0) received as bit 10 (Parity) -> too slow */
        delta = -1;
      }
      else if (status & mask_fe)
      { /* bit 10 (Parity) received as bit 11 (sb0) -> too fast */
        delta = +1;
      }

      if (delta || (status & (mask_parity << 4)))
      { /* we must adapt baud rate or a parity error occured in
         * any received char therefore repeat */
        len = 0;
      }
    }

    /* we should receive at least two characters */
    if (len < 2)
    {
      /* restart receiver with possibly new baud rate divisor */

      if(delta)
      { /* update baud rate divisor */
        uint16_t divisor = (UBRRH << 8) | UBRRL;

        if (divisor > 356 and delta < 0)
        {
          divisor -= 1;
        }
        else if (divisor < 499 and delta > 0)
        {
          divisor += 1;
        }

        UBRRH  = (divisor >> 8) & 0x7F;
        UBRRL  = divisor & 0xFF;
      }

      m_RecvLength = 0;
      /* reenable uart */
      UCSRB |= _BV(RXEN);
    }
    else
    { /*receiving finished successfully */
      m_State = STATE_DONE;
    }
  }
}

template<int LENGTH>
void AdaptingUart<LENGTH>::activate()
{
  UCSRB = 0; /* disable uart */

  m_State      = STATE_RECV;
  m_RecvLength = 0;

  memset(m_Buffer, 0x00, sizeof(m_Buffer));

  /* | _BV(UPM0) */

  UCSRA = 0;
  UCSRC = _BV(URSEL) | _BV(UPM1)  | _BV(UCSZ1) | _BV(UCSZ0);
  UCSRB = _BV(RXCIE) | _BV(RXEN)  | _BV(UCSZ2);
}


#endif /* UART_ATMEGA_HPP_ */
