/*
 * main.cpp
 *
 *  Created on: 22.04.2015
 *      Author: andi
 */

#include "ecpp/Target.hpp"
#include "ecpp/Byteorder.hpp"
#include "protocol.hpp"

using namespace ecpp;

static IOPin<AVR_IO_PB0> RefCapacitor;
static IOPin<AVR_IO_PB1> HumCapacitor;

static IOPin<AVR_IO_PB2> LedCathode;
static IOPin<AVR_IO_PB3> LedAnode;

static IOPin<AVR_IO_PB4> Temp;
static IOPin<AVR_IO_PB5> Reset;

static IOPort<AVR_IO_PB> Port;

void delayBittime(); __attribute__((noinline))
void delayLong();    __attribute__((noinline))

void delayBittime()
{
  _delay_us( 1000000UL / 1200UL);
}

void delayLong()
{
  _delay_ms(1000);
}

static void
initPort()
{
  const uint8_t PortMask = RefCapacitor.MASK | HumCapacitor.MASK |
                           LedCathode.MASK | LedAnode.MASK |
                           Temp.MASK;

  *(Port.PORT) = 0;
  *(Port.DDR)  = PortMask;
}

struct measurement_data data __attribute__((section(".noinit")));

#define USE_AC 1

int main(void)
{

  uint16_t count;
  initPort();

#ifdef USE_AC
  ADMUX  = 0;
  ADCSRA = 0;
  ADCSRB = _BV(ACME);

  ACSR   = 0;
#endif

  /* measure humidity */
  for (count = 0; count < 0xFFFF; count++)
  {
    /* measure voltage and discharge
     * hum capacitor */
    HumCapacitor.enableOutput();


    asm ( "nop");

#ifdef USE_AC
    asm ( "nop");

    if(ACSR & _BV(ACO))
      break;
#else
    /* check if ref capacitor is charged */
    if(RefCapacitor.getInput())
      break;
#endif

    HumCapacitor.disableOutput();

    /* transfer charge */

    RefCapacitor.setOutput();
    RefCapacitor.enableOutput();

    RefCapacitor.disableOutput();
    RefCapacitor.clearOutput();
  }

  data.humidity_counts = hton16(count);

  initPort();

  /* measure light */
  /* charge led */
  LedCathode.setOutput();
  delayBittime();

  /* discharge capacitor via led and count time */
  LedCathode.disableOutput();
  LedCathode.clearOutput();

  for (count = 0; count < 0xFFFF; count++)
  {
    _delay_us(1000);

    if (!LedCathode.getInput())
      break;
  }

  data.led_counts = hton16(count);

  /* measure temperature */

  /* activate current source */
  LedCathode.clearOutput();
  LedCathode.enableOutput();

  LedAnode.setOutput();
  LedAnode.enableOutput();

  Temp.disableOutput();

  /* let things settle */
  delayLong();

  /* setup adc */
  ADMUX  = _BV(REFS0) | _BV(MUX1);
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS1) | _BV(ADPS0);
  ADCSRB = 0;

  ADCSRA |= _BV(ADSC);

  while(ADCSRA & _BV(ADSC));

  data.temp = hton16(ADC);

  ADCSRA = 0;
  ADMUX  = 0;

  Temp.enableOutput();

  data.sync = 0xC0;
  data.type = 0;

  /* uart */
  while(1)
  {
    const uint8_t *p = reinterpret_cast<const uint8_t*>(&(data));
    uint8_t  i = 0;

    delayLong();

    for(i = 0; i < sizeof(data); ++i)
    {
      // Start Bit
      LedAnode.clearOutput();
      delayBittime();

      uint8_t mask = 0x01;
      uint8_t par  =    0;
      while(mask)
      {
        if(mask & p[i])
        {
          LedAnode.setOutput();
          par += 1;
        }
        else
        {
          LedAnode.clearOutput();
        }

        delayBittime();

        mask = mask << 1;
      }

      LedAnode.clearOutput();
      delayBittime(); /* 9th bit always zero */

      if (par & 0x01)
        LedAnode.setOutput();
      else
        LedAnode.clearOutput();

      delayBittime(); /* parity */

      LedAnode.setOutput();

      delayBittime(); /* stop bit */
      delayBittime(); /* stop bit */
      delayBittime(); /* delay */
    }
  }


#if 0
  DDRC = 0xF;

  PORTD &= ~ _BV(PD7);
  DDRD  |=   _BV(PD7);
  DDRD  |=   _BV(PD6);

  _delay_us(1000);

  while(1)
  {
    _delay_us(1000);
    PORTD  ^=  _BV(PD6);
  }

  while(1)
  {

    /* measure voltage & discharge cx */
    PORTB &= ~ _BV(PB0);
    DDRB  |=   _BV(PB0);

    asm ( "nop");

    if(PIND & _BV(PD7))
    {
      /* cref charged */
      lastcounts = counts;
      counts     = 0;

      /* discharge cref */
      DDRD  |=  _BV(PD7);
      _delay_us(1000);
      DDRD  &= ~_BV(PD7);

      PORTD  ^=  _BV(PD6);
    }
    else if(counts < 0xFFFF)
    {
      counts += 1;
    }

    /* transfer charge */
    DDRB  &=  ~_BV(PB0);

    PORTD |= _BV(PD7);
    DDRD  |= _BV(PD7);

    DDRD  &= ~_BV(PD7);
    PORTD &= ~_BV(PD7);


    if (lastcounts & 0xF000)
      PORTC &= ~ _BV(PC3);
    else
      PORTC |=   _BV(PC3);

    if (lastcounts & 0xFF00)
      PORTC &= ~ _BV(PC2);
    else
      PORTC |=   _BV(PC2);

    if (lastcounts & 0xFFF0)
      PORTC &= ~ _BV(PC1);
    else
      PORTC |=   _BV(PC1);

    if (lastcounts & 0xFFFF)
      PORTC &= ~ _BV(PC0);
    else
      PORTC |=   _BV(PC0);
  }
#endif
}
