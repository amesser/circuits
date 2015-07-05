/*
 * sensor-attiny13a.S
 *
 *  Created on: 28.06.2015
 *      Author: andi
 */

#include <avr/io.h>


#define PinRefCapacitor PB0
#define PinHumCapacitor PB1

#define PinLedCathode   PB2
#define PinLedAnode     PB3

#define PinTemp         PB4
#define PinReset        PB5

#define MSK_REFCAP      _BV(PinRefCapacitor)
#define MSK_HUMCAP      _BV(PinHumCapacitor)
#define MSK_LEDC        _BV(PinLedCathode)
#define MSK_LEDA        _BV(PinLedAnode)
#define MSK_TEMP        _BV(PinTemp)

#define MSK_ADCSRA_PRESCALER  _BV(ADPS1) | _BV(ADPS0)

#define MSK_ADCSRA_VCC  _BV(ADEN)  | _BV(ADSC) | _BV(ADATE)
#define MSK_ADMUX_VCC   _BV(ADLAR) | _BV(MUX1)

#define VCC_3V 0
#define VCC_4V 1
#define VCC_5V 2

.section ".noinit"

/* buffer to hold the measurement data */
buffer:
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000

/* buffer to hold the calibration data */
#define calibration_size (8*3)

calibration:
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000
  .word 0x0000

.section ".vectors"

reset:
  eor r1,r1

checkSupplyVoltage:
  /* forward bias temp diode */
  ldi r24, MSK_REFCAP | MSK_HUMCAP | MSK_LEDC | MSK_LEDA
  ldi r22, MSK_LEDC | MSK_LEDA
  rcall updatePins

  /* enable adc in free runnning mode */
  ldi r18, MSK_ADMUX_VCC
  ldi r19, MSK_ADCSRA_VCC | MSK_ADCSRA_PRESCALER

  out _SFR_IO_ADDR(ADMUX), r18
  out _SFR_IO_ADDR(ADCSRA), r19

  rcall waitADC

  /* start timer 0 with clk / 1024 */
  ldi r24, _BV(CS02) | _BV(CS00)
  rcall startTimer

  /* wait for about 5 seconds for operating voltage:
   * F_CPU / 1024. / 256. * 5 */
  ldi r19, 23 /* r19 is not modified by getSupplyVoltageState */

waitSupplyVoltage:
  rcall getSupplyVoltageState
  mov r2, r24 /* make a backup of state in r2, used later on */

  cp r1, r24
  brne doMeasurement

  in  r24, _SFR_IO_ADDR(TIFR0)
  bst r24, TOV0

  brtc waitSupplyVoltage

  /* timer 0 overflow occured */
  out _SFR_IO_ADDR(TIFR0) , r24
  subi r19, 1
  brcc waitSupplyVoltage

/* the supply voltage stayed below 3.66 V for
 * about 5 seconds, this is initiator for
 * calibration data download */
doCalibrationDownload:
  /* r4 will hold the number of bits to be received */
  ldi  r28, lo8(calibration_size * 8)
  mov r4, r28

  /* let y pointer point to the beginning of the calibration area */
  ldi  r28, lo8(calibration)
  ldi  r29, hi8(calibration)

  eor r24, 24
  eor r3,  r3 /* r3 will hold the actual byte */

doCalibrationDownload_RestartTimer:
  /* backup last state */
  mov r25, r24

  /* download timeout is about 30 seconds
   * F_CPU / 64. / 256. * 30 */

  ldi r26, lo8(2197)
  ldi r27, hi8(2197)

  /* restart timer 0 with clk / 64 */
  ldi r24, _BV(CS02) | _BV(CS00)
  rcall startTimer

doCalibrationDownload_CheckTimer:
  /* check for timer 0 overflow */
  in  r24, _SFR_IO_ADDR(TIFR0)
  bst r24, TOV0

  brtc doCalibrationDownload_EvalVoltageState
   /* timer 0 overflow */
  sbiw r26, 1
  brcs doMeasurement

/* r25 contains the last state of the voltage */
doCalibrationDownload_EvalVoltageState:
  rcall getSupplyVoltageState

  /* check if the voltage changed since last call */
  cp r25, r24
  brne doCalibrationDownload_RestartTimer

  /* check if the voltage changed since last bit */
  cp r2, r24
  breq doCalibrationDownload_CheckTimer

  /* check kif a voltage change was stable for at
   * least 80 ms */
  ldi r30, lo8(2197 - 5)
  ldi r31, hi8(2197 - 5)

  sub r30, r26
  sbc r31, r27

  brcc doCalibrationDownload_CheckTimer

  /* a new bit was received
   * r25/r24 contains the new state
   * r2  contains the old state
   * r25 must not be changed!!! */

  ldi r30, lo8(wrappedInteger3 + 3)
  ldi r31, hi8(wrappedInteger3 + 3)

  add r30, r24
  adc r31,  r0

  sub r30,  r2
  sbc r31,  r0

  /* memorize state */
  mov r2, r24

  /* get the bit */
  lpm r24, Z
  dec r24

  /* store the bit */
  lsl r3
  or  r3, r24

  /* decrement bit counter */
  dec  r4

  /* check if byte complete  */
  mov  r24, r4
  andi r24, 0x07
  brne doCalibrationDownload_CheckTimer

  /* store byte */
  st y+, r3
  mov r3, r1

  /* check if all bytes received */
  tst r4
  brne doCalibrationDownload_CheckTimer

writeCalibrationData:
  ldi r18, calibration_size

writeCalibrationDataLoop:
  ld r19, -y
  dec r18

  out _SFR_IO_ADDR(EEARL), r18
  out _SFR_IO_ADDR(EEDR), r19

  sbi _SFR_IO_ADDR(EECR), EEMWE
  sbi _SFR_IO_ADDR(EECR), EEWE

writeCalibrationDataWait:
  sbic _SFR_IO_ADDR(EECR), EEWE
  rjmp writeCalibrationDataWait

  brne writeCalibrationDataLoop


doMeasurement:
  /* initialize pins */
  ldi r24, MSK_REFCAP | MSK_HUMCAP | MSK_LEDC | MSK_LEDA | MSK_TEMP
  eor r22, r22
  rcall updatePins

/* disable Timer 0 */
  out _SFR_IO_ADDR(TCCR0B), r1
/* disable ADC */
  out _SFR_IO_ADDR(ADMUX), r1
  out _SFR_IO_ADDR(ADCSRA), r1


/* load the calibration parameters from eeprom into ram,
 * address calibration */
readCalibrationData:
  ldi  r18, lo8(calibration + calibration_size)
  ldi  r19, hi8(calibration + calibration_size)
  movw r30, r18

  ldi r18, calibration_size

readCalibrationDataLoop:
  dec r18
  out _SFR_IO_ADDR(EEARL), r18
  sbi _SFR_IO_ADDR(EECR), EERE
  in  r19, _SFR_IO_ADDR(EEDR)
  st -Z, r19
  brne readCalibrationDataLoop


finish:
  rjmp finish


.section ".rodata"
wrappedInteger3:
  .byte 0x00, 0x01, 0x02, 0x00, 0x01, 0x02

.section ".text"

updatePins:
  andi r22, MSK_REFCAP | MSK_HUMCAP | MSK_LEDC | MSK_LEDA
  out _SFR_IO_ADDR(DDRB),  r1
  out _SFR_IO_ADDR(PORTB), r22
  out _SFR_IO_ADDR(DDRB),  r24
  ret

/* wait for ADC until it has done its measurement */
waitADC:
  sbis _SFR_IO_ADDR(ADCSRA), ADIF
  rjmp waitADC
  ret

/* measure the voltage of the temperature diode and use it
 * to detect the supply voltage:
 * 0 :  < 3.666
 * 1 :  < 4.333
 * 2 : >= 4.333
 */
#define VCC_LIMIT_A 38 /* (256. * 0.55 / (3. + 0.666)) */
#define VCC_LIMIT_B 32 /* (256. * 0.55 / (4. + 0.333)) */

getSupplyVoltageState:
  ldi  r24, VCC_5V
  in   r18, _SFR_IO_ADDR(ADCH)
  cpi  r18, VCC_LIMIT_B
  sbci r24, 0
  cpi  r18, VCC_LIMIT_A
  sbci r24, 0
  ret

/* start timer 0 with given prescaler */
startTimer:
  out _SFR_IO_ADDR(TCCR0B), r24
  ldi r24, _BV(TOV0) | _BV(PSR10)
  out _SFR_IO_ADDR(GTCCR) , r24
  out _SFR_IO_ADDR(TCNT0) , r1
  out _SFR_IO_ADDR(TIFR0) , r24
  ret



