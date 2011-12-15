/* Pulse Generator
   Copyright (C) 2003 Free Software Foundation, Inc.
   Written by Stephane Carrez (stcarrez@nerim.fr)	

This file is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

In addition to the permissions in the GNU General Public License, the
Free Software Foundation gives you unlimited permission to link the
compiled version of this file with other programs, and to distribute
those programs without any restriction coming from the use of this
file.  (The General Public License restrictions do apply in other
respects; for example, they cover modification of the file, and
distribution when not linked into another program.)

This file is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; see the file COPYING.  If not, write to
the Free Software Foundation, 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.  */

/*! @page pulse Pulse Generator

    This program is a simple pulse generator.  It uses an output
    compare timer to generate sequences of pulses of various periods.

    The OC4 compare is set to generate an interrupt and toggle the
    PA4/OC4 output pin.  In the interrupt handler, a new compare value
    is set according to a static table which represents the pattern to
    generate.  The compare value is always computed from the previous
    compare value to ensure there is no drift due to the program itself.
    Consider the following chart:

<pre>
     ______          ___________         ___
PA4 |      |        |           |       |
    +------+--------+-----------+-------+---> time
      ^ ^   ^ ^
      |_|   |_| Interrupts  
    <->    <>
     Interrupt latency
</pre>

    The PA4 pin is changed when the compare register OC4 matches the
    free running counter.  This is changed in hardware and thus is
    synchronous with the free running counter.  The interrupt is handled
    immediately but there will be a delay to finish the current instruction
    (assuming that interrupts are enabled).  The next compare value is set
    by adding a constant to the previous value, thus providing an exact
    and reproducible pulse.

    The interrupt latency is the time between when it is raised by the
    hardware and when it is first handled by the processor (ie, when it
    starts pushing the interrupt frame).  The interrupt latency can be
    checked in the gdb simulator by using the command:

    (gdb) sim info

    and the output looks like:

<pre>
N  Interrupt     Cycle Taken         Latency   Delta between interrupts
 0 OUT4                18715               3
 1 OUT4                17713               1      1002 (501.0 us)
 2 OUT4                17514               2       199 (99.5 us)
 3 OUT4                 7513               1     10001 (5.0 ms)
 4 OUT4                 5513               1      2000 (1.0 ms)
 5 OUT4                 3513               1      2000 (1.0 ms)
 6 OUT4                 2514               2       999 (499.5 us)
 7 OUT4                 1515               3       999 (499.5 us)
 8 OUT4                  516               4       999 (499.5 us)
</pre>

    If you connect an oscilloscope on PA4 you should see the pulses
    with the timing indicated in `cycle_table'.

  @htmlonly
  Source file: <a href="pulse_8c-source.html">pulse.c</a>
  @endhtmlonly

*/
#include <sys/param.h>
#include <sys/ports.h>
#include <sys/interrupts.h>
#include <sys/sio.h>
#include <sys/locks.h>

void output_compare_interrupt (void) __attribute__((interrupt));

#define US_TO_CYCLE(N) ((N) * 2)

/* The cycle table defines the sequence of pulses to generate.
   Each value indicates the number of cycles to wait before inverting
   the output pin.  The US_TO_CYCLE macro makes the translation so
   that values can be expressed in microseconds (assuming QZ at 8Mhz).

   Note: A value below 100 cycles will produce a 32ms pulse because
   we are not that fast to update the next output compare value.  */
static const unsigned short cycle_table[] = {
   US_TO_CYCLE (500),
   US_TO_CYCLE (500),
   US_TO_CYCLE (500),
   US_TO_CYCLE (1000),
   US_TO_CYCLE (1000),
   US_TO_CYCLE (5000),
   US_TO_CYCLE (100),
   US_TO_CYCLE (500),
   US_TO_CYCLE (5000),
   US_TO_CYCLE (1000),
   US_TO_CYCLE (100),
   US_TO_CYCLE (100)
};

#define TABLE_SIZE(T) ((sizeof T / sizeof T[0]))

#ifdef USE_INTERRUPT_TABLE

/* Interrupt table used to connect our timer_interrupt handler.

   Note: the `XXX_handler: foo' notation is a GNU extension which is
   used here to ensure correct association of the handler in the struct.
   This is why the order of handlers declared below does not follow
   the HC11 order.  */
struct interrupt_vectors __attribute__((section(".vectors"))) vectors = 
{
  res0_handler:           fatal_interrupt, /* res0 */
  res1_handler:           fatal_interrupt,
  res2_handler:           fatal_interrupt,
  res3_handler:           fatal_interrupt,
  res4_handler:           fatal_interrupt,
  res5_handler:           fatal_interrupt,
  res6_handler:           fatal_interrupt,
  res7_handler:           fatal_interrupt,
  res8_handler:           fatal_interrupt,
  res9_handler:           fatal_interrupt,
  res10_handler:          fatal_interrupt, /* res 10 */
  sci_handler:            fatal_interrupt, /* sci */
  spi_handler:            fatal_interrupt, /* spi */
  acc_overflow_handler:   fatal_interrupt, /* acc overflow */
  acc_input_handler:      fatal_interrupt,
  timer_overflow_handler: fatal_interrupt,
  output5_handler:        fatal_interrupt, /* out compare 5 */
  output3_handler:        fatal_interrupt, /* out compare 3 */
  output2_handler:        fatal_interrupt, /* out compare 2 */
  output1_handler:        fatal_interrupt, /* out compare 1 */
  capture3_handler:       fatal_interrupt, /* in capt 3 */
  capture2_handler:       fatal_interrupt, /* in capt 2 */
  capture1_handler:       fatal_interrupt, /* in capt 1 */
  rtii_handler:           fatal_interrupt,
  irq_handler:            fatal_interrupt, /* IRQ */
  xirq_handler:           fatal_interrupt, /* XIRQ */
  swi_handler:            fatal_interrupt, /* swi */
  illegal_handler:        fatal_interrupt, /* illegal */
  cop_fail_handler:       fatal_interrupt,
  cop_clock_handler:      fatal_interrupt,

  /* What we really need.  */
  output4_handler:        output_compare_interrupt, /* out compare 4 */
  reset_handler:          _start
};

#endif

static const unsigned short* cycle_next;
static volatile unsigned char wakeup;
static unsigned short change_time;

/* Output compare interrupt to setup the new timer.  */
void
output_compare_interrupt (void)
{
  unsigned short dt;

  _io_ports[M6811_TFLG1] |= M6811_OC4F;

  /* Setup the new output compare as soon as we can.  */
  dt = *cycle_next;
  dt += change_time;
  set_output_compare_4 (dt);
  change_time = dt;

  /* Prepare for the next interrupt.  */
  cycle_next++;
  if (cycle_next >= &cycle_table[TABLE_SIZE (cycle_table)])
    cycle_next = cycle_table;

  wakeup = 1;
}

int
main ()
{
  unsigned short j;
  unsigned char c = 0;
  unsigned char i = 0;
  
  lock ();
  serial_init ();

  /* Install the interrupt handler (unless we use the interrupt table).  */
  set_interrupt_handler (TIMER_OUTPUT4_VECTOR, output_compare_interrupt);

  cycle_next = cycle_table;

  /* Set OC4 compare to toggle the output pin.  */
  _io_ports[M6811_TCTL1] = M6811_OL4;
  _io_ports[M6811_TMSK1] = M6811_OC4I;

  /* Start the pulse generation.  */
  change_time = get_timer_counter () + 300;
  set_output_compare_4 (change_time);
  unlock ();

  for (j = 0; j < 1000; j++)
    {
      /* Wait for the output compare interrupt to be raised.  */
      wakeup = 0;
      while (wakeup == 0)
        continue;

      /* Produce some activity on serial line so that we know
         it is running and interrupts are raised/caught correctly.  */
      c++;
      if (c == 1)
        serial_send ('\b');
      else if (c == 128)
        serial_send ("-\\|/"[(++i) & 3]);
    }
  return 0;
}
