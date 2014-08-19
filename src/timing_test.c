// blink.c
//
// Example program for bcm2835 library
// Blinks a pin on an off every 0.5 secs
//
// After installing bcm2835, you can build this 
// with something like:
// gcc -o blink blink.c -l bcm2835
// sudo ./blink
//
// Or you can test it before installing with:
// gcc -o blink -I ../../src ../../src/bcm2835.c blink.c
// sudo ./blink
//
// Author: Mike McCauley
// Copyright (C) 2011 Mike McCauley
// $Id: RF22.h,v 1.21 2012/05/30 01:51:25 mikem Exp $

#include "bcm2835.h"
#include <stdio.h>
#include <sched.h>
#include <memory.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <time.h>
#include <memory.h>
#include <math.h>
#include <termio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
// Blinks on RPi Plug P1 pin 11 (which is GPIO pin 17)
#define PIN RPI_GPIO_P1_11


/*
void test_serial(void)
{
	FILE *serialFileDescriptor;

// open the serial like POSIX C
serialFileDescriptor = open(
    "/dev/tty.usbserial-A6008cD3",
    O_RDWR |
    O_NOCTTY |
    O_NONBLOCK );
B1000000

// block non-root users from using this port
ioctl(serialFileDescriptor, TIOCEXCL);

// clear the O_NONBLOCK flag, so that read() will
//   block and wait for data.
fcntl(serialFileDescriptor, F_SETFL, 0);

// grab the options for the serial port
tcgetattr(serialFileDescriptor, &options);

// setting raw-mode allows the use of tcsetattr() and ioctl()
cfmakeraw(&options);

// specify any arbitrary baud rate
ioctl(serialFileDescriptor, IOSSIOSPEED, &baudRate);
}
*/


inline uint32_t timestamp(void)
{
	struct timespec ts;
	uint32_t microseconds;
	clock_gettime(CLOCK_MONOTONIC_RAW,&ts);
	microseconds = (uint32_t)ts.tv_sec * 1000000UL;
	microseconds+= (uint32_t)ts.tv_nsec / 1000UL;
	return microseconds;
}

void burn_cycles(uint32_t cycles)
{
	volatile double d1=1.0,d2=100.0,d3=1000.0,d4=10000.0;

	while(cycles)
	{
		d1 = sin(d2);
		d2 = cos(d3);
		d3 = d4 / d2;
		d4 = d1 * d2;
		cycles--;
	}
}

void burn_microseconds(uint32_t m)
{
	uint32_t t_now;
	t_now = timestamp();
	while(timestamp() - t_now < m);
}



void gpio_test(void)
{
	uint32_t t1,t2;
    // Configure some GPIO pins fo some testing
    // Set RPI pin P1-11 to be an output
    bcm2835_gpio_fsel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);
    // Blink
    while (1)
    {
    	t1=timestamp();
    	usleep(50);
    	t2=timestamp();
    	if((t2-t1)<100) burn_microseconds(100-(t2-t1));
		bcm2835_gpio_write(RPI_GPIO_P1_11, HIGH);
    	t1=timestamp();
    	usleep(50);
    	t2=timestamp();
    	if((t2-t1)<100) burn_microseconds(100-(t2-t1));
		bcm2835_gpio_write(RPI_GPIO_P1_11, LOW);
    }
}


void write_mode(uint8_t mode)
{
	//write mode
	char buf[] =
	{ 0x01, 0x00 }; // Data to send
	buf[1] = mode;
	bcm2835_spi_transfern(buf, 2);
	//printf("Mode: %02X \n",  buf[1]);
}

uint8_t read_mode(void)
{
	//read mode
	char buf[] =
	{ 0x05, 0x00 }; // Data to send
	bcm2835_spi_transfern(buf, 2);
	//printf("Mode: %02X \n",  buf[1]);
	return buf[1];
}

uint32_t enab[3];
void disable_interrupts(void)
{
	enab[0] = bcm2835_peri_read(bcm2835_irq + 0x210 / 4);
	enab[1] = bcm2835_peri_read(bcm2835_irq + 0x214 / 4);
	enab[2] = bcm2835_peri_read(bcm2835_irq + 0x218 / 4);
	bcm2835_peri_write(bcm2835_irq + 0x21c / 4, 0xffffffff);
	bcm2835_peri_write(bcm2835_irq + 0x220 / 4, 0xffffffff);
	bcm2835_peri_write(bcm2835_irq + 0x224 / 4, 0xffffffff);
}

void restore_interrupts(void)
{
	bcm2835_peri_write(bcm2835_irq + 0x210 / 4, enab[0]);
	bcm2835_peri_write(bcm2835_irq + 0x214 / 4, enab[1]);
	bcm2835_peri_write(bcm2835_irq + 0x218 / 4, enab[2]);
}


void uart0_show_baud(void)
{
	volatile uint32_t ibrd, fbrd, cr;

	ibrd = bcm2835_peri_read(bcm2835_uart0 + 0x24/4);
	fbrd = bcm2835_peri_read(bcm2835_uart0 + 0x28/4);
	cr   = bcm2835_peri_read(bcm2835_uart0 + 0x30/4);

	printf("i=%d   f=%d    cr=0x%08x\n",ibrd, fbrd, cr);
}

void uart0_set_baud(uint32_t ibrd, uint32_t fbrd)
{
	volatile uint32_t cr;
	uart0_show_baud();

	cr   = bcm2835_peri_read(bcm2835_uart0 + 0x30/4);

	usleep(1000);
	//bcm2835_peri_write(bcm2835_uart0 + 0x30/4, cr & 0xFFFFFFFEUL);
	usleep(1000);
	bcm2835_peri_write(bcm2835_uart0 + 0x24/4, ibrd);
	bcm2835_peri_write(bcm2835_uart0 + 0x28/4, fbrd);
	usleep(1000);
	//bcm2835_peri_write(bcm2835_uart0 + 0x30/4, cr);
	usleep(1000);

	uart0_show_baud();
}


void uart0_test(void)
{
		int i;
		volatile uint32_t* uart0;

		if (!bcm2835_init()) return;

		uart0 = bcm2835_uart0;

		for (i = 0; i < 100; i++)
		{
			printf("0x%04x: 0x%08X\n", (unsigned long) (&(uart0[i])) & 0xfffL,
					bcm2835_peri_read(&(uart0[i])));
		}
		printf("\n");
}


int irq_test(void)
{
	int i;
	volatile uint32_t* irq;

	if (!bcm2835_init())
		return 1;

	irq = bcm2835_irq + 0x200 / 4;

	for (i = 0; i < 10; i++)
	{
		printf("0x%04x: 0x%08X\n", (unsigned long) (&(irq[i])) & 0xfffL,
				bcm2835_peri_read(&(irq[i])));
	}
	printf("\n");

	bcm2835_peri_write(bcm2835_irq + 0x21c / 4, 0xffffffff);
	bcm2835_peri_write(bcm2835_irq + 0x220 / 4, 0xffffffff);
	bcm2835_peri_write(bcm2835_irq + 0x224 / 4, 0xffffffff);

	enab[0] = bcm2835_peri_read(bcm2835_irq + 0x210 / 4);
	enab[1] = bcm2835_peri_read(bcm2835_irq + 0x214 / 4);
	enab[2] = bcm2835_peri_read(bcm2835_irq + 0x218 / 4);

	bcm2835_peri_write(bcm2835_irq + 0x210 / 4, 0x00040008);
	bcm2835_peri_write(bcm2835_irq + 0x214 / 4, 0x42100001);
	bcm2835_peri_write(bcm2835_irq + 0x218 / 4, 0x00000006);

	printf("0x%08x 0x%08x 0x%08x \n\n", enab[0], enab[1], enab[2]);

	for (i = 0; i < 10; i++)
	{
		printf("0x%04x: 0x%08X\n", (unsigned long) (&(irq[i])) & 0xfffL,
				bcm2835_peri_read(&(irq[i])));
	}

	bcm2835_close();

	return 0;
}



/*
 * worst case, 52ms get "stolen" if interrupts are not disabled. The smaller the sleep between iterations, the more likely it is that the max is hit.
 * w/ 10ms sleep between iterations, on average 1ms get stolen from every 62ms cycle burn loop
 * if the cycle burn loop is 620+ ms long, it is very likely that 10/40/50ms get "stolen"
 */
int cycleburn_test(void)
{
	int loops=0;
	uint32_t t1, t2, max=0, delta;

	while(1)
	{
		loops++;
		//disable_interrupts();
		t1 = (uint32_t) bcm2835_st_read();
		burn_cycles(100000);
		t2 = (uint32_t) bcm2835_st_read();
		//restore_interrupts();
		delta = (t2-t1)-62850;
		if(delta > max)
		{
			max=delta;
			printf("%05d: %d.  Max Delta=%d.  \n",loops, t2-t1, max);
		}
		usleep(10000);
	}
	return 0;
}

#define NUM_BYTES 250


int spi_test(void)
{
	uint32_t st1, st2, st_max = 0;
	int i, j, loops = 0;
	uint8_t tx_buf[NUM_BYTES];
	uint8_t rx_buf[NUM_BYTES]; // = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };

	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16); // The default
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

	// Send a some bytes to the slave and simultaneously read
	// some bytes back from the slave
	// Most SPI devices expect one or 2 bytes of command, after which they will send back
	// some data. In such a case you will have the command bytes first in the buffer,
	// followed by as many 0 bytes as you expect returned data bytes. After the transfer, you
	// Can the read the reply bytes from the buffer.
	// If you tie MISO to MOSI, you should read back what was sent.

	printf("Mode: %02X \n", read_mode());
	write_mode(0x00);
	printf("Mode: %02X \n", read_mode());
	write_mode(0x40);
	printf("Mode: %02X \n", read_mode());

	while (1)
	{

		usleep(500);
		write_mode(0x40);

		loops++;

		disable_interrupts();
		st1 = (uint32_t) bcm2835_st_read();
		for (i = 0; i < 1; i++)
		{
			//write_mode(0x40);
			tx_buf[0] = 0x02;
			tx_buf[1] = 0x00;
			tx_buf[2] = 0x00;
			for (j = 3; j < NUM_BYTES; j++)
				tx_buf[j] = j;
			bcm2835_spi_transfern(tx_buf, NUM_BYTES);
			//bcm2835_delayMicroseconds(10);
			//usleep(100);
		}
		st2 = (uint32_t) bcm2835_st_read();
		restore_interrupts();

		if ((st2 - st1) > st_max)
		{
			st_max = st2 - st1;
			printf("%06d: Max write time for %d bytes: %d us\n", loops,
					NUM_BYTES, st_max);
		}

		if (loops % 1000 == 0)
			st_max = 0;

		//write_mode(0x40);
		for (i = 0; i < 1; i++)
		{
			//write_mode(0x40);
			memset(rx_buf, 0, NUM_BYTES);
			rx_buf[0] = 0x03; //read command
			bcm2835_spi_transfern(rx_buf, NUM_BYTES);
			//bcm2835_delayMicroseconds(10);
			for (j = 3; j < NUM_BYTES; j++)
			{
				if (rx_buf[j] != j)
				{
					printf("data = %02x %02x %02x ... %02x \n", rx_buf[3],
							rx_buf[4], rx_buf[5], rx_buf[NUM_BYTES - 1]);
					break;
				}
			}

		}
	}

	bcm2835_spi_end();
	bcm2835_close();

	return 0;
}

/*
1000us:
delta=2066, error min,max,avg=12,   228, 32  hist=>10k: 0   >5k: 0   >2.5k: 0   >1k: 0   >500: 0   >250: 6    >100:145    >50:5584    >20:18900   <20: 4365   loops: 29000

900Mhz, 100us, sleep-only:  18% CPU usage; 1-2 x per second, t > 100us; avg error=8.9us;  10-20 x per second > 50us
delta=111, error min,max,avg= 1,   175,  8  hist=>10k: 0   >5k: 0   >2.5k: 0   >1k: 0   >500: 0   >250: 1    >100:184    >50: 537    >20: 9999   <20:749279   loops:760000
 */

#define microseconds 100

int timing_test()
{
	uint32_t loops = 0, e[20], error_sum = 0, st1, st2, t_last=0, delta, error, error_min =
			microseconds * 10, error_max = 0, error_avg = microseconds;

	struct timespec ts =
	{ 0, (microseconds - 15) * 1000 };

	memset(e, 0, sizeof(e));

	st1=timestamp();   burn_cycles(250);    	st2=timestamp();	delta = st2-st1;	printf("burn_cycles(250): %u\n",delta);

	st1=timestamp();   burn_cycles(154);    	st2=timestamp();	delta = st2-st1;	printf("burn_cycles(154): %u\n",delta);
	st1=timestamp();   burn_cycles(798);    	st2=timestamp();	delta = st2-st1;	printf("burn_cycles(798): %u\n",delta);
	st1=timestamp();   burn_cycles(798);    	st2=timestamp();	delta = st2-st1;	printf("burn_cycles(798): %u\n",delta);
	st1=timestamp();   burn_cycles(1575);    	st2=timestamp();	delta = st2-st1;	printf("burn_cycles(1575): %u\n",delta);

	st1=timestamp();   burn_microseconds(microseconds); st2=timestamp();	delta = st2-st1;
	printf("burn_microseconds(%d): %u\n",microseconds,delta);


	st2=timestamp();
	while (1)
	{
		loops++;
		//st1=st2;
		//ts1=ts2;
		//st1 = (uint32_t) bcm2835_st_read();
		//clock_gettime(CLOCK_MONOTONIC_RAW,&ts1);
		//bcm2835_delayMicroseconds(1000);
		st1=timestamp();
		nanosleep(&ts, NULL );
		//burn_cycles(154);
		//disable_interrupts();
		//burn_cycles(1575);
		//restore_interrupts();
		//burn_microseconds(microseconds-1);
		st2=timestamp();
		delta = st2-st1;
		if(delta < microseconds) delta=microseconds;
		error = delta - microseconds; //delta - microseconds*2;
		if (error > error_max)
			error_max = error;
		if (error < error_min)
			error_min = error;
		error_sum += error;
		if (error > 10000)		e[0]++;
		else if (error > 5000)	e[1]++;
		else if (error > 2500)	e[2]++;
		else if (error > 1000)	e[3]++;
		else if (error > 500)	e[4]++;
		else if (error > 250)	e[5]++;
		else if (error > 100)	e[6]++;
		else if (error > 50)	e[7]++;
		else if (error > 20)	e[8]++;
		else if (error <= 20)	e[9]++;
		e[10]++;
		//if (st1 - t_last > 5000000 )
		if(loops % 100 == 0)
		{
			usleep(10000); //just checking..
		}
		if(loops % 1000 == 0)
		{
			t_last = st1;
			error_avg = error_sum / 1000;
			printf(
					"delta=%d, error min,max,avg=%2u,%6u,%3u  hist=>10k:%2u   >5k:%2u   >2.5k:%2u   >1k:%2u   >500:%2u   >250:%2u    >100:%3u    >50:%4u    >20:%5u   <20:%5u   loops:%6u\n",
					st2 - st1, error_min, error_max, error_avg,
					e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9], e[10]);
			error_min = 100000;
			error_max = 0;
			error_avg = 0;
			error_sum = 0;
			fflush(stdout);
			//st2=timestamp();
		}
	}
}

int main(int argc, char **argv)
{
	struct sched_param sp;

#if 1
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	sched_setscheduler(0, SCHED_FIFO, &sp);
	mlockall(MCL_CURRENT | MCL_FUTURE); //doesn't make much of a difference
	printf("result = %d\n",	setpriority(PRIO_PROCESS, 0, sched_get_priority_max(SCHED_FIFO)));
#endif

	if (!bcm2835_init()) return 1;

	printf("t=%ld\n",timestamp());
	usleep(1000);
	printf("t=%ld\n",timestamp());

	//gpio_test();
	//uart0_test(); bcm2835_close(); return 0;
	//uart0_set_baud(10,54);
	//uart0_show_baud();
	//irq_test(); bcm2835_close(); return 0;
	//spi_test(); bcm2835_close(); return 0;
	//cycleburn_test(); bcm2835_close(); return 0;
	timing_test(); bcm2835_close(); return 0;

	bcm2835_close();
	return 0;
}

