/*
 * Read ADNS3080 OFlow sensor via SPI (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 * Copyright (c) 2013 Oswald Berthold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// network stuff 
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
// mavlink
#include <mavlink.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define ADNS3080_PRODUCT_ID            0x17

// #define DST_HOST "192.168.3.22" // eta
// #define DST_PORT 14550 // mavhub
#define DST_HOST "127.0.0.1" // eta
#define DST_PORT 32000 // mavhub


#define SYSID 21
// #define DEBUG_GETMOT

static void pabort(const char *s)
{
  perror(s);
  abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

static int transfer_delta(int fd, int reg)
{
  int ret;
  /*
    uint8_t tx[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
    0xF0, 0x0D,
    };
  */
  // uint8_t tx[] = {0x00, 0x00};
  uint8_t tx[] = {reg, 0x00};
  uint8_t rx[ARRAY_SIZE(tx)] = {0, };
  int8_t dval;
  struct spi_ioc_transfer tr = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = ARRAY_SIZE(tx),
    .delay_usecs = 50, // delay,
    .speed_hz = speed,
    .bits_per_word = bits,
  };

  // request data
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  // dummy read
  tr.tx_buf = (unsigned long)NULL;
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  tr.tx_buf = (unsigned long)tx;
  if (ret < 1)
    pabort("can't send spi message");

#ifdef DEBUG_GETMOT
  for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
    if (!(ret % 6))
      puts("");
    printf("reg %d: %.2X\n", reg, rx[ret]);
  }
  puts("");
#endif
  dval = rx[1];
  return dval;
}

////////////////////////////////////////////////////////////
// get squal (surface quality)
static int transfer_squal(int fd)
{
  int ret;
  uint8_t tx[] = {0x05, 0x00};
  uint8_t rx[ARRAY_SIZE(tx)] = {0, };
  struct spi_ioc_transfer tr = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = ARRAY_SIZE(tx),
    .delay_usecs = 50, // delay,
    .speed_hz = speed,
    .bits_per_word = bits,
  };

  // request data
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  // dummy read
  tr.tx_buf = (unsigned long)NULL;
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  tr.tx_buf = (unsigned long)tx;
  if (ret < 1)
    pabort("can't send spi message");

#ifdef DEBUG_GETMOT
  for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
    if (!(ret % 6))
      puts("");
    printf("squal: %.2X\n", rx[ret]);
  }
  puts("");
#endif
  return rx[1];
}

////////////////////////////////////////////////////////////
// get motion
static int transfer_motion(int fd)
{
  int ret;
  int retval = -1;
  uint8_t tx[] = {0x02, 0x00};
  uint8_t rx[ARRAY_SIZE(tx)] = {0, };
  struct spi_ioc_transfer tr = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = ARRAY_SIZE(tx),
    .delay_usecs = 80,
    .speed_hz = speed,
    .bits_per_word = bits,
  };

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  tr.tx_buf = (unsigned long)NULL;
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  tr.tx_buf = (unsigned long)tx;
  if (ret < 1) {
    // pabort("can't send spi message");
    printf("cant' send spi message\n");
    return retval;
  }
#ifdef DEBUG_GETMOT
  for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
    //	if (!(ret % 6))
    //		puts("");
    printf("%.2X ", rx[ret]);
  }
  puts("");
#endif
  if ((rx[1] & 0x80) != 0)
    return 1;
  else
    return -1;
}

////////////////////////////////////////////////////////////
// motion clear
static void transfer_motion_clear(int fd)
{
  int ret;
  int retval = -1;
  uint8_t tx[] = {0x12, 0x00};
  uint8_t rx[ARRAY_SIZE(tx)] = {0, };
  struct spi_ioc_transfer tr = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = ARRAY_SIZE(tx),
    .delay_usecs = 80,
    .speed_hz = speed,
    .bits_per_word = bits,
  };

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  tr.tx_buf = (unsigned long)NULL;
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  tr.tx_buf = (unsigned long)tx;
  if (ret < 1) {
    // pabort("can't send spi message");
    printf("cant' send spi message\n");
    return;
  }

  /* for (ret = 0; ret < ARRAY_SIZE(tx); ret++) { */
  /*   //	if (!(ret % 6)) */
  /*   //		puts(""); */
  /*   printf("%.2X ", rx[ret]); */
  /* } */
  /* puts(""); */
  /* if ((rx[1] & 0x80) != 0) */
  /*   return 1; */
  /* else */
  /*   return -1; */
}


////////////////////////////////////////////////////
// dummy init
static void init(int fd) {
  int ret;
  int retval = -1;
  int retry = 3;
  uint8_t tx[] = {0x00, 0x00};
  uint8_t rx[ARRAY_SIZE(tx)] = {0, };
  struct spi_ioc_transfer tr = {
    .tx_buf = (unsigned long)tx,
    .rx_buf = (unsigned long)rx,
    .len = ARRAY_SIZE(tx),
    .delay_usecs = 10000,
    .speed_hz = speed,
    .bits_per_word = bits,
  };

  // retry 3 times
  for (; retry > 0 ; retry--) {
    if(retry == 2)
      tx[0] = 0x00; // test response shift with different reg.
    else
      tx[0] = 0x00;

    // request data
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    // do dummy read to empty fifo
    tr.tx_buf = (unsigned long)NULL;
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    // reset tx back to proper
    tr.tx_buf = (unsigned long)tx;

    if (ret < 1) { // error?
      // pabort("can't send spi message");
      printf("cant' send spi message\n");
      return;
    }
    printf("Product ID / init: ");
    for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
      //      if (!(ret % 6))
      //              puts("");
      printf("%.2X ", rx[ret]);
    }
    puts("");
    if(rx[1] != 0x17) {
      printf("SPI comm failed\n");
      exit(1);
    }
  }
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char *argv[])
{
  int ret = 0;
  int fd;

  // network stuff
  int sock;
  struct sockaddr_in server_addr;
  struct hostent *host;

  // setup mavlink
  int cnt;
  int txbytes;
  uint8_t mav_tx_buf[MAVLINK_MAX_PACKET_LEN];
  static mavlink_message_t msg;
  static mavlink_message_t msg_hb;
  // mavlink_huch_potibox_t potibox;
  // mavlink_attitude_t attitude;
  mavlink_huch_visual_flow_t flow;
  mavlink_huch_visual_oflow_sen_t of_sen;
  uint16_t len;

#if (defined MAVLINK_VERSION && MAVLINK_VERSION >= 2)
  printf("prepping mavlink heartbeat\n");
  mavlink_msg_heartbeat_pack(SYSID, // system_id
                             35, // component_id,
                             &msg_hb,
                             MAV_TYPE_GENERIC, // mav_type,
                             MAV_AUTOPILOT_GENERIC, // autopilot,
                             MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  //base mode
                             0, //custom mode
                             MAV_STATE_ACTIVE);   //system status
#endif
  len = mavlink_msg_to_send_buffer(mav_tx_buf, &msg_hb);

  // setup socket, init network
  host = (struct hostent *) gethostbyname((char *)DST_HOST);

  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
      perror("socket");
      exit(1);
    }
	
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(DST_PORT);
  server_addr.sin_addr = *((struct in_addr *)host->h_addr);
  bzero(&(server_addr.sin_zero),8);

  // get command-line options
  parse_opts(argc, argv);

  fd = open(device, O_RDWR);
  if (fd < 0)
    pabort("can't open device");

  /*
   * spi mode
   */
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1)
    pabort("can't set spi mode");

  ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
  if (ret == -1)
    pabort("can't get spi mode");

  /*
   * bits per word
   */
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1)
    pabort("can't set bits per word");

  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1)
    pabort("can't get bits per word");

  /*
   * max speed hz
   */
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1)
    pabort("can't set max speed hz");

  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1)
    pabort("can't get max speed hz");

  printf("spi mode: %d\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

  init(fd);
  /* close(fd); */
  /* exit(0); */

  int i = 0;
  int gotmot = 0;
  int squal, dx, dy;
  // for(; i < 1000; i++) {
  cnt = 0;
  while(1) {
    // printf("-------------------\n");
    // usleep(50);
    gotmot = transfer_motion(fd);
    squal = transfer_squal(fd);
    dx = dy = 0;

    if(gotmot > 0) {
      // printf("got motion: %d\n", gotmot);
      dx = transfer_delta(fd, 0x03);
      dy = transfer_delta(fd, 0x04);
    }
    printf("squal,dx,dy: %d, %d, %d\n", squal, dx, dy);

    // transmit via mavlink
    flow.usec = 0;
    flow.u = (float)dx;
    flow.v = (float)dy;
    flow.u_i = (float)squal;

	of_sen.id = 0;
	of_sen.u = (float)dx;
	of_sen.v = (float)dy;
	of_sen.squal = (float)squal;

    // prepare for transmission
    // mavlink_msg_huch_visual_flow_encode(SYSID, 35, &msg, &flow);
    // len = mavlink_msg_to_send_buffer(mav_tx_buf, &msg);
    mavlink_msg_huch_visual_oflow_sen_encode(SYSID, 35, &msg, &of_sen);
    len = mavlink_msg_to_send_buffer(mav_tx_buf, &msg);
    // transmit data
    txbytes = sendto(sock, mav_tx_buf, len, 0,
                     (struct sockaddr *)&server_addr, sizeof(struct sockaddr));

    //if (buf[0]=='z') STOP=TRUE;
    if((cnt % 100) == 0) {
      len = mavlink_msg_to_send_buffer(mav_tx_buf, &msg_hb);
      txbytes = sendto(sock, mav_tx_buf, len, 0,
                       (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
      printf("sent heartbeat: %d|%d\n", len, txbytes);
    }

    transfer_motion_clear(fd);
    cnt++;
    usleep(10000);
  }

  close(fd);

  return ret;
}
