/*****************************************************************************
**                                                                           
**  Name:          brcm_patchram_plus.c
**
**  Description:   This program downloads a patchram files in the HCD format
**                 to Broadcom Bluetooth based silicon and combo chips and
**				   and other utility functions.
**
**                 It can be invoked from the command line in the form
**						<-d> to print a debug log
**						<--patchram patchram_file>
**						<--baudrate baud_rate>
**						<--bd_addr bd_address>
**						<--enable_lpm>
**						uart_device_name
**
**                 For example:
**
**                 brcm_patchram -d --patchram  \
**						BCM2045B2_002.002.011.0348.0349.hcd /dev/ttyHS0
**
**                 It will return 0 for success and a number greater than 0
**                 for any errors.
**
**                 For Android, this program invoked using a 
**                 "system(2)" call from the beginning of the bt_enable
**                 function inside the file 
**                 mydroid/system/bluetooth/bluedroid/bluetooth.c.
**
**  
**  
**     Copyright 2009 Broadcom Corporation
**  
**  This program is the proprietary software of Broadcom Corporation and/or its
**  licensors, and may only be used, duplicated, modified or distributed 
**  pursuant to the terms and conditions of a separate, written license 
**  agreement executed between you and Broadcom (an "Authorized License").  
**  Except as set forth in an Authorized License, Broadcom grants no license 
**  (express or implied), right to use, or waiver of any kind with respect to 
**  the Software, and Broadcom expressly reserves all rights in and to the 
**  Software and all intellectual property rights therein.  IF YOU HAVE NO 
**  AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY WAY,
**  AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF THE 
**  SOFTWARE.  
**   
**  Except as expressly set forth in the Authorized License,
**   
**  1.     This program, including its structure, sequence and organization, 
**  constitutes the valuable trade secrets of Broadcom, and you shall use all 
**  reasonable efforts to protect the confidentiality thereof, and to use this 
**  information only in connection with your use of Broadcom integrated circuit
**  products.
**   
**  2.     TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS
**  IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR 
**  WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH RESPECT 
**  TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL IMPLIED 
**  WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR A 
**  PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS, QUIET 
**  ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU ASSUME 
**  THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
**  
**  3.     TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM 
**  OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, 
**  INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY 
**  RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM 
**  HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN 
**  EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, 
**  WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY 
**  FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
**  
**  
******************************************************************************/
#include <stdio.h>
#include <getopt.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>

#ifdef ANDROID
#include <termios.h>
#else
#include <sys/termios.h>
#endif

#include <string.h>
#include <signal.h>

int uart_fd = -1;
int hcdfile_fd = -1;
int termios_baudrate = 0;
int bdaddr_flag = 0;
int enable_lpm = 0;
int debug = 0;

struct termios termios;
unsigned char buffer[1024];

unsigned char hci_reset[] = { 0x01, 0x03, 0x0c, 0x00 };

unsigned char hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };

unsigned char hci_update_baud_rate[] = { 0x01, 0x18, 0xfc, 0x02, 0x00, 0x00,
	0x00 };

unsigned char hci_write_bd_addr[] = { 0x01, 0x01, 0xfc, 0x06, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00 };

unsigned char hci_write_sleep_mode[] = { 0x01, 0x27, 0xfc, 0x0c, 
	0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00 };

int
parse_patchram(char *optarg)
{
	char *p;

	if (!(p = strrchr(optarg, '.'))) {
		fprintf(stderr, "file %s not an HCD file\n", optarg);
		exit(3);
	}

	p++;

	if (strcasecmp("hcd", p) != 0) {
		fprintf(stderr, "file %s not an HCD file\n", optarg);
		exit(4);
	}

	if ((hcdfile_fd = open(optarg, O_RDONLY)) == -1) {
		fprintf(stderr, "file %s could not be opened, error %d\n", optarg, errno);
		exit(5);
	}
}

// Based on what is supplied to the UART
#define CRYSTAL_FREQ 24000000

void 
BRCM_encode_baud_rate(uint baud_rate, unsigned char *encoded_baud)
{
    uint encoded_segment = 0;
    unsigned char encoded_baud_rate = 0;
 
    if(baud_rate == 0 || encoded_baud == NULL)
    {
      fprintf(stderr, "Baudrate not supported!");
      return; 
    }
 
    //Upper four bits
    encoded_segment = (((CRYSTAL_FREQ / baud_rate) % 16) / 2);
    encoded_baud_rate = ((unsigned char)encoded_segment) << 4;
 
    // Lower four bits of high nibble
    encoded_segment  = ((((CRYSTAL_FREQ / baud_rate) % 16) / 2) +
                        (((CRYSTAL_FREQ / baud_rate) % 16) % 2));
    encoded_baud_rate |= (unsigned char)encoded_segment;
 
    *encoded_baud = encoded_baud_rate;
    
    // Lower byte
    encoded_segment = ( 256 - ((CRYSTAL_FREQ / baud_rate) / 16) );
 
    *(encoded_baud+1) = (unsigned char)encoded_segment;
}

typedef struct {
	int baud_rate;
	int termios_value;
} tBaudRates;

tBaudRates baud_rates[] = {
	{ 115200, B115200 },
	{ 230400, B230400 },
	{ 460800, B460800 },
	{ 500000, B500000 },
	{ 576000, B576000 },
	{ 921600, B921600 },
	{ 1000000, B1000000 },
	{ 1152000, B1152000 },
	{ 1500000, B1500000 },
	{ 2000000, B2000000 },
	{ 2500000, B2500000 },
	{ 3000000, B3000000 },
#ifndef __CYGWIN__
	{ 3500000, B3500000 },
	{ 4000000, B4000000 }
#endif
};

int
validate_baudrate(int baud_rate, int *value)
{
	int i;

	for (i = 0; i < (sizeof(baud_rates) / sizeof(tBaudRates)); i++) {
		if (baud_rates[i].baud_rate == baud_rate) {
			*value = baud_rates[i].termios_value;
			return(1);
		}
	}

	return(0);
}

int
parse_baudrate(char *optarg)
{
	int baudrate = atoi(optarg);

	if (validate_baudrate(baudrate, &termios_baudrate)) {
		BRCM_encode_baud_rate(baudrate, &hci_update_baud_rate[4]);
	}

	return(0);
}

int
parse_bdaddr(char *optarg)
{
	int bd_addr[6];
	int i;

	sscanf(optarg, "%02x%02x%02x%02x%02x%02x", 
		&bd_addr[0], &bd_addr[1], &bd_addr[2],
		&bd_addr[3], &bd_addr[4], &bd_addr[5]);

	for (i = 0; i < 6; i++) {
		hci_write_bd_addr[4 + i] = bd_addr[i];
	}

	bdaddr_flag = 1;	

	return(0);
}

int
parse_enable_lpm(char *optarg)
{
	enable_lpm = 1;
	return(0);
}

int
parse_cmd_line(int argc, char **argv)
{
	int c;
	int digit_optind = 0;

	typedef int (*PFI)();

	PFI parse_param[] = { parse_patchram, parse_baudrate,
		parse_bdaddr, parse_enable_lpm };

    while (1)
    {
    	int this_option_optind = optind ? optind : 1;
        int option_index = 0;

       	static struct option long_options[] = {
         {"patchram", 1, 0, 0},
         {"baudrate", 1, 0, 0},
         {"bd_addr", 1, 0, 0},
         {"enable_lpm", 0, 0, 0},
         {0, 0, 0, 0}
       	};

       	c = getopt_long_only (argc, argv, "d", long_options, &option_index);

       	if (c == -1) {
      		break;
		}

       	switch (c) {
        case 0:
        	printf ("option %s", long_options[option_index].name);

        	if (optarg) {
           		printf (" with arg %s", optarg);
			}

           	printf ("\n");

			(*parse_param[option_index])(optarg);
		break;

		case 'd':
			debug = 1;
		break;

        case '?':
			//nobreak
        default:

			printf("Usage %s:\n", argv[0]);
			printf("\t<-d> to print a debug log\n");
			printf("\t<--patchram patchram_file>\n");
			printf("\t<--baudrate baud_rate>\n");
			printf("\t<--bd_addr bd_address>\n");
			printf("\t<--enable_lpm\n");
			printf("\tuart_device_name\n");
           	break;

        }
	}

   	if (optind < argc) {
       	if (optind < argc) {
       		printf ("%s ", argv[optind]);

			if ((uart_fd = open(argv[optind], O_RDWR | O_NOCTTY)) == -1) {
				fprintf(stderr, "port %s could not be opened, error %d\n", argv[2], errno);
			}
		}

       	printf ("\n");
    }

	return(0);
}

void
init_uart()
{
	tcflush(uart_fd, TCIOFLUSH);
	tcgetattr(uart_fd, &termios);

#ifndef __CYGWIN__
	cfmakeraw(&termios);
#else
	termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
	termios.c_oflag &= ~OPOST;
	termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	termios.c_cflag &= ~(CSIZE | PARENB);
	termios.c_cflag |= CS8;
#endif

	termios.c_cflag |= CRTSCTS;
	tcsetattr(uart_fd, TCSANOW, &termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcsetattr(uart_fd, TCSANOW, &termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcflush(uart_fd, TCIOFLUSH);
	cfsetospeed(&termios, B115200);
	cfsetispeed(&termios, B115200);
	tcsetattr(uart_fd, TCSANOW, &termios);
}

void
dump(unsigned char *out, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (i && !(i % 16)) {
			fprintf(stderr, "\n");
		}

		fprintf(stderr, "%02x ", out[i]);
	}

	fprintf(stderr, "\n");
}

read_event(int fd, unsigned char *buffer)
{
	int i = 0;
	int len = 3;
	int count;

	while ((count = read(fd, &buffer[i], len)) < len) {
		i += count;
		len -= count;
	}

	i += count;
	len = buffer[2];

	while ((count = read(fd, &buffer[i], len)) < len) {
		i += count;
		len -= count;
	}

	if (debug) {
		count += i;

		fprintf(stderr, "received %d\n", count);
		dump(buffer, count);
	}
}

void
hci_send_cmd(unsigned char *buf, int len)
{
	if (debug) {
		fprintf(stderr, "writing\n");
		dump(buf, len);
	}

	write(uart_fd, buf, len);
}

void
expired(int sig)
{
	hci_send_cmd(hci_reset, sizeof(hci_reset));
	alarm(4);
}

void
proc_reset()
{
	signal(SIGALRM, expired);


	hci_send_cmd(hci_reset, sizeof(hci_reset));

	alarm(4);

	read_event(uart_fd, buffer);

	alarm(0);
}

void
proc_patchram()
{
	int len;

	hci_send_cmd(hci_download_minidriver, sizeof(hci_download_minidriver));

	read_event(uart_fd, buffer);

	read(uart_fd, &buffer[0], 2);

	while (read(hcdfile_fd, &buffer[1], 3)) {
		buffer[0] = 0x01;

		len = buffer[3];

		read(hcdfile_fd, &buffer[4], len);

		hci_send_cmd(buffer, len + 4);

		read_event(uart_fd, buffer);
	}

	proc_reset();
}

void
proc_baudrate()
{
	hci_send_cmd(hci_update_baud_rate, sizeof(hci_update_baud_rate));

	read_event(uart_fd, buffer);

	cfsetospeed(&termios, termios_baudrate);
	cfsetispeed(&termios, termios_baudrate);
	tcsetattr(uart_fd, TCSANOW, &termios);
}

void
proc_bdaddr()
{
	hci_send_cmd(hci_write_bd_addr, sizeof(hci_write_bd_addr));

	read_event(uart_fd, buffer);
}

void
proc_enable_lpm()
{
	hci_send_cmd(hci_write_sleep_mode, sizeof(hci_write_sleep_mode));

	read_event(uart_fd, buffer);
}

int
main (int argc, char **argv)
{
	parse_cmd_line(argc, argv);

	if (uart_fd < 0) {
		exit(1);
	}

	init_uart();

	proc_reset();

	if (hcdfile_fd > 0) {
		proc_patchram();
	}

	if (termios_baudrate) {
		proc_baudrate();
	}

	if (bdaddr_flag) {
		proc_bdaddr();
	}

	if (enable_lpm) {
		proc_enable_lpm();
	}

	exit(0);
}
