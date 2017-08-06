/**
 * Copyright (c) 2016, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "uart.h"


static const int termbaud[] =
{
	B4800,
	B9600,
	B19200,
	B38400,
	B57600,
	B115200,
	B1000000,
	B1152000,
	B3000000,
};

static const int baudrate[] =
{
	4800,
	9600,
	19200,
	38400,
	57600,
	115200,
	1000000,
	1152000,
	3000000,
};

int uart_open(int* pfd, const char* device, uint32_t baudrate, int oflag)
{
  int fd = open(device, oflag);
  if (fd < 0)
  {
    printf("UART open ERROR!\n");
    return -1;
  }
  printf("open uart ok!\n");
  if (uart_config(fd, baudrate, 8, 'N', 1) < 0)
  {
    printf("UART config ERROR!\n");
    if (uart_close(fd) < 0)
    {
      printf("UART close ERROR!\n");
      return -3;
    }
    return -2;
  }
  *pfd = fd;
  return fd;
}

int uart_config(int fd, int speed, int databits, char parity, int stopbits)
{
	int i, j;
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	// Step 1: set char size
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	// Step 2: set stop bits
	switch (databits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	// Step 3: set parity
	switch (parity)
	{
	case 'O': // Odd
	case 'o': // Odd
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		//		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': // Even
	case 'e': // Even
		//		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  // No parity
	case 'n':  // No parity
		newtio.c_cflag &= ~PARENB;
		break;
	}
	j = sizeof(baudrate) / 4;
	for (i = 0; i < j; i++)
	{
		if (baudrate[i] == speed)
		{
			cfsetispeed(&newtio, termbaud[i]);
			cfsetospeed(&newtio, termbaud[i]);
			break;
		}
	}
	if ((i == j) && (baudrate[i - 1] != speed))
	{
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
	}
	if (stopbits == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (stopbits == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 1;
	newtio.c_cc[VMIN] = 1;
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Input
	newtio.c_oflag &= ~OPOST;   // Output

	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
	  perror("com set error");
	  return -1;
	}

	printf("set UART port paramster done!\n");
	return 0;
}

int uart_read(int fd, uint8_t* data, uint32_t len)
{
  int n = 0;
  while (n < len)
  {
    int r = read(fd, data + n, len - n);
    if (r < 0 )
    {
      break;
    }
    n += r;
  }

  return n;
}

int uart_write(int fd, const uint8_t* data, uint32_t len)
{
  int n = 0;
  while (n < len)
  {
    int w = write(fd, data + n, len - n);
    if (w < 0 )
    {
      break;
    }
    n += w;
  }

  return n;
}

int uart_close(int fd)
{
  return close(fd);
}

