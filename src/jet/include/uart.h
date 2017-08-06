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

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <semaphore.h>

#define UART_OFLAG_RD (O_RDONLY | O_NOCTTY)
#define UART_OFLAG_WR (O_WRONLY | O_NOCTTY)
#define UART_OFLAG_RW (O_RDWR | O_NOCTTY)

int uart_open(int* pfd, const char* device, uint32_t baudrate, int oflag);

int uart_config(int fd, int speed, int databits, char parity, int stopbits);

int uart_read(int fd, uint8_t* data, uint32_t len);

int uart_write(int fd, const uint8_t* data, uint32_t len);

int uart_close(int fd);

#ifdef __cplusplus
}
#endif

#endif

