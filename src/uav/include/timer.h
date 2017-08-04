/**
 * Copyright (c) 2017, Jack Mo (mobangjack@foxmail.com).
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

#pragma once

#include <stdint.h>
#include <unistd.h>
//#include <sys/time.h>
#include <time.h>

class Timer
{
public:
	Timer(const uint32_t millis = 0);
	int64_t remain();
	bool timeout();
	void reset(const uint32_t millis);
	
public:
	static int get_time (timespec* time);

private:
	timespec expiry;
};
