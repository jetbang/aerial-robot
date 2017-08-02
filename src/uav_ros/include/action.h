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

#include "uav.h"

namespace uav
{
class Action
{
public: // public types

    enum State
    {
        PENDING,
        EXECUTING,
        FINISHED,
    };

    enum Type
    {
        STAND_BY,
        LANDING,
        GRABBING,
        TAKING_OFF,
        HOVER,
        LOCAL_NAV,
        DROPPING,
    };

public: // public variables
    State state;
    Type type;

public:
    uav::UAV& uav;

public:
    Action(uav::UAV& uav);
    
public: // public methods
    virtual bool init();
    virtual bool exec();
    virtual bool done();
}
}