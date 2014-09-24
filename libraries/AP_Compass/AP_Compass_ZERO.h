/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_ZERO_H
#define AP_Compass_ZERO_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"

//A test driver that puts the _field attribute always 0
class AP_Compass_ZERO : public Compass
{
private:


public:
    AP_Compass_ZERO() : Compass() {
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

};
#endif
