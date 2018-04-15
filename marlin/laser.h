#ifndef LASER_H
#define LASER_H

#include "Marlin.h"
#include "bitband.h"

#define LASER   PAout(3)
#define SYNC    PBout(12)
#define LDAC    PBout(14)

enum mode{POINT = 0, LINE, AREA};

void laser_init(void);
void servo_init(void);
void action(mode mode, unsigned short g, unsigned int num);
void point(int x, int y);
void point_xy(AxisEnum axis, unsigned short da);

FORCE_INLINE void transform(void)
{
    LDAC = 0;
    LDAC = 1;
};

#endif
