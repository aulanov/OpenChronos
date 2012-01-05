#ifndef ALTITUDE_LOGIC_H
#define ALTITUDE_LOGIC_H

#include <project.h>

extern void update_pressure_table(s16 href, u32 p_meas);
extern s16 conv_pa_to_altitude(u32 p_meas);

#endif // ALTITUDE_LOGIC_H
