#ifndef SERIAL_HPP
#define SERIAL_HPP

#include "struct_typedef.h"
#include <cstdint>

void cmd_vel_callback(uint32_t seq, fp32 vx, fp32 vy, fp32 wz);
void set_mode_callback(uint32_t seq, int32_t mode);




#endif