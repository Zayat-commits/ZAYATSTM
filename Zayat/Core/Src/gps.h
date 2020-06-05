/*
 * gps.h
 *
 *  Created on: Jun 4, 2020
 *      Author: Yousef
 */

#ifndef SRC_GPS_H_
#define SRC_GPS_H_
#include "main.h"
void gps_init(void);
uint8_t Read_gps(accel *position, accel *velocity);

#endif /* SRC_GPS_H_ */
