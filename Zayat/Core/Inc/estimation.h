/*
 * estimation.h
 *
 *  Created on: Jun 5, 2020
 *      Author: Khaled Ali
 */

#ifndef INC_ESTIMATION_H_
#define INC_ESTIMATION_H_
void matrix_multi(int o, int n, int l,f32 mat1[][l],f32 mat2[][n], f32 mat3[][n]);
void init_EKF(void);
void predictstate(parameters *ptr);
void predict(parameters *ptr);
void updatefromGps(parameters *ptr);
void updatefromMag(parameters *ptr);
#endif /* INC_ESTIMATION_H_ */
