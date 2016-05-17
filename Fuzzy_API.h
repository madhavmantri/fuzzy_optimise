/*
 * Fuzzy_API.h
 *
 * Created: 6/1/2013 6:38:23 PM
 *  Author: rishav jain
 */ 

#include<stdint.h>
#ifndef FUZZY_API_H_
#define FUZZY_API_H_

#define Kp_Small_Value 1
#define Kp_Medium_Value 2
#define Kp_Large_Value 3
#define Kd_Small_Value 4
#define Kd_Medium_Value 6
#define Kd_Large_Value 11

void Fuzzy_API(volatile int32_t[],volatile int32_t[],uint8_t*);
void Error_Fuzzification(volatile int32_t[], volatile int32_t[]);
void Create_Fuzzy_Matrix(void);
void Determine_Weights(void);
void Give_Motor_Velocity(volatile int32_t[],volatile int32_t[],uint8_t*);


#endif /* FUZZY_API_H_ */
