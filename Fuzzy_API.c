/*
 * Fuzzy_API.c
 *
 * Created: 6/1/2013 5:54:35 PM
 *  Author: rishav jain
 */ 


#include "Fuzzy_API.h"

	
int32_t Motor_Velocity[2] = {0},  
		Negative_Error[2] = {0}, Zero_Error[2] = {0}, Positive_Error[2] = {0},
		Negative_D_Error[2] = {0}, Zero_D_Error[2] = {0}, Positive_D_Error[2] = {0},
		Kp_Small[2] = {0}, Kp_Medium[2] = {0}, Kp_Large[2] = {0},
		Kd_Small[2] = {0}, Kd_Medium[2] = {0}, Kd_Large[2] = {0},		
		Fuzzy_Matrix[2][3][3] = {{{0}}};
			
int32_t Kp_Multiplier[2], Kd_Multiplier[2];
int32_t		Kp_Divider[2], Kd_Divider[2];
int32_t		Fault[2] = {0};
	

void Fuzzy_API(volatile int32_t error[],volatile int32_t d_error[], uint8_t* oc)
{
	Error_Fuzzification(error, d_error);
	Create_Fuzzy_Matrix();
	Determine_Weights();
	Give_Motor_Velocity(error, d_error,oc);
}
void Error_Fuzzification(volatile int32_t Err[], volatile int32_t D_Err[])
{
	int32_t i = 0;
	for(i=0;i<2;i++)
	{
		Positive_Error[i] = Negative_Error[i] = Zero_Error[i] = 0;
		if(Err[i]>10)
		{
			Positive_Error[i] = 10;
		}
		else if(Err[i]<-10)
		{
			Negative_Error[i] = 10;
		}
		else
		{
			if(Err[i] > 0)
			{
				Positive_Error[i] = (Err[i]);
				Zero_Error[i] = 10 - ((Err[i]));
			}
			else
			{
				Negative_Error[i] =  -((Err[i]));
				Zero_Error[i] = 10 + (Err[i]);
			}
		}
		Positive_D_Error[i] = Negative_D_Error[i] = Zero_D_Error[i] = 0;
		if(D_Err[i]>10)
		{
			Positive_D_Error[i] = 10;
		}
		else if(D_Err[i]<-10)
		{
			Negative_D_Error[i] = 10;
		}
		else
		{
			if(D_Err[i] > 0)
			{
				Positive_D_Error[i] = (D_Err[i]);
				Zero_D_Error[i] = 10 - ((D_Err[i]));
			}
			else
			{
				Negative_D_Error[i] =  -(D_Err[i]);
				Zero_D_Error[i] = 10 + (D_Err[i]);
			}
		}
		
		
	}
}
void Create_Fuzzy_Matrix(void)
{
	int32_t i = 0;
	


	for(i = 0;i < 2;i++)
	{
		Fuzzy_Matrix[i][0][0] = Negative_D_Error[i] < Negative_Error[i] ? Negative_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][1] = Zero_D_Error[i] < Negative_Error[i] ? Zero_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][2] = Positive_D_Error[i] < Negative_Error[i] ? Positive_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][1][0] = Negative_D_Error[i] < Zero_Error[i] ? Negative_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][1] = Zero_D_Error[i] < Zero_Error[i] ? Zero_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][2] = Positive_D_Error[i] < Zero_Error[i] ? Positive_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][2][0] = Negative_D_Error[i] < Positive_Error[i] ? Negative_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][1] = Zero_D_Error[i] < Positive_Error[i] ? Zero_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][2] = Positive_D_Error[i] < Positive_Error[i] ? Positive_D_Error[i] : Positive_Error[i];
	}

}
void Determine_Weights(void)
{
	int32_t i ;
	Kp_Multiplier[1] = Kp_Multiplier[0] = Kd_Multiplier[1] = Kd_Multiplier[0] = 0;
	

	for(i = 0; i < 2; i++)
	{
		
		Kp_Small[i] = Fuzzy_Matrix[i][1][1];

		Kp_Medium[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][0][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][1][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][1][2];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][0][0] ? Kp_Medium[i] : Fuzzy_Matrix[i][0][0];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][2];
		

		Kp_Large[i] =  Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];
		
		
		Kd_Small[i] = Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];
		Kd_Small[i] = Kd_Small[i] > Fuzzy_Matrix[i][1][1] ? Kd_Small[i] : Fuzzy_Matrix[i][1][1];
		
		
		Kd_Medium[i] = Fuzzy_Matrix[i][0][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][0][0] : Fuzzy_Matrix[i][0][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][2];
		

		Kd_Large[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][1][2] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][1][2];
		
		
		Kp_Multiplier[i] = (Kp_Small[i] * Kp_Small_Value) + (Kp_Medium[i] * Kp_Medium_Value) + (Kp_Large[i] * Kp_Large_Value);
		Kd_Multiplier[i] = (Kd_Small[i] * Kd_Small_Value) + (Kd_Medium[i] * Kd_Medium_Value) + (Kd_Large[i] * Kd_Large_Value);
		
		Kp_Divider[i] = (Kp_Small[i] + Kp_Medium[i] + Kp_Large[i]);
		Kd_Divider[i] = (Kd_Small[i] + Kd_Medium[i] + Kd_Large[i]);
	}

}
void Give_Motor_Velocity(volatile int32_t err[],volatile int32_t d_err[], uint8_t* ocm)
{
	int32_t i;
	
	for(i = 0; i < 2; i++)
	{
		
		if(Kp_Divider[i] != 0 && Kd_Divider[i] != 0)
		{
			Fault[i] = (int32_t) ((err[i] * Kp_Multiplier[i]) / Kp_Divider[i]);
			Fault[i] += (int32_t) ((d_err[i] * Kd_Multiplier[i]) / Kd_Divider[i]);
		}
		Motor_Velocity[i] += Fault[i];
	}
	
	if(Motor_Velocity[0] < 0)
	{
		if(Motor_Velocity[0] < -255)
		{
			ocm[0] = 255;
			Motor_Velocity[0] = -255;
		}
		else
		ocm[0] = -Motor_Velocity[0];
		
		ocm[1] = 0;
	}
	else
	{
		if(Motor_Velocity[0] > 255)
		{
			ocm[1] = 255;
			Motor_Velocity[0] = 255;
		}
		else
		ocm[1] = Motor_Velocity[0] ;
		
		ocm[0] = 0;
	}
	
	if(Motor_Velocity[1] < 0)
	{
		if(Motor_Velocity[1] <-255)
		{
			ocm[3] = 255;
			Motor_Velocity[1] = -255;
		}
		else
		ocm[3] = -Motor_Velocity[1] ;
		
		ocm[2] = 0;
	}
	else
	{
		if(Motor_Velocity[1] >255)
		{
			ocm[2] = 255;
			Motor_Velocity[1] = 255;
		}
		else
		ocm[2] = Motor_Velocity[1] ;
		
		ocm[3] = 0;
	}

}	

	
	//Uart_Write('a');	
	//Uart_Write(Error);
	//Uart_Write(OCR_API[0]);
	//Uart_Write(OCR_API[1]);
	
	


