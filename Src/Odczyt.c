/*
 * Odczyt.c
 *
 *  Created on: 22.04.2020
 *      Author: klaudia
 */


#include "math.h"

float Sum;
float Value;

float HAL1_Odczyt (float Sensor_L1, float Sensor_L2, float Sensor_S1, float Sensor_S2, float Sensor_R1, float Sensor_R2, float ActualValue)
{
	if (Sensor_R2 > 0)
	{
		Sum = Sensor_R2 * 4 + Sensor_R1 * 2;
		Value = Sum / (Sensor_R2 + Sensor_R1);
	}

	else if (Sensor_L1 > 0)
	{
		Sum = Sensor_L1 * (-4) + Sensor_L2 * (-2);
		Value = Sum / (Sensor_L1 + Sensor_L2);
	}

	else if (Sensor_R1 > 0)
	{
		Sum = Sensor_R1 * 2 + Sensor_S2;
		Value = Sum / (Sensor_R1 + Sensor_S2);
	}

	else if (Sensor_L2 > 0)
	{
		Sum = Sensor_L2 * (-2) - Sensor_S1;
		Value = Sum / (Sensor_L2 + Sensor_S1);
	}

	else
	{
		Sum = Sensor_S1 * (-1) + Sensor_S2;
		Value = Sum / (Sensor_S1 + Sensor_S2);
	}


	Value = Value * 100;
	Value = round(Value);
	Value = Value / 100;

	ActualValue = Value;

	return ActualValue;
}
