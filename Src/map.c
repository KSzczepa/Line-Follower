/*
 * map.c
 *
 *  Created on: 22.04.2020
 *      Author: klaudia
 */


#include "main.h"
#include "math.h"




void HAL_map(int Value, int Sensor)
{
	//Value = (float)Value;

	if (Value <= 750)
	{
		Sensor = 0;
	}

	else if (Value > 750 && Value <= 1350)
	{
		Sensor = 0.2;
	}

	else if (Value > 1350 && Value <= 1950)
	{
		Sensor = 0.4;
	}

	else if (Value > 1950 && Value <=  2550)
	{
		Sensor = 0.6;
	}

	else if (Value > 2550 && Value <= 3150)
	{
		Sensor = 0.8;
	}

	else
	{
		Sensor = 1;
	}

	//return Sensor;
}
