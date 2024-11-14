/*
 * slave_reg.c
 *
 *  Created on: Nov 14, 2024
 *      Author: dell
 */

#include "slave_reg.h"

void turn_on_led(char slave_choice)
{
	switch (slave_choice)
	{
	case 1:
		LL_GPIO_SetOutputPin(LED_SLAVE_1_GPIO_Port, LED_SLAVE_1_Pin);
		break;
	case 2:
		LL_GPIO_SetOutputPin(LED_SLAVE_2_GPIO_Port, LED_SLAVE_2_Pin);
		break;
	case 3:
		LL_GPIO_SetOutputPin(LED_SLAVE_3_GPIO_Port, LED_SLAVE_3_Pin);
		break;
	case 4:
		LL_GPIO_SetOutputPin(LED_SLAVE_4_GPIO_Port, LED_SLAVE_4_Pin);
	}
}

void turn_of_led(char slave_choice)
{
	switch (slave_choice)
	{
	case 1:
		LL_GPIO_ResetOutputPin(LED_SLAVE_1_GPIO_Port, LED_SLAVE_1_Pin);
		break;
	case 2:
		LL_GPIO_ResetOutputPin(LED_SLAVE_2_GPIO_Port, LED_SLAVE_2_Pin);
		break;
	case 3:
		LL_GPIO_ResetOutputPin(LED_SLAVE_3_GPIO_Port, LED_SLAVE_3_Pin);
		break;
	case 4:
		LL_GPIO_ResetOutputPin(LED_SLAVE_4_GPIO_Port, LED_SLAVE_4_Pin);
	}
}

void write_to_mem(uint8_t address,uint8_t data)
{

}
