/*
 * slave_reg.h
 *
 *  Created on: Nov 14, 2024
 *      Author: dell
 */

#ifndef SLAVE_REG_H_
#define SLAVE_REG_H_


#include "main.h"

enum instruction_e{
	TURN_OFF_LED,
	TURN_ON_LED,
	RESET_DEVICE,
	WRITE_MEM,
	READ_MEM
};

typedef struct
{
	uint8_t device_id;
	enum instruction_e	instructions;
	uint8_t memory[256];
}slave_t;


#endif /* SLAVE_REG_H_ */
