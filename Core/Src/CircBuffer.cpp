/*
 * CircBuffer.cpp
 *
 *  Created on: Nov 23, 2022
 *      Author: bernardosantana
 */

#include "CircBuffer.h"

CircBuffer::CircBuffer()
{

}

CircBuffer::~CircBuffer()
{

}

void CircBuffer::Read(uint8_t* data, size_t len)
{
	size_t avaiSize = Available();
	if(avaiSize < len) len = avaiSize;

	for(uint16_t i = 0; i < len; i++)
	{
		*(data+i) = buffer[head%MAX_BUFFER_SIZE];
		head++;
	}
}


void CircBuffer::Write(uint8_t* data, size_t len)
{
	for(uint16_t i = 0; i < len; i++)
	{
		buffer[tail%MAX_BUFFER_SIZE] = *(data+i);
		tail++;
	}
}


void CircBuffer::Peak(uint8_t* data, size_t len)
{
	size_t avaiSize = Available();
	if(avaiSize < len) len = avaiSize;

	for(uint16_t i = 0; i < len; i++)
		*(data+i) = buffer[head%MAX_BUFFER_SIZE];
}


size_t CircBuffer::Available()
{
	return tail < head ? (tail+MAX_BUFFER_SIZE - head) : (tail - head);
}

