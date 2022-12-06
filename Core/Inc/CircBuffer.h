/*
 * CircBuffer.h
 *
 *  Created on: Nov 23, 2022
 *      Author: bernardosantana
 */

#ifndef EXAMPLE_USER_CIRCBUFFER_H_
#define EXAMPLE_USER_CIRCBUFFER_H_

#include <stdint.h>
#include <stdio.h>

#define MAX_BUFFER_SIZE 1024

class CircBuffer {
public:
	CircBuffer();
	virtual ~CircBuffer();
	void Read(uint8_t* data, size_t len);
	void Write(uint8_t* data, size_t len);
	void Peak (uint8_t* data, size_t len);
	size_t Available();

private:
	uint8_t buffer[MAX_BUFFER_SIZE] = {0};
	size_t head, tail = 0;
};

#endif /* EXAMPLE_USER_CIRCBUFFER_H_ */
