/*
 * serialPort.h
 *
 *  Created on: Nov 27, 2022
 *      Author: bernardosantana
 */

#ifndef INC_SERIALPORT_H_
#define INC_SERIALPORT_H_

#include "CircBuffer.h"
#include "stm32f4xx.h"

class serialPort{
	public:
		serialPort();
		serialPort(UART_HandleTypeDef* uart);
		virtual ~serialPort();

		void AssignUart(UART_HandleTypeDef* uart);

		void Read(uint8_t* data, size_t len);
		void Send(uint8_t* data, size_t len);
		void Peak (uint8_t* data, size_t len);
		void Receive();
		size_t Available();

	private:
		UART_HandleTypeDef* uart;
		uint8_t readByte;
		CircBuffer rxBuffer;

		void enableRxIT();
	};

#endif /* INC_SERIALPORT_H_ */
