/*
 * serialPort.cpp
 *
 *  Created on: Nov 27, 2022
 *      Author: bernardosantana
 */

#include <serialPort.h>


serialPort::serialPort()
{
	//empty
}

serialPort::serialPort(UART_HandleTypeDef* _uart) {
	this->uart = _uart;
	enableRxIT();
}

serialPort::~serialPort() {
	// TODO Auto-generated destructor stub
}


void serialPort::AssignUart(UART_HandleTypeDef* _uart)
{
	this->uart = _uart;
	enableRxIT();
}

void serialPort::Receive()
{
	rxBuffer.Write(&readByte, 1);
	enableRxIT();
}


void serialPort::Read(uint8_t* data, size_t len)
{
	rxBuffer.Read(data, len);
}

void serialPort::Send(uint8_t* data, size_t len)
{
	HAL_UART_Transmit_DMA(uart, data, len);
}

void serialPort::Peak(uint8_t* data, size_t len)
{
	rxBuffer.Peak(data, len);
}

size_t serialPort::Available()
{
	return rxBuffer.Available();
}


// Private

void serialPort::enableRxIT()
{
	HAL_UART_Receive_DMA(uart, &readByte, 1);
}
