/*
com.cpp - Implementación de las funciones de comunicacion

Jesus Rueda Gonzalez - 19/05/2015

UPCT
*/

#include <sstream>
#include <iostream>
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <string.h> //for errno
#include <errno.h> //error output
#include <string>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "control.h"
#include "main.h"

using namespace std;

void bluecom(struct mem_global *mem_global){
	
	// Find Serial device on Raspberry with ~ls /dev/tty*
	// ARDUINO_UNO "/dev/ttyACM0"
	// FTDI_PROGRAMMER "/dev/ttyUSB0"
	// HARDWARE_UART "/dev/ttyAMA0"
	char device[]= "/dev/ttyAMA0";
	// filedescriptor
	int fd;
	unsigned long baud = 9600;
	unsigned long time=0;

	cout << "Iniciando comunicacion" << endl;
	fflush(stdout);
	 
	//get filedescriptor
	if ((fd = serialOpen (device, baud)) < 0){
		cout << "No se puede abrir el puerto serie: " << strerror (errno) << endl;
	    exit(1); //error
	}
	
	while((*mem_global).salida){

		// Pong every 3 seconds
		if(millis()-time>=100){
		std::string mandar = "x = " + std::to_string((*mem_global).x) + " y = " + std::to_string((*mem_global).y) + "\n";
		serialPuts (fd, mandar.c_str());
			// you can also write data from 0-255
			// 65 is in ASCII 'A'
			//serialPutchar (fd, 65);
			time=millis();
		}
	 
		// read signal
		if(serialDataAvail (fd)){
		char newChar = serialGetchar (fd);
			if(newChar == -1){
				cout << "Error al leer carácter: Sin datos disponibles!" << endl;
			}
			else{
				cout << newChar;
				if (newChar == ':'){
					cout << "Recibido carácter de terminación de programa." << endl;
					(*mem_global).salida == false;
				}
				fflush(stdout);
			}
  		}	
	}

}
