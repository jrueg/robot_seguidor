#include <sstream>
#include <string>
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include <time.h>
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <wiringPi.h>
#include <wiringSerial.h>
#include <errno.h>
#include "control.h"
#include "main.h"

using namespace std;

int main(int argc, char* argv[])
{
	// Valores iniciales de la memoria compartida
	struct mem_global mem_global;
	
	mem_global.H_MIN = 15;
	mem_global.H_MAX = 50;
	mem_global.S_MIN = 150;
	mem_global.S_MAX = 256;
	mem_global.V_MIN = 0;
	mem_global.V_MAX = 256;
	mem_global.x = 0;
	mem_global.y = 0;
	mem_global.vel = 0;
	mem_global.salida = true;
	
	//setup GPIO in wiringPi mode
	if (wiringPiSetup () == -1){
		cout << "Imposible iniciar wiringPi: " << strerror (errno) << endl;
		exit(1); //error
	}	

	//Lanza thread seguimiento
	std::thread th_seguimiento(seguimiento, &mem_global);
	
	//Lanza thread comunicaciones
	std::thread th_bluecom(bluecom, &mem_global);

	while (mem_global.salida){
		cout << "Desde thread principal: x = " << mem_global.x << " y = " << mem_global.y << endl;
		delay(500);
	}

	cout << "Terminando programa. Esperando threads secundarios..." << endl;

	th_seguimiento.join();
	th_bluecom.join();

	cout << "FIN" << endl;

	return 0;
}
