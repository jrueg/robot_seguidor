/*
control.cpp - Implementación de las funciones de control

Jesus Rueda Gonzalez - 19/05/2015

UPCT
*/

#include <string>
#include <wiringPi.h>
#include <softPwm.h>
#include "control.h"

controlador_pid::controlador_pid(double _P, double _I, double _D, double _Ts, double _lim_sup, double _lim_inf)
{
	this->P = _P;
	this->I = _I;
	this->D = _D;
	this->Ts = _Ts;
	this->lim_sup = _lim_sup;
	this->lim_inf = _lim_inf;
	this->derivada = 0;
	this->ref = 0;
	this->realim = 0;
	this->integral = 0;
}

controlador_pid::controlador_pid(double _P, double _I, double _D, double _lim_sup, double _lim_inf)
{
	this->P = _P;
	this->I = _I;
	this->D = _D;
	this->Ts = 0;
	this->lim_sup = _lim_sup;
	this->lim_inf = _lim_inf;
	this->derivada = 0;
	this->ref = 0;
	this->realim = 0;
	this->integral = 0;
}

int controlador_pid::reset()
{
	this->derivada = 0;
	this->ref = 0;
	this->integral = 0;
	return(0);
}

int controlador_pid::setpoint(double _ref)
{
	this->ref = _ref;
	return(0);
}

int controlador_pid::feedback(double _realim)
{
	this->realim = _realim;
	return(0);
}

double controlador_pid::error()
{
	return(derivada);
}

double controlador_pid::calculo()
{
	double error, salida;

	if (Ts == 0){
		return(-1);
	}

	error = ref - realim;

	salida = error*P + integral*I*Ts + (error - derivada) / Ts*D;
	derivada = error;

	if (salida > lim_sup)
	{
		salida = lim_sup;
	}
	else if (salida < lim_inf)
	{
		salida = lim_inf;
	}
	else
	{
		integral += error;
	}

	return(salida);
}

double controlador_pid::calculo(double _realim)
{
	double error, salida;

	realim = _realim;

	if (Ts == 0){
		return(-1);
	}

	error = ref - realim;

	salida = error*P + integral*I*Ts + (error - derivada) / Ts*D;
	derivada = error;

	if (salida > lim_sup)
	{
		salida = lim_sup;
	}
	else if (salida < lim_inf)
	{
		salida = lim_inf;
	}
	else
	{
		integral += error;
	}

	return(salida);
}

double controlador_pid::calculo(double _realim, double _Ts)
{
	double error, salida;

	realim = _realim;
	Ts = _Ts;

	error = ref - realim;

	salida = error*P + integral*I*Ts + (error - derivada) / Ts*D;
	derivada = error;

	if (salida > lim_sup)
	{
		salida = lim_sup;
	}
	else if (salida < lim_inf)
	{
		salida = lim_inf;
	}
	else
	{
		integral += error;
	}

	return(salida);
}

int controlador_pid::redefine(double _P, double _I, double _D, double _lim_sup, double _lim_inf)
{
	this->P = _P;
	this->I = _I;
	this->D = _D;
	this->Ts = 0;
	this->lim_sup = _lim_sup;
	this->lim_inf = _lim_inf;
	this->derivada = 0;
	this->ref = 0;
	this->realim = 0;
	this->integral = 0;
	return(0);
}

int controlador_pid::redefine(double _P, double _I, double _D, double _Ts, double _lim_sup, double _lim_inf)
{
	this->P = _P;
	this->I = _I;
	this->D = _D;
	this->Ts = _Ts;
	this->lim_sup = _lim_sup;
	this->lim_inf = _lim_inf;
	this->derivada = 0;
	this->ref = 0;
	this->realim = 0;
	this->integral = 0;
	return(0);
}


controlador_p::controlador_p(double _P, double _lim_sup, double _lim_inf){
	P = _P;
	Ts = 0;
	ref = 0;
	realim = 0;
	lim_sup = _lim_sup;
	lim_inf = _lim_inf;
}

controlador_p::controlador_p(double _P, double _Ts, double _lim_sup, double _lim_inf){
	P = _P;
	Ts = _Ts;
	ref = 0;
	realim = 0;
	lim_sup = _lim_sup;
	lim_inf = _lim_inf;
}

int controlador_p::setpoint(double _ref){
	ref = _ref;
	return(0);
}

int controlador_p::feedback(double _realim){
	realim = _realim;
	return(0);
}

double controlador_p::calculo(){
	double error, salida;

	if (Ts == 0){
		return(-1);
	}

	error = ref - realim;

	salida = error*P;

	if (salida > lim_sup)
	{
		salida = lim_sup;
	}
	else if (salida < lim_inf)
	{
		salida = lim_inf;
	}

	return(salida);
}

double controlador_p::calculo(double _realim){
	double error, salida;

	realim = _realim;

	if (Ts == 0){
		return(-1);
	}

	error = ref - realim;

	salida = error*P;

	if (salida > lim_sup)
	{
		salida = lim_sup;
	}
	else if (salida < lim_inf)
	{
		salida = lim_inf;
	}

	return(salida);
}

double controlador_p::calculo(double _realim, double _Ts){
	double error, salida;

	realim = _realim;
	Ts = _Ts;

	if (Ts == 0){
		return(-1);
	}

	error = ref - realim;

	salida = error*P;

	if (salida > lim_sup)
	{
		salida = lim_sup;
	}
	else if (salida < lim_inf)
	{
		salida = lim_inf;
	}

	return(salida);
}

int controlador_p::redefine(double _P, double _lim_sup, double _lim_inf){
	P = _P;
	Ts = 0;
	ref = 0;
	realim = 0;
	lim_sup = _lim_sup;
	lim_inf = _lim_inf;
	return(0);
}

int controlador_p::redefine(double _P, double _Ts, double _lim_sup, double _lim_inf){
	P = _P;
	Ts = _Ts;
	ref = 0;
	realim = 0;
	lim_sup = _lim_sup;
	lim_inf = _lim_inf;
	return(0);
}

motor_dc::motor_dc(unsigned char _Pin_EN, unsigned char _Pin_C1, unsigned char _Pin_C2){
	Pin_EN = _Pin_EN;
	Pin_C1 = _Pin_C1;
	Pin_C2 = _Pin_C2;
	vel = 0;

	pinMode(Pin_C1, OUTPUT);
	pinMode(Pin_C2, OUTPUT);
	softPwmCreate(_Pin_EN, 0, 100);

	digitalWrite(Pin_C1, 0);
	digitalWrite(Pin_C2, 0);
	softPwmWrite(Pin_EN, vel);
}

motor_dc::~motor_dc(){
	digitalWrite(Pin_C1, 0);
	digitalWrite(Pin_C2, 0);
	vel = 0;
	softPwmWrite(Pin_EN, vel);
}

void motor_dc::velocidad(int _vel){
	int uvel;

	if (_vel == 0){
		digitalWrite(Pin_C1, 0);
		digitalWrite(Pin_C2, 0);
	}

	if (_vel < 0 && vel >= 0){
		digitalWrite(Pin_C1, 0);
		digitalWrite(Pin_C2, 0);
		digitalWrite(Pin_C2, 1);
	}

	if (_vel > 0 && vel <= 0){
		digitalWrite(Pin_C2, 0);
		digitalWrite(Pin_C1, 0);
		digitalWrite(Pin_C1, 1);
	}

	vel = _vel;

	if (vel < 0){
		uvel = -vel;
	}
	else{
		uvel = vel;
	}

	if (uvel < 0) uvel = 0;
	if (uvel > 100) uvel = 100;

	softPwmWrite(Pin_EN, uvel);

}

void servoBlaster(unsigned char pin, int vel){
	// Correccion de la velocidad en limites
	if (vel < 0) vel = 0;
	if (vel > 100) vel = 100;
	// Envio del comando en porcentaje por consola
	std::string servoPos = "echo " + std::to_string(pin) + "=" + std::to_string(vel) + "% > /dev/servoblaster";
	system(servoPos.c_str());
}

sonar::sonar(unsigned char _echo, unsigned char _trig){
	echo = _echo;
	trig = _trig;

	pinMode(echo, INPUT);
	pinMode(trig, OUTPUT);
	digitalWrite(trig, 0);
}

int sonar::dist(){
	//Mandar pulso
	digitalWrite(trig, 1);
	delayMicroseconds(20);
	digitalWrite(trig, 0);

	//Esperar al eco
	while (digitalRead(echo) == 0);

	//Wait for echo end
	long tiempo_inicial = micros();
	while (digitalRead(echo) == 1);
	long tiempo_vuelo = micros() - tiempo_inicial;

	//Distancia (en centimetros)
	return tiempo_vuelo / 58;
}