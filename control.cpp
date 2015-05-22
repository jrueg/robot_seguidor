/*
control.cpp - Implementación de las funciones de control

Jesus Rueda Gonzalez - 19/05/2015

UPCT
*/

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

int controlador_pid::redefine(double _P, double _I, double _D, double _Ts, double _lim_sup, double _lim_inf)
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
