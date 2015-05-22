/*
control.h - Encabezado de las funciones de control

Jesus Rueda Gonzalez - 19/05/2015

UPCT
*/

#ifndef _FUNC_CONTROL_H
#define _FUNC_CONTROL_H

class controlador_pid
{
public:
	controlador_pid(double _P, double _I, double _D, double _lim_sup, double _lim_inf);
	controlador_pid(double _P, double _I, double _D, double _Ts, double _lim_sup, double _lim_inf);
	int reset();
	int setpoint(double _ref);
	int feedback(double _realim);
	double calculo();
	double calculo(double _realim);
	double calculo(double _realim, double _Ts);
	int redefine(double _P, double _I, double _D, double _lim_sup, double _lim_inf);
	int redefine(double _P, double _I, double _D, double _Ts, double _lim_sup, double _lim_inf);
	double error();

private:
	double P;
	double I;
	double D;
	double integral;
	double derivada;
	double ref;
	double Ts;
	double lim_sup;
	double lim_inf;
	double realim;
};

class controlador_p{
public:
	controlador_p(double _P, double _lim_sup, double _lim_inf);
	controlador_p(double _P, double _Ts, double _lim_sup, double _lim_inf);
	int setpoint(double _ref);
	int feedback(double _realim);
	double calculo();
	double calculo(double _realim);
	double calculo(double _realim, double _Ts);
	int redefine(double _P, double _lim_sup, double _lim_inf);
	int redefine(double _P, double _Ts, double _lim_sup, double _lim_inf);

private:
	double P;
	double Ts;
	double ref;
	double realim;
	double lim_inf;
	double lim_sup;
};

#endif