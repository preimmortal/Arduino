#ifndef _TEMP_H_
#define _TEMP_H_

#include <dht11.h>
#include <math.h>

//Function Declarations
double Fahrenheit(double celsius);
double Kelvin(double celsius);
double dewPoint(double celsius, double humidity);
double dewPointFast(double celsius, double humidity);


#endif