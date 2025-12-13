#include "iostream"
#include "Sensor.h"

int main()
{
    Sensor s;
    s.configure("dummy");
    s.cyclic();
   return 0; 
}