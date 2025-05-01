#ifndef THRUSTER_H
#define THRUSTER_H

#include "define.h"

enum DriverType
{
    CYTRON,     // 0
    BTS,        // 1 
};

class Thruster
{   

private:
    // Pins for the thruster
    int m_pins[2]; // [0] = dirPin, [1] = pWMPin for cytron and [0] = rightPWM, [1] = leftPWM for BTS
    
    DriverType m_eDriverType; // Type of driver (CYTRON or BTS)

    float m_speed; // Speed of the thruster

public:   
    
    // Constructor
    Thruster(int pin1 = 0, int pin2 = 0, DriverType eType = CYTRON);
    

    // Set the direction and speed of the thruster
    void setThruster(float speed);

    // Get the speed of the thruster
    void getSpeed();
    


}

#endif
