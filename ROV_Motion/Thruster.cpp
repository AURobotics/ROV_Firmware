#include "Thruster.h"

Thruster::Thruster(int pin1, int pin2, DriverType eType)
{
        m_pins[0] = pin1;
        m_pins[1] = pin2;
        m_eDriverType = eType;

        pinMode(m_pins[0], OUTPUT);   // Set direction pin as output
        pinMode(m_pins[1], OUTPUT);   // Set speed pin as output
        m_speed = 0;                // Initialize speed to 0
}

void Thruster::setThruster(float speed)
{
    speed = int(speed); // Convert speed to integer
    m_speed = speed; // Store the speed

    if (m_eDriverType == CYTRON)
    {
        // Set the direction and speed for CYTRON driver
        digitalWrite(m_pins[0], (speed >= 0) ? HIGH : LOW); // Set direction
        analogWrite(m_pins[1], int(abs(speed)));           // Set speed
    }
    else if (m_eDriverType == BTS)
    {
        // Set the speed for BTS driver
        if (speed >= 0)
        {
            analogWrite(m_pins[0], int(abs(speed))); // Set right PWM
            analogWrite(m_pins[1], 0);               // Set left PWM to 0
        }
        else
        {
            analogWrite(m_pins[0], 0);               // Set right PWM to 0
            analogWrite(m_pins[1], int(abs(speed))); // Set left PWM
        }
    }
}

float Thruster::getSpeed()
{
    return m_speed;
}