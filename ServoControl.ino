/*
 * Sketch to control the servo pins of Arduino via serial interface
 *
 */

#include <Servo.h>

char operation; // Holds operation (R, W, ...)
char mode; // Holds the mode (D, A)
int pin_number; // Holds the pin number
int digital_value; // Holds the digital value
int analog_value; // Holds the analog value
int value_to_write; // Holds the value that we want to write
int wait_for_transmission = 5; // Delay in ms in order to receive the serial data

// create servo object to control a servo
Servo SERVO3; 
Servo SERVO5;
Servo SERVO6;
Servo SERVO9;
Servo SERVO10;

// assigning servo object to ports on the arduino
int SERVO3_PIN = 3;
int SERVO5_PIN = 5;
int SERVO6_PIN = 6;
int SERVO9_PIN = 9;
int SERVO10_PIN = 10;

void setup() {
    Serial.begin(9600); // Serial Port at 9600 baud
    Serial.setTimeout(500); // Instead of the default 1000ms, in order
                            // to speed up the Serial.parseInt() 
    
    SERVO3.attach(SERVO3_PIN);
    SERVO3.write(0); // reset to original position

    SERVO5.attach(SERVO5_PIN);
    SERVO5.write(0); // reset to original position

    SERVO6.attach(SERVO6_PIN);
    SERVO6.write(0); // reset to original position

    SERVO9.attach(SERVO9_PIN);
    SERVO9.write(0); // reset to original position

    SERVO10.attach(SERVO10_PIN);
    SERVO10.write(180); // reset to original position
}

void servo_write(int pin_number, int servo_value){
    /*
     * Performs a servo write on pin_number with the servo_value
     * The value must be 0 to 180
     */
     
     if (pin_number==SERVO3_PIN)
     {
       SERVO3.write(servo_value);
       delay(10);
       
     } 
     
     else if (pin_number == SERVO5_PIN) 
     {
       SERVO5.write(servo_value);
       delay(10);
     }
     
     else if (pin_number == SERVO6_PIN) 
     {
       SERVO6.write(servo_value);
       delay(10);
     }

     else if (pin_number == SERVO9_PIN) 
     {
       SERVO9.write(servo_value);
       delay(10);
     }
     
     else if (pin_number == SERVO10_PIN) 
     {
       SERVO10.write(servo_value);
       delay(10);
     }
}

void loop() {
    // Check if characters available in the buffer
    if (Serial.available() > 0) 
    {
        // parse information
        // courtesy of lekum 
        operation = Serial.read();
        delay(wait_for_transmission); // If not delayed, second character is not correctly read
        mode = Serial.read();
        pin_number = Serial.parseInt(); // Waits for an int to be transmitted
        
        if (Serial.read()==':')
        {
            value_to_write = Serial.parseInt(); // Collects the value to be written
        }

        // if we recieve proper input write servo
        if (operation == 'W')
        {
            if (mode == 'S')
            {
                servo_write(pin_number, value_to_write);
            }
        }
        
    }
}
