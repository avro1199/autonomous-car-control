#include <BluetoothSerial.h>

#define limit 75 // Limit for the PID output

#define relay_fr 13
#define brake_mf 14 // Motor forward pin
#define brake_mb 27 // Motor backward pin
#define accelarator 26
#define steering 25

BluetoothSerial SerialBT; // Create Bluetooth Serial object

int tg_angle = 0; // Variable to store the target angle

// pid variables
float kp = 6, ki = 4.0, kd = 0; // PID coefficients
int previous_error = 0;         // Previous error for PID
int integral = 0;               // Integral term for PID

uint8_t speed = 0; // Speed variable for motor control

void setup()
{
    pinMode(steering, OUTPUT);    // Set DAC pin as output (not strictly necessary for DAC)
    pinMode(accelarator, OUTPUT); // Set GPIO 26 as output for DAC
    pinMode(relay_fr, OUTPUT);    // Set relay pin as output
    digitalWrite(relay_fr, HIGH); // Turn on the relay

    ledcAttach(brake_mf, 1000, 8);
    ledcAttach(brake_mb, 1000, 8);
    ledcWrite(brake_mf, 0);
    ledcWrite(brake_mb, 0);

    dacWrite(steering, 0);    // Initialize DAC output to 0
    dacWrite(accelarator, 0); // Initialize GPIO 26 output to 0
    // Attach GPIO 34 as input for analog reading
    pinMode(34, INPUT); // Set GPIO as input for analog reading

    SerialBT.begin("CarESP"); // Bluetooth device name
    SerialBT.println("Bluetooth Serial started. Pair and connect to 'CarESP'.");
}

void loop()
{
    if (SerialBT.available())
    {
        char cmd = SerialBT.read(); // Read command from Bluetooth
        if (cmd == 'S')
            1+1;
        else if (cmd >= '0' && cmd <= '9')
        {
            speed = (cmd - '0') * 255 / 10.0; // Convert char to int
            SerialBT.print("Speed set to: ");
            SerialBT.println(speed);
            dacWrite(accelarator, speed); // Set speed on GPIO 26
        }
        else if (cmd == 'F')
        {
            digitalWrite(relay_fr, HIGH); // Set relay to forward
            SerialBT.println("Relay set to Forward");
        }
        else if (cmd == 'B')
        {
            digitalWrite(relay_fr, LOW); // Set relay to backward
            SerialBT.println("Relay set to Backward");
        }
        else if (cmd == 'R')
        {
            tg_angle += 5; // Increase target angle
            if (tg_angle > 35)
                tg_angle = 35; // Limit to max angle
            SerialBT.print("Target Angle increased to: ");
            SerialBT.println(tg_angle);
            delay(50); // Delay to avoid rapid changes
        }
        else if (cmd == 'L')
        {
            tg_angle -= 5; // Decrease target angle
            if (tg_angle < -35)
                tg_angle = -35; // Limit to min angle
            SerialBT.print("Target Angle decreased to: ");
            SerialBT.println(tg_angle);
            delay(50); // Delay to avoid rapid changes
        }
        else if (cmd == 'u')
        {
            speed = 0;                    // Stop the motor
            dacWrite(accelarator, speed); // Set speed on GPIO 26
            ledcWrite(brake_mf, 250);     // Brake motor forward
            ledcWrite(brake_mb, 0);       // Stop motor backward
            SerialBT.println("Brake applied, Motor stopped");
            delay(700);             // Delay to allow motor to stop
            ledcWrite(brake_mf, 0); // Release brake motor forward
            ledcWrite(brake_mb, 0); // Release brake motor backwar
        }
        else if (cmd == 'U')
        {
            ledcWrite(brake_mf, 0);   // Release brake motor forward
            ledcWrite(brake_mb, 250); // Brake motor backward
            SerialBT.println("Brake released");
            delay(600);
            ledcWrite(brake_mf, 0); // Release brake motor forward
            ledcWrite(brake_mb, 0); // Release brake motor backward
        }
    }
    // Measure the current angle
    int current_angle = measure_angle();
    SerialBT.print("Current Angle: ");
    SerialBT.println(current_angle);

    // Calculate PID control
    int error = tg_angle - current_angle; // Calculate error
    if (abs(error) < 2)                   // If error is small, reset integral term
    {
        // ki = 0; // Reset integral term if error is zero
        integral = 0; // Reset integral term
    }
    integral += error;                                         // Update integral term
    int derivative = error - previous_error;                   // Calculate derivative term
    int output = kp * error + ki * integral + kd * derivative; // PID output
    previous_error = error;                                    // Update previous error for next iteration
    output = constrain(output, -limit, limit);                 // Constrain output to DAC range
    SerialBT.print("PID Output: ");
    SerialBT.println(output);
    // Write output to DAC
    dacWrite(steering, constrain(207 + output, 10, 255)); // Write to DAC pin 25
    delay(20);                                            // Delay for stability
}

int measure_angle()
{
    int analogValue = analogRead(34); // Use GPIO 34 as analog input
    // SerialBT.print("Raw Value: ");
    // SerialBT.print(analogValue);
    // SerialBT.print(" | ");
    analogValue = map(constrain(analogValue, 1300, 2500), 1300, 2500, -35, 35); // Map to a range of 0-100
    // SerialBT.print("Current Angle: ");
    // SerialBT.print(analogValue);
    // SerialBT.println(" degrees");
    return analogValue; // Return the target angle
}