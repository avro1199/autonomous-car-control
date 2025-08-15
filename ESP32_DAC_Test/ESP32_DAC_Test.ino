#define limit 75 // Limit for the PID output

int tg_angle = 0; // Variable to store the target angle

// pid variables
float kp = 6, ki = 4.0, kd = 0; // PID coefficients
int previous_error = 0;     // Previous error for PID
int integral = 0;           // Integral term for PID

void setup()
{
    Serial.begin(115200); // Start serial communication
    pinMode(25, OUTPUT);  // Set DAC pin as output (not strictly necessary for DAC)
    pinMode(34, INPUT);   // Set GPIO as input for analog reading
    Serial.println("Enter a value for Target Angle (-35, 35):");
}

void loop()
{
    if (Serial.available())
    {
        int value = Serial.parseInt(); // Read integer from serial
        if (value >= -35 && value <= 35)
        {
            tg_angle = value; // Set target angle
            Serial.print("Target Angle set to: ");
            Serial.println(tg_angle);
        }
        else
        {
            Serial.println("Invalid input. Please enter a value between -35 and 35.");
        }
        // Clear any remaining input
        while (Serial.available())
            Serial.read();
    }

    // Measure the current angle
    int current_angle = measure_angle();
    Serial.print("Current Angle: ");
    Serial.println(current_angle);

    // Calculate PID control
    int error = tg_angle - current_angle;                      // Calculate error
    if(abs(error) < 2) // If error is small, reset integral term
    {
        // ki = 0; // Reset integral term if error is zero
        integral = 0; // Reset integral term
    }
    integral += error;                                         // Update integral term
    int derivative = error - previous_error;                   // Calculate derivative term
    int output = kp * error + ki * integral + kd * derivative; // PID output
    previous_error = error;                                    // Update previous error for next iteration
    output = constrain(output, -limit, limit);                       // Constrain output to DAC range
    Serial.print("PID Output: ");
    Serial.println(output);
    // Write output to DAC
    dacWrite(25, constrain(207 + output, 10, 255)); // Write to DAC pin 25
    delay(20);                 // Delay for stability
}

int measure_angle()
{
    int analogValue = analogRead(34); // Use GPIO 34 as analog input
    Serial.print("Raw Value: ");
    Serial.print(analogValue);
    Serial.print(" | ");
    analogValue = map(constrain(analogValue, 1300, 2500), 1300, 2500, -35, 35); // Map to a range of 0-100
    Serial.print("Current Angle: ");
    Serial.print(analogValue);
    Serial.println(" degrees");
    return analogValue; // Return the target angle
}