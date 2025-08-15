#include "HardwareTimer.h"

#define debug_verbose
#define debug_minimum

#define CHANNELS 8
#define limit 40 // Limit for the PID output

#define PPM_PIN PA0 // PPM input pin (A0)

#define steering_pot PC1 // A4 for analog input (A4)
#define steering PA4     // DAC pin for steering output (A2)
#define accelarator PA5  // DAC pin for accelerator output (D13)

#define relay_fr PA7 // Relay pin for forward direction (D11)
#define horn     PB6 // Horn relay pin (D10)
#define brake_mf PC7 // Motor forward pin (D9)
#define brake_mb PA6 // Motor backward pin (D12)

HardwareTimer *MyTim;

// PPM Capture Variables
volatile uint16_t ppmValues[CHANNELS];
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastCapture = 0;

int target_angle = 0; // Variable to store the target angle

// pid variables
float kp = 6, ki = 1.0, kd = 0; // PID coefficients
int previous_error = 0;         // Previous error for PID
int integral = 0;               // Integral term for PID

uint8_t speed = 0; // Speed variable for motor control

bool braked = false; // Brake state variables

void IC_Handler()
{                                               // No parameters
    uint32_t now = MyTim->getCaptureCompare(1); // CCR1 value
    uint32_t diff = (now >= lastCapture) ? (now - lastCapture) : ((0xFFFF - lastCapture) + now);
    lastCapture = now;

    if (diff > 3000)
    {
        ppmIndex = 0; // new frame
    }
    else if (ppmIndex < CHANNELS)
    {
        ppmValues[ppmIndex] = diff;
        ppmIndex++;
    }
}

void setup()
{

    // TIM2 on PA0 (A0)
    MyTim = new HardwareTimer(TIM2);

    MyTim->setMode(1, TIMER_INPUT_CAPTURE_RISING, PPM_PIN); // channel 1, rising edge, pin
    MyTim->setOverflow(0xFFFF, MICROSEC_FORMAT);            // microsecond resolution
    MyTim->attachInterrupt(1, IC_Handler);                  // channel, callback
    MyTim->setInterruptPriority(0, 0);                      // highest priority
    MyTim->resume();

    // --------------------------------------------------------------------------------------
    Serial2.begin(115200);

    pinMode(steering_pot, INPUT);
    pinMode(steering, OUTPUT);
    pinMode(accelarator, OUTPUT);
    pinMode(brake_mf, OUTPUT); // Motor forward pin
    pinMode(brake_mb, OUTPUT); // Motor backward pin
    pinMode(relay_fr, OUTPUT); // Set relay pin as output
    pinMode(horn, OUTPUT);

    analogReadResolution(12); // Set ADC resolution to 12 bits
    analogWriteResolution(8); // Set PWM/DAC resolution to 8 bits
    analogWriteFrequency(20000);

    analogWrite(steering, 207);  // Initialize DAC output to 207
    analogWrite(accelarator, 0); //
    analogWrite(brake_mf, 0);    // Initialize motor forward pin to 0
    analogWrite(brake_mb, 0);    // Initialize motor backward pin to 0
    digitalWrite(relay_fr, HIGH);
    digitalWrite(horn, HIGH);

    // for (int i = 100; i <= 255; i += 5)
    // {
    //     analogWrite(steering, i); // Initialize steering output
    //     Serial2.print("Raw Value = ");
    //     Serial2.print(i);
    //     Serial2.println();
    //     delay(5000);
    // }
    // Serial2.println("Measurement complete, starting main loop...");
    // while(1) // Wait for a while before starting the main loop
    // {
    //     delay(1000);
    // }
}

void loop()
{
    // for (int i = 0; i < CHANNELS; i++)
    // {
    //     Serial2.print(ppmValues[i]);
    //     Serial2.print("  ");
    // }
    // Serial2.println();
    target_angle = map(constrain(ppmValues[0], 1030, 2010), 1030, 2010, -35, 35); // Map PPM value to angle range
#ifdef debug_verbose
    Serial2.print("Target Angle: ");
    Serial2.println(target_angle);
#endif

    speed = map(constrain(ppmValues[2], 1050, 2000), 1050, 2000, 0, 255); // Map PPM value to speed
    if (braked)
    {
        analogWrite(accelarator, 0);
    }
    else
    {
        analogWrite(accelarator, speed);
    }
#ifdef debug_verbose
    Serial2.print("Speed: ");
    Serial2.println(speed);
#endif

    // Control Forward/Backward Relay
    digitalWrite(relay_fr, ppmValues[4] > 1500 ? LOW : HIGH); // Forward if PPM value is above 1500
    // Control Horn
    digitalWrite(horn, ppmValues[3] < 1300 ? LOW : HIGH);

    // Control Motor Brake
    if (ppmValues[7] > 1500) // Brake if PPM value is above 1500
    {
        if (!braked)
        {
            braked = true;
            analogWrite(accelarator, 0); // Stop the motor
            analogWrite(brake_mf, 250);  // Brake motor forward
            analogWrite(brake_mb, 0);    // Stop motor backward
#if defined(debug_minimum) || defined(debug_verbose)
            Serial2.println("Brake applied, Motor stopped");
#endif
            delay(700);               // Delay to allow motor to stop
            analogWrite(brake_mf, 0); // Release brake motor forward
            analogWrite(brake_mb, 0); // Release brake motor backward
        }
    }
    else if (ppmValues[7] < 1500)
    {
        if (braked)
        {
            braked = false;
            analogWrite(brake_mf, 0);   // Brake motor forward
            analogWrite(brake_mb, 250); // Stop motor backward
#if defined(debug_minimum) || defined(debug_verbose)
            Serial2.println("Brake released, Motor running");
#endif
            delay(500);               // Delay to allow motor to run
            analogWrite(brake_mf, 0); // Release brake motor forward
            analogWrite(brake_mb, 0); // Release brake motor backward
        }
    }

    int current_angle = measure_angle();
#ifdef debug_verbose
    Serial2.print("Current Angle: ");
    Serial2.println(current_angle);
#endif

    // Calculate PID control
    int error = target_angle - current_angle; // Calculate error
    if (abs(error) < 2)                       // If error is small, reset integral term
    {
        // ki = 0; // Reset integral term if error is zero
        integral = 0; // Reset integral term
    }
    integral += error;                                         // Update integral term
    int derivative = error - previous_error;                   // Calculate derivative term
    int output = kp * error + ki * integral + kd * derivative; // PID output
    previous_error = error;                                    // Update previous error for next iteration
    output = constrain(output, -limit, limit);                 // Constrain output to DAC range
#if defined(debug_minimum) || defined(debug_verbose)
    Serial2.print("PID Output: ");
    Serial2.println(output);
#endif
    // Write output to DAC
    analogWrite(steering, constrain(196 + output, 10, 255)); // Write to DAC pin
    delay(20);
}

int measure_angle()
{
    int analogValue = analogRead(steering_pot); // analog input
#ifdef debug_verbose
    Serial2.print("Raw Value: ");
    Serial2.print(analogValue);
    Serial2.print(" | ");
#endif
    analogValue = map(constrain(analogValue, 1500, 2720), 1500, 2720, -35, 35); // Map to a range of 0-100
#ifdef debug_verbose
    Serial2.print("Current Angle: ");
    Serial2.print(analogValue);
    Serial2.println(" degrees");
#endif
    return analogValue; // Return the target angle
}