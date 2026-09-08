#define DAC1 PA4    // A2
#define DAC2 PA5    // D13
#define RELAY1 PB5  // D4
#define RELAY2 PB4  // D5
#define RELAY3 PB10 // D6
#define RELAY4 PA8  // D7
#define MFWD PB6    // D10
#define MBKD PC7    // D9
#define ADC1 PC0    // A5
#define ADC2 PC1    // A4

#define limit 40 // Limit for the PID output

// RC mapping configuration
static const int RC_MIN = 1000;
static const int RC_MID = 1500;
static const int RC_MAX = 2000;
static const int RC_DEADBAND = 10; // around center (tune if needed)

// Failsafe: if no valid frame recently, stop motors
static const uint32_t FAILSAFE_US = 500000; // 500 ms
static uint32_t lastFrameUs = 0;

// -------------------- iBUS (UART) --------------------
static const uint32_t IBUS_BAUD = 115200;
static const uint8_t IBUS_FRAME_LEN = 32;

// HardwareSerial IBusSerial(2);
// static const int IBUS_RX_PIN = 16; // iBUS signal
// static const int IBUS_TX_PIN = 17; // unused

// iBUS frame buffer
static uint8_t frame[IBUS_FRAME_LEN];
static uint8_t idx = 0;

// Speed control
static uint16_t TOP_SPEED = 0;
static uint16_t BRAKE_THRESHOLD = 1300; // RC value below which braking is applied

HardwareSerial IBusSerial(PA10, PA9);

bool breaked = false;

int current_angle = 0; // Variable to store the current angle
int target_angle = 0;  // Variable to store the target angle

// pid variables
float kp = 6, ki = 1.0, kd = 0; // PID coefficients
int previous_error = 0;         // Previous error for PID
float integral = 0;             // Integral term for PID

void setup()
{
    Serial2.begin(115200); // gp

    // for Steering Feedback
    pinMode(ADC1, INPUT);
    pinMode(ADC2, INPUT);

    // for Relay Control
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(RELAY3, OUTPUT);
    pinMode(RELAY4, OUTPUT);
    // all off
    digitalWrite(RELAY1, HIGH); // Relay 1 OFF
    digitalWrite(RELAY2, HIGH); // Relay 2 OFF
    digitalWrite(RELAY3, HIGH); // Relay 3 OFF
    digitalWrite(RELAY4, HIGH); // Relay 4 OFF

    pinMode(MFWD, OUTPUT);
    pinMode(MBKD, OUTPUT);
    pinMode(DAC1, OUTPUT);
    pinMode(DAC2, OUTPUT);

    analogWrite(DAC1, 0);
    analogWrite(DAC2, 207);

    // Start iBUS serial
    IBusSerial.begin(IBUS_BAUD);
    lastFrameUs = micros();
}

void loop()
{
    // Read iBUS frame and print channel values
    uint16_t ch[14];
    if (readIbusFrame(ch))
    {
        if (ch[7] > RC_MID)
        {
            analogWrite(DAC1, 0);
            if (!breaked)
            {
                // break apply
                analogWrite(MFWD, 200);
                analogWrite(MBKD, 0);
                delay(700);
                analogWrite(MFWD, 0);
                analogWrite(MBKD, 0);
                breaked = true;
            }
        }
        else
        {
            if (breaked)
            {
                // break release
                analogWrite(MFWD, 0);
                analogWrite(MBKD, 200);
                delay(500);
                analogWrite(MFWD, 0);
                analogWrite(MBKD, 0);
                breaked = false;
            }
        }
        if (!breaked)
        {
            analogWrite(DAC1, map(ch[2], RC_MIN, RC_MAX, 0, 255)); // Throttle
        }
        else
        {
            analogWrite(DAC1, 0); // Throttle
        }

        if (ch[4] > RC_MID)
        {
            digitalWrite(RELAY4, LOW); // Relay 4 ON
        }
        else
        {
            digitalWrite(RELAY4, HIGH); // Relay 4 OFF
        }
        if (ch[5] > RC_MID)
        {
            digitalWrite(RELAY2, LOW); // Relay 2 ON
        }
        else
        {
            digitalWrite(RELAY2, HIGH); // Relay 2 OFF
        }
        if (ch[3] < RC_MID - 140)
        {
            digitalWrite(RELAY3, LOW); // Relay 3 ON
        }
        else
        {
            digitalWrite(RELAY3, HIGH); // Relay 3 OFF
        }

        target_angle = map(ch[0], RC_MIN, RC_MAX, -45, 45); // Map RC channel 0 to target angle (-45 to 45 degrees)

        Serial2.print("Target Angle: ");
        Serial2.print(target_angle);
    }
    current_angle = map(analogRead(ADC1), 0, 1023, -45, 45); // Map ADC1 reading to current angle (-45 to 45 degrees)
    Serial2.print(" -- Current Angle: ");
    Serial2.println(current_angle);

    // PID control
    int error = target_angle - current_angle; // Calculate error
    // if (abs(error) < 2)                   // If error is small, reset integral term
    // {
    //     // ki = 0; // Reset integral term if error is zero
    //     integral = 0; // Reset integral term
    // }
    integral += error; // Update integral term
    integral = constrain(integral, -limit * 1.0 / ki, limit * 1.0 / ki);
    int derivative = error - previous_error;                   // Calculate derivative term
    int output = kp * error + ki * integral + kd * derivative; // PID output
    previous_error = error;                                    // Update previous error for next iteration
    output = constrain(output, -limit, limit);                 // Constrain output to DAC range
    // Write output to DAC
    analogWrite(DAC2, constrain(196 + output, 10, 255)); // Write to DAC pin
    delay(20);
}