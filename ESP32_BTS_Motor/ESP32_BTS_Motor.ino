#define mf 14
#define mb 27

void setup()
{
    Serial.begin(115200);
    ledcAttach(mf, 1000, 8);
    ledcAttach(mb, 1000, 8);
    ledcWrite(mf, 0);
    ledcWrite(mb, 0);
    Serial.println("Motor Driver Check Started...");
    Serial.println("Send F<speed>,<on_time_ms> for forward, B<speed>,<on_time_ms> for backward, S for stop.");
}

void runMotor(int forwardSpeed, int backwardSpeed, int onTime) {
    ledcWrite(mf, forwardSpeed);
    ledcWrite(mb, backwardSpeed);
    delay(onTime);
    ledcWrite(mf, 0);
    ledcWrite(mb, 0);
    Serial.println("Motor Stopped after on-time");
}

void loop()
{
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.startsWith("F") || cmd.startsWith("B")) {
            int commaIdx = cmd.indexOf(',');
            if (commaIdx > 1) {
                int speed = cmd.substring(1, commaIdx).toInt();
                int onTime = cmd.substring(commaIdx + 1).toInt();
                if (cmd.startsWith("F")) {
                    Serial.print("Forward: ");
                    Serial.print(speed);
                    Serial.print(" for ");
                    Serial.print(onTime);
                    Serial.println(" ms");
                    runMotor(speed, 0, onTime);
                } else {
                    Serial.print("Backward: ");
                    Serial.print(speed);
                    Serial.print(" for ");
                    Serial.print(onTime);
                    Serial.println(" ms");
                    runMotor(0, speed, onTime);
                }
            } else {
                Serial.println("Invalid command format. Use F<speed>,<on_time_ms>");
            }
        } else if (cmd.equalsIgnoreCase("S")) {
            ledcWrite(mf, 0);
            ledcWrite(mb, 0);
            Serial.println("Motor Stopped");
        } else {
            Serial.println("Invalid command");
        }
    }
}
