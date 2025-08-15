#define PPM_PIN 2  // Use an interrupt-capable pin (D2 for example)
#define CHANNELS 8

volatile uint16_t ppmValues[CHANNELS];
volatile byte channelIndex = 0;
volatile unsigned long lastMicros = 0;

void ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) { // Start of frame detected
    channelIndex = 0;
  } else if (channelIndex < CHANNELS) {
    ppmValues[channelIndex] = diff;
    channelIndex++;
  }
}

void setup() {
  Serial2.begin(115200);
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
}

void loop() {
  // Just print channel values
  Serial2.print("PPM: ");
  for (byte i = 0; i < CHANNELS; i++) {
    Serial2.print(ppmValues[i]);
    Serial2.print(" ");
  }
  Serial2.println();
  delay(5);
}