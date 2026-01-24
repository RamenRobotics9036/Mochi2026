// Code for the arduino
// Detects when a coral is present in the intake
// Whenever either proximity sensor running averager is over a threshhold


const int analogPin0 = A0;
const int analogPin1 = A1;
const int outputPin = 2;
const int ledPin = 13;

const int sampleInterval = 9; // milliseconds
const int numSamples = 5;
const int THRESHOLD = 600; // Changed threshold

int samples0[numSamples] = {0};
int samples1[numSamples] = {0};
int sampleIndex = 0;

unsigned long lastSampleTime = 0;

void setup() {
  pinMode(outputPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    // Read sensors
    int reading0 = analogRead(analogPin0);
    int reading1 = analogRead(analogPin1);

    // Store readings
    samples0[sampleIndex] = reading0;
    samples1[sampleIndex] = reading1;

    // Advance index
    sampleIndex++;
    if (sampleIndex >= numSamples) {
      sampleIndex = 0;
    }

    // Compute averages
    int sum0 = 0;
    int sum1 = 0;
    for (int i = 0; i < numSamples; i++) {
      sum0 += samples0[i];
      sum1 += samples1[i];
    }

    int avg0 = sum0 / numSamples;
    int avg1 = sum1 / numSamples;

    // Output to serial
    Serial.print("min:0 max:1024 t:");
    Serial.print(THRESHOLD);
    Serial.print(" Avg0:");
    Serial.print(avg0);
    Serial.print("  Avg1:");
    Serial.println(avg1);

    // Output logic
    if (avg0 > THRESHOLD || avg1 > THRESHOLD) {
      digitalWrite(outputPin, HIGH);
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(outputPin, LOW);
      digitalWrite(ledPin, LOW);
    }
  }
}
