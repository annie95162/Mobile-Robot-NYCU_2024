// Arduino code to test ultrasonic sensor (e.g., HC-SR04)

#define TRIG_PIN 13  // Define the pin for the Trigger
#define ECHO_PIN 7 // Define the pin for the Echo

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set trig and echo pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Clear the trig pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Set the trig pin HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin and measure the duration of the pulse in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in centimeters
  long distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Wait for a short period before the next measurement
  delay(500);
}

