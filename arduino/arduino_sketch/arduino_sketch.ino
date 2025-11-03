void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Hello from Arduino sketch!");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
