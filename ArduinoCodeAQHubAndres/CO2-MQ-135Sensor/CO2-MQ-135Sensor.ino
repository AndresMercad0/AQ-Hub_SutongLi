//Coded and Tested By:
//Sheekar Banerjee, AI-ML, IOT Solution Engineer and Researcher
void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}
void loop() {
    int sensorValue = analogRead(A0); // read the input on analog pin 0:
    //Serial.println("The amount of CO2 (in PPM): ");
    Serial.println(sensorValue);
    delay(100);
} 