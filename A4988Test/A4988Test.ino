int x;
void setup() { 
  pinMode(6,OUTPUT); // Enable 
  pinMode(5,OUTPUT); // Step 
  pinMode(4,OUTPUT); // Dir 
  digitalWrite(6,LOW); // Set Enable low 
}
void loop() {
  digitalWrite(4,HIGH); // Set Dir high
  for(x = 0; x < 500; x++) // Loop 200 times 
  { 
    digitalWrite(5,HIGH); // Output high 
    delayMicroseconds(500); // Wait 1/2 a ms 
    digitalWrite(5,LOW); // Output low 
    delayMicroseconds(500); // Wait 1/2 a ms 
  } 
  delay(1000); // pause one second
  digitalWrite(4,LOW); // Set Dir low
  for(x = 0; x < 900; x++) // Loop 2000 times 
  { 
    digitalWrite(5,HIGH); // Output high 
    delayMicroseconds(500); // Wait 1/2 a ms 
    digitalWrite(5,LOW); // Output low 
    delayMicroseconds(500); // Wait 1/2 a ms 
  } 
  delay(1000); // pause one second 
}
