String data;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!Serial.available()){delay(100);}
  while(Serial.available())                                 
    {
      data = Serial.readString();
    }
  Serial.println("echo:");
  Serial.println(data);
  data="";
  delay(100);
}
