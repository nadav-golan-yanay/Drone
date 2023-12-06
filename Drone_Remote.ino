// Define your joystick and potentiometer pins
const int Thr = 36;
const int Eli = 39;
const int Ali = 34;
const int Rud = 35;

//const int pot1 = 32;
//const int pot2 = 33;

const int button1 = 25;
const int button2 = 26;
const int button3 = 27;
const int button4 = 14;

void setup(){
  Serial.begin(115200);
}


void loop() {
  // Read joystick and potentiometer values
  Serial.print(digitalRead(button4));
  Serial.print(" | ");
  Serial.print(digitalRead(button3));
  Serial.print(" | ");
  Serial.print(digitalRead(button2));
  Serial.print(" | ");
  Serial.print(digitalRead(button1));
  Serial.print(" | ");

  Serial.print(smooth(analogRead(Rud)));
  Serial.print(" | ");
  Serial.print(smooth(analogRead(Ali)));
  Serial.print(" | ");
  Serial.print(smooth(analogRead(Eli)));
  Serial.print(" | ");
  Serial.println(smooth(analogRead(Thr)));

  
}

int smooth(int val){
  int first = val;
  delay(1);
  int second = val;
  delay(1);
  int third = val;
  delay(1);
  int fourth = val;
  delay(1);
  int fifth = val;
  delay(1);

  int re = (first+second+third+fourth+fifth)/5;

  return(re);
}
