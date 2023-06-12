#include "clicli.h"
#include "PID.h"
PID mydrone(6, 5);
clicli mycli(mydrone);  

void setup() { 

  mydrone.begin();
  mycli.begin();
  Serial.println("Start");
  pinMode(13, INPUT_PULLUP);

 }

void loop() { 
  //mycli.run();
  mydrone.PIDcalc(mydrone.PichRead(), 0);
  //mydrone.Stab(0, 15, 1, 0, 0);
  //delay(10);
  /*Serial.print(mydrone.PIDcalc(mydrone.PichRead(), 0));
  Serial.print("     |        ");
  Serial.println(mydrone.PichRead());*/

 }