#include "clicli.h"
#include "PID.h"
PID mydrone(6, 5);
clicli mycli(mydrone);  

void setup() { 

  mydrone.begin();
  mycli.begin();
  Serial.println("Start");

 }

void loop() { 
  mycli.run();
  //mydrone.PIDcalc(mydrone.PichRead(), 0);

 }