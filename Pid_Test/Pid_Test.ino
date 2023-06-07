#include "clicli.h"
#include "PID.h"
PID mydrone(10, 9);
clicli mycli(mydrone);  

void setup() { 

  mydrone.begin();
  mycli.begin();
  Serial.println("Start");

 }

void loop() { 
  mycli.run();
  //mydrone.getattached(0);
 }