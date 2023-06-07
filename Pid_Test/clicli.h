#ifndef CLICLI_H
#define CLICLI_H
#include "PID.h"

 class clicli {

  public:
   clicli(PID &pid) ;
   void begin();   //must be called from  void setup()
   void run();   //must be called from  void loop()

  private:
   PID &mydrone;
   int number;

 };
#endif 