#ifndef Drone_H
#define Drone_H
 class Drone {

  public:
   Drone(int MotorPinFront, int MotorPinBack);
   void begin();   //must be called from  void setup()
   void MotorTest(int mot, int speed);
   void getattached(int mot);
   int PichRead();
  float PIDcalc(double inp, int sp);
  void Stab(int dag, int speed, float KP, float KI, float KD);
  void Fly(int powerF, int powerB);

  private:
    //variable:
    int _MotorPinFront = 0;
    int _MotorPinBack = 0;
   


 };
#endif 