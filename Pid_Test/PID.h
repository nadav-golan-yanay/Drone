#ifndef PID_H
#define PID_H
 class PID {

  public:
   PID(int MotorPinFront, int MotorPinBack);
   void begin();   //must be called from  void setup()
   void MotorTest(int mot, int speed);
   void getattached(int mot);

  private:
    //variable:
    int _MotorPinFront = 0;
    int _MotorPinBack = 0;
   


 };
#endif 