/*

                    CLI Software for Arduino

               A Simple Command Line Interface 
                
              Feature                |  CLI Usage
___________________________________________________
 Digial Write HIGH to a specific pin |  h (pin)
 Digial Write LOW to a specific pin  |  l (pin)
 Analog Write to pwm ports           |  a (pin) (value)
 Digital Read                        |  r (pin)
 Analog Read                         |  e (pin) 

               
      by Gal Arbel
      Oct 2022

      Credits: Shimi Mahluf 

*/


#include "clicli.h"
#include "Arduino.h"
#include "PID.h"

const unsigned int MAX_MESSAGE_LENGTH = 64;

clicli::clicli(PID &pid):mydrone(pid), number(7) {
}

void clicli::begin() {
  Serial.begin(115200);
}
void clicli::run() {

// CLI - Messages from Terminal
  while (Serial.available() > 0) { 
   char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;
   char inByte = Serial.read();   //Read the next available byte in the serial receive buffer
    if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
     {
     message[message_pos] = inByte;  //Add the incoming byte to our message
     message_pos++;
     }
     //Full message received...
     else
     {
      message[message_pos] = '\0';     //Add null character to string
      Serial.println(message);     //echo the message to terminal
        
      float command[10];
      int argindex = 0;
      char cmd;
      char delim[] = " ";
	     char tmpmsg[MAX_MESSAGE_LENGTH];
       strcpy(tmpmsg,message);
       message_pos = 0;
       message[message_pos] = '\0';     //Add null character to string

        char *ptr = strtok(tmpmsg, delim);
	      while(ptr != NULL)
	       {
		      //Serial.printf("'%s'\n", ptr);
          if (argindex == 0) {
            cmd = ptr[0];
          }
          command[argindex] = atof(ptr);   
          //Serial.println(command[argindex]);
          argindex++;  
		      ptr = strtok(NULL, delim);
	       } 

      switch (cmd) {

      case '?': //Help ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        Serial.print("This function");
        delay(300);
        Serial.print(" will help");
        delay(300);
        Serial.print(" you navigate");
        delay(300);
        Serial.print(" your way");
        delay(300);
        Serial.println(" in the code:");
        Serial.println(" ");

        delay(400);
        Serial.print("Each commend activate");
        delay(300);
        Serial.print(" with a combination.");
        delay(300);
        Serial.print(" of a single");
        delay(300);
        Serial.print(" LETTER and nombers");
        delay(300);
        Serial.println(" thats comes after.");
        delay(400);
        Serial.println(" ");

        Serial.println("The order is very important!!!");
        delay(400);
        Serial.println(" ");

        Serial.println("ALL letters in lowercase letters!!!");
        delay(450);
        Serial.println(" ");

        Serial.println("List of orders:");
        
        delay(450); //case h
        Serial.print("For");
        delay(300);
        Serial.print(" seting port to HIGH");
        delay(300);
        Serial.print(" - Enter the LETTER");
        delay(300);
        Serial.print(" 'h'");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.println(" the port Nomber");
        Serial.println(" ");

        delay(450); //case l
        Serial.print("For");
        delay(300);
        Serial.print(" seting port to LOW");
        delay(300);
        Serial.print(" - Enter the LETTER");
        delay(300);
        Serial.print(" 'l'");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.println(" the port Nomber");
        Serial.println(" ");

        delay(450); //case r
        Serial.print("For");
        delay(300);
        Serial.print(" digital read from port");
        delay(300);
        Serial.print(" - Enter the LETTER");
        delay(300);
        Serial.print(" 'r'");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.println(" the port Nomber");
        Serial.println(" ");

        delay(450); //case e
        Serial.print("For");
        delay(300);
        Serial.print(" analog read from port");
        delay(300);
        Serial.print(" - Enter the LETTER");
        delay(300);
        Serial.print(" 'e'");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.println(" the port Nomber");
        Serial.println(" ");

        delay(450); //case t
        Serial.print("For");
        delay(300);
        Serial.print(" testing drone motors");
        delay(300);
        Serial.print(" - Enter the LETTER");
        delay(300);
        Serial.print(" 't'");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.print(" the motor port Nomber");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.println(" the speed Nomber");
        Serial.println(" ");

        delay(450); //case s
        Serial.print("For");
        delay(300);
        Serial.print(" PID fly");
        delay(300);
        Serial.print(" - Enter the LETTER");
        delay(300);
        Serial.print(" 's'");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.print(" the motor port Nomber");
        delay(300);
        Serial.print(" and then enter");
        delay(300);
        Serial.println(" the speed Nomber");
        Serial.println(" ");

        

      break;



       case 'h': //Set port to HIGH
        pinMode(command[1],OUTPUT);
        digitalWrite(command[1],HIGH);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.println(" is SET");   
        delay(1000);
        break;
       case 'l': // Set port to LOW
        pinMode(command[1],OUTPUT);
        digitalWrite(command[1],LOW);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.println(" is RESET");   
        delay(1000);
        break;
       /*
      case 'a': // analog Write to pwm ports
        pinMode(command[1],OUTPUT);
        analogWrite(command[1],command[2]);
        Serial.print("Writing "); 
        Serial.print(command[2]);   
        Serial.print(" to pin ");  
        Serial.println(command[1]);   
        delay(1000);
        break;
							*/
       case 'r': // digital read
        pinMode(command[1],INPUT);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.print(" Value = "); 
        Serial.println(digitalRead(command[1]));   
        delay(1000);
        break;

        case 'e': // analog read
        pinMode(command[1],INPUT);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.print(" Value = "); 
        Serial.println(analogRead(command[1]));   
        delay(1000);
        break;

        case 't': //test drone motors
        mydrone.MotorTest(command[1], command[2]);
        break;

        case 's':
        while (true){
          mydrone.Stab(command[1], command[2], command[3], command[4], command[5]);
          if (digitalRead(13) == 0){
            mydrone.Fly(0, 0);
            break;
          }
        }
        break;

        case 'c':
        mydrone.Fly(0, 0);
        break;

        case 'f':
        mydrone.Fly(command[1], command[2]);
        break;

        case 'g':
        while (true){
          if (digitalRead(13) == 0){
            Serial.println(mydrone.PichRead());
            break;
          }
        }
        break;

        
       
       message_pos = 0;     //Reset for the next message
      }
   }
   delay (60);
   
 } 

}
