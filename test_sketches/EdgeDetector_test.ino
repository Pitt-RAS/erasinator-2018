#include "EdgeDetector.h"
EdgeDetector eDetect= EdgeDetector();
void setup() {
  // put your setup code here, to run once:
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  pinMode(13,OUTPUT);
  //eDetect = EdgeDetector();
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13,LOW);
}

void loop() {
  eDetect.update();
  if(eDetect.off_board()){
    digitalWrite(13,HIGH);
  }
  else{
    digitalWrite(13,LOW);
  }
  delay(300);
}
