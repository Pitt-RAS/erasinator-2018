#include "Arduino.h"
#include "EdgeDetector.h"

EdgeDetector::EdgeDetector(){
	qtra= QTRSensorsAnalog(IR_pins,(char)num_IR_sensors, IR_samples, 255);
	pinMode(right_bumper_pin, INPUT_PULLUP);
	pinMode(left_bumper_pin, INPUT_PULLUP);
	edge_detected=false;
}
void EdgeDetector::update(){
	qtra.read(IR_values);
	if(IR_values[0]>650 || IR_values[0]>650 || digitalRead(right_bumper_pin)==LOW || digitalRead(left_bumper_pin)==LOW){
//  if(digitalRead(right_bumper_pin)==LOW || digitalRead(left_bumper_pin)==LOW){
		edge_detected=true;
	}
	else{
		edge_detected=false;
	}
}
bool EdgeDetector::off_board(){
	return edge_detected;
}
bool EdgeDetector::insta_off_board(){
	update();
	return edge_detected;
}
