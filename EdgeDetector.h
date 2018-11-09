#ifndef EdgeDetector_h
#define EdgeDetector_h

#include "Arduino.h"
#include "QTRSensors.h"


class EdgeDetector{
	public:
		EdgeDetector();
		
		void update();
		bool off_board();
		bool insta_off_board();
		
	private:
		int right_bumper_pin=17;
		int left_bumper_pin=16;
		int right_IR_pinA=A0;
		int left_IR_pinA=A1;
		int num_IR_sensors=2;
		int IR_samples=4;
    unsigned char IR_pins[2] = {right_IR_pinA,left_IR_pinA};
		bool edge_detected;
		unsigned int IR_values[2];
    QTRSensorsAnalog qtra;
};

#endif
