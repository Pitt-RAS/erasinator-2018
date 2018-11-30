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
        unsigned char right_bumper_pin=17;
        unsigned char left_bumper_pin=16;
        unsigned char right_IR_pinA=A0;
        unsigned char left_IR_pinA=A1;
        int num_IR_sensors=2;
        int IR_samples=4;
        unsigned char IR_pins[2] = {right_IR_pinA,left_IR_pinA};
        bool edge_detected;
        unsigned int IR_values[2];
    QTRSensorsAnalog qtra;
};

#endif
