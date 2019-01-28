#ifndef BUFFER_H_
#define BUFFER_H_
/*
 * Circular Buffer implementation 
 * Buffer stores doubles
 * Xinke Chen 2018
 */

#define BUFFER_SIZE 10

class Buffer {
public:
    Buffer();
    int push(double value);
    double pop();
    double average();
private:
    double buffer[BUFFER_SIZE];
    int head;
    int tail;
    int length;
};
#endif
