#include "Buffer.h"

Buffer::Buffer() {
    head = tail = 0;
    buffer[BUFFER_SIZE] = {0};
}

// Adds value to buffer, returns index where value is inserted
int Buffer::push(double value) {
    buffer[tail] = value;
    tail = (tail + 1) % BUFFER_SIZE;
    return tail;
}

double Buffer::pop() {
    double removed = buffer[head];
    head = (head - 1) % BUFFER_SIZE;
    if (head < 0) {
        // Negative value check
        head += BUFFER_SIZE;
    }
    return removed;
}

double Buffer::average() {
    double sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / BUFFER_SIZE;
}