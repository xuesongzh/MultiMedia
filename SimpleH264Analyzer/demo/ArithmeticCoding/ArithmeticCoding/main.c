#include <stdio.h>

#include "ArithmeticCoder.h"

int main() {                                // 0 1 1 0
    unsigned int input_data[] = {0, 0, 1};  // 011
    unsigned int input_length = sizeof(input_data) / sizeof(unsigned int);

    for (int idx = 0; idx < input_length; idx++) {
        Encode_one_symbol(input_data[idx]);
    }
    Encode_stop();

    return 0;
}