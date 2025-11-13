#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"

#include "gps/gps.h"
#include "gps/examples/gps_tester.h"


int main() {
    stdio_init_all();

    // test_timing();

    test_read_and_decode();
}

