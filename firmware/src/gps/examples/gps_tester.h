#ifndef _GPS_TESTER_H_
#define _GPS_TESTER_H_

#define GPS ((uart_inst_t *)uart1_hw) // Identifier for UART instance for gps
#define GPS_TX_PIN 20 // TX PIN
#define GPS_RX_PIN 21 // RX PIN


#define NUM_NMEA_STRINGS (6)
#define NMEA_STRING_SIZE (128)


void test_timing();
void test_read_and_decode();

#endif /* _GPS_TESTER_H_ */