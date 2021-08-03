/*
 *  string_util.c      Integer to string for sending integer values over USART
 *  String utility
 *  Created by Andrew Stange
 */

#include "kernel.h"
// The following function came from
// https://stackoverflow.com/questions/8257714/how-to-convert-an-int-to-string-in-c
// int_to_str
char* int_to_str(int value, char *result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}   // END int_to_str












//
//
//



void DisplayHeader(void) {
    USART2_send_str("    Accel   Gyro\r\n");
}

void DisplayIntegerValue(short value) {
    // Convert the number to string
    char valueAsString[12];
    int_to_str(value, valueAsString, 10);

    // Right justify display the string
    short strlen = 0;
    while(valueAsString[strlen])
    {
        strlen++;
    }

    while(strlen < 7)
    {
        USART2_send_str(" ");
        ++strlen;
    }

    USART2_send_str(valueAsString);
}

void DisplayAxisValue(char* label, short accel, short gyro) {
    USART2_send_str(label);
    USART2_send_str(":");
    DisplayIntegerValue(accel);
    DisplayIntegerValue(gyro);
}

void DisplayAxisValues(short accelX, short accelY, short accelZ, short gyroX, short gyroY, short gyroZ) {
//    short accelX, accelY, accelZ;
//    accelGetValues(&accelX, &accelY, &accelZ);
//    short gyroX, gyroY, gyroZ;
//    gyroGetValues(&gyroX, &gyroY, &gyroZ);

    DisplayAxisValue("X", accelX, gyroX);
    USART2_send_str("\r\n");
    DisplayAxisValue("Y", accelY, gyroY);
    USART2_send_str("\r\n");
    DisplayAxisValue("Z", accelZ, gyroZ);

    USART2_send_str("\033[2A");  // Makes cursor got up two lines
    USART2_send_str("\r");       // Return cursor to the beginning of the line
}



void DisplayIMUValue(char* label, short val) {
    USART2_send_str(label);
    USART2_send_str(":");
    DisplayIntegerValue(val);
}


void DisplayIMUValues(short roll, short pitch, short yaw) {
    DisplayIMUValue("Roll", roll);
    USART2_send_str("\r\n");
    DisplayIMUValue("Pitch", pitch);
    USART2_send_str("\r\n");
    DisplayIMUValue("Yaw", yaw);
    USART2_send_str("\033[2A");  // Makes cursor got up two lines
    USART2_send_str("\r");       // Return cursor to the beginning of the line
}




