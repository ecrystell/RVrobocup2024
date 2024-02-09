#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>


bool compareWord(char *buf, const char *c_str) {
  return memcmp(buf, c_str, strlen(c_str)) == 0;
}

char *getNextWord(char *str, const char *delimiter) {
  char *pch;
  if (str != NULL) {
    pch = strtok(str, delimiter);
  }
  pch = strtok(NULL, delimiter);
  return pch;
}

long toHex(char *str) {
  char *endPtr;
  long convertedNumber = strtol(str, &endPtr, 16);

  // Check for conversion errors
  if (*endPtr != '\0') {
    return -1;
  } else {
    // Print the result as hexadecimal
    printf("Converted number (hex): 0x%lx\n", convertedNumber);

    // Print the result as decimal
    printf("Converted number (decimal): %ld\n", convertedNumber);
    return convertedNumber;
  }
}

#endif