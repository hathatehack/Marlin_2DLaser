/*
  Copyright (c) 2016 hathatehack  All right reserved.
  Copyright (c) 2011 Arduino.  All right reserved.
*/

#ifndef _WIRING_DIGITAL_H
#define _WIRING_DIGITAL_H


#ifdef __cplusplus
 extern "C" {
#endif

extern void pinMode(uint8_t PinNum, uint32_t PinMode);

extern void digitalWrite(uint8_t PinNum, uint8_t PinStatus);

extern int digitalRead(uint8_t PinNum);

#ifdef __cplusplus
}
#endif
#endif  /* __WIRING_DIGITAL_H */
