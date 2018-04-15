#ifndef __WIRING_ANALOG_H
#define __WIRING_ANALOG_H


#ifdef __cplusplus
 extern "C" {
#endif

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite(uint8_t ulPin, uint32_t ulValue);

/*
 * \brief Reads the value from the specified analog pin. Default is 10 bits (range from 0 to 1023).
 *
 * \param ulPin
 *
 * \return Read value from selected pin, if no error.
 */
extern uint32_t analogRead(uint8_t ulPin);

/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(uint32_t res);

/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(uint32_t res);

#ifdef __cplusplus
}
#endif
#endif  /* __WIRING_ANALOG_H */
