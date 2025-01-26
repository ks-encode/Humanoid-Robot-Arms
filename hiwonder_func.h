#ifndef HIWONDER_FUNC_H
#define HIWONDER_FUNC_H
#include <Arduino.h>

void LobotSerialServoInit(HardwareSerial &SerialX, int TX_EN, int RX_EN);
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id);
int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id);
void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id);
void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed);
void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time);
void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id);
void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id);
int LobotSerialServoReadTemp(HardwareSerial &SerialX, uint8_t id);
int LobotSerialServoReadTempLimit(HardwareSerial &SerialX, uint8_t id);
void LobotSerialWriteAngleRange(HardwareSerial &SerialX, uint8_t id, int16_t min, uint16_t max);
void LobotSerialWriteTempLimit(HardwareSerial &SerialX, uint8_t id, int8_t temp);

#endif