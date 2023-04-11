#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <Arduino.h>
#include <SoftwareSerial.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define BAUDRATE 115200
#define port Serial1
#define ALL_CHANNELS 0

#define r_pin 9
#define t_pin 10


class FUTABA_SBUS
{
	public:
		uint8_t sbusData[25];
		int16_t channels[18];
		int16_t servos[18];
		uint8_t toSendArray[256];
		uint8_t  failsafe_status;
		int sbus_passthrough;
		int toChannels;
		void begin(void);
		int16_t Channel(uint8_t ch);
		uint8_t DigiChannel(uint8_t ch);
		void Servo(uint8_t ch, int16_t position);
		void DigiServo(uint8_t ch, uint8_t position);
		uint8_t Failsafe(void);
		void PassthroughSet(int mode);
		int PassthroughRet(void);
		void UpdateServos(void);
		void UpdateChannels(void);
		void FeedLine(void);
	private:
		uint8_t byte_in_sbus;
		uint8_t bit_in_sbus;
		uint8_t ch;
		uint8_t bit_in_channel;
		uint8_t bit_in_servo;
		uint8_t inBuffer[256];
		int bufferIndex;
		uint8_t inData;
		int feedState;
		SoftwareSerial* mySerial;

};

#endif