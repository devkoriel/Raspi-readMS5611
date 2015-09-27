#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define MS5611_ADDRESS 0x77

#define RESET         0x1E
#define CONV_D1_256   0x40
#define CONV_D1_512   0x42
#define CONV_D1_1024  0x44
#define CONV_D1_2048  0x46
#define CONV_D1_4096  0x48
#define CONV_D2_256   0x50
#define CONV_D2_512   0x52
#define CONV_D2_1024  0x54
#define CONV_D2_2048  0x56
#define CONV_D2_4096  0x58
#define CMD_ADC_READ  0x00
#define CMD_PROM_READ 0xA0

#define OSR_256      1
#define OSR_512      2
#define OSR_1024     3
#define OSR_2048     5
#define OSR_4096     10

unsigned int PROM_read(int DA, int PROM_CMD)
{
	return wiringPiI2CReadReg16(DA, PROM_CMD);
}

void main()
{
	int Device_Adress;
	int i;

	unsigned int C[6];

	Device_Adress = wiringPiI2CSetup(MS5611_ADDRESS);

	for (i = 0; i < 7; i++)
	{
		C[i] = PROM_read(Device_Adress, CMD_PROM_READ + i * 2);

		printf("C[%d] = %d", i + 1, C[i]);
	}
}