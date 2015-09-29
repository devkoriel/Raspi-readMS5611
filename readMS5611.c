#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/poll.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

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
	unsigned int PROM_DATA;

	wiringPiI2CWrite(DA, PROM_CMD);
	read(DA, PROM_DATA, 2);
	close(DA);

	return PROM_DATA;
}

/*unsigned int BARO_read(int DA, int BARO_CMD)
{
	return i2c_smbus_read_block_data(DA, BARO_CMD);
}*/

void main()
{
	int Device_Address;
	int i;

	unsigned int C[6];
	unsigned long D1;
	unsigned long D2;

	Device_Address = wiringPiI2CSetup(MS5611_ADDRESS);

	for (i = 0; i < 6; i++)
	{
		C[i] = PROM_read(Device_Address, CMD_PROM_READ + i * 2);

		printf("C[%d] = %d\n", i + 1, C[i]);
	}

	//D1 = BARO_read(Device_Address, CONV_D1_4096);
	//D2 = BARO_read(Device_Address, CONV_D2_4096);

	//printf("D1 = %d\n", D1);
	//printf("D2 = %d", D2);
}