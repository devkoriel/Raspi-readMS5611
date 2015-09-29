#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <fcntl.h>
#include <math.h>

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
	uint16_t ret = 0;
	uint8_t r8b[] = { 0, 0 };

	if (write(DA, PROM_CMD, 1) != 1){
		printf("read set reg Failed to write to the i2c bus.\n");
	}

	if (read(DA, r8b, 2) != 2){
		printf("Failed to read from the i2c bus.\n");
	}

	ret = (uint16_t(r8b[0]) << 8) | uint16_t(r8b[1]);
	return ret;
}

void main()
{
	unsigned int i;

	int fd;

	unsigned int C[7];

	unsigned long D1;
	unsigned long D2;

	float P;
	float T;
	float dT;
	float OFF;
	float SENS;
	float H_alt;
	float H_temp;
	float Altitude;
	char buf0[26] = { 0, };

	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0){
		printf("Failed to open the bus.\n");
			return -1;
	}

	if (ioctl(fd, I2C_SLAVE, MS5611_ADDRESS) < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
			return -1;
	}

	if (write(fd, RESET, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(10000);

	for (i = 0; i < 7; i++){
		usleep(1000);

		C[i] = PROM_read(fd, CMD_PROM_READ + (i * 2));
		printf("C[%d] = %d\n", i+1, C[i]);
	}

}