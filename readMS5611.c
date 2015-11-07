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
#include <time.h>

#define MS5611_ADDRESS 0x77

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

#define OSR_256      1000 //us
#define OSR_512      2000 //us
#define OSR_1024     3000 //us
#define OSR_2048     5000 //us
#define OSR_4096     10000 //us

#define alpha 0.96
#define beta 0.96
#define gamma 0.96

// check daily sea level pressure at 
// http://www.kma.go.kr/weather/observation/currentweather.jsp
#define SEA_LEVEL_PRESSURE 1023.20 // Seoul 1023.20hPa

unsigned int PROM_read(int DA, char PROM_CMD)
{
	uint16_t ret = 0;
	uint8_t r8b[] = { 0, 0 };

	if (write(DA, &PROM_CMD, 1) != 1){
		printf("read set reg Failed to write to the i2c bus.\n");
	}

	if (read(DA, r8b, 2) != 2){
		printf("Failed to read from the i2c bus.\n");
	}

	ret = r8b[0] * 256 + r8b[1];

	return ret;
}

long CONV_read(int DA, char CONV_CMD)
{
	long ret = 0;
	uint8_t D[] = { 0, 0, 0 };

	int  h;
	char zero = 0x0;

	if (write(DA, &CONV_CMD, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(OSR_4096);

	if (write(DA, &zero, 1) != 1) {
		printf("write reset 8 bit Failed to write to the i2c bus.\n");
	}

	h = read(DA, &D, 3);

	if (h != 3) {
		printf("Failed to read from the i2c bus %d.\n", h);

	}

	ret = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];

	return ret;
}

void main()
{
	int i;

	int fd;

	uint16_t C[7];

	uint32_t D1;
	uint32_t D2;

	char RESET = 0x1E;

	int64_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P;

	double Temparature, fltd_Temparature;
	double Pressure, fltd_Pressure;

	float Altitude, pre_Altitude;
	int roc, fltd_roc;

	long curSampled_time = 0;
	long prevSampled_time = 0;
	float Sampling_time, prevSampling_time;
	struct timespec spec;

	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0){
		printf("Failed to open the bus.\n");
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, MS5611_ADDRESS) < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}

	if (write(fd, &RESET, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(10000);

	for (i = 0; i < 7; i++){
		usleep(1000);

		C[i] = PROM_read(fd, CMD_PROM_READ + (i * 2));
		//printf("C[%d] = %d\n", i, C[i]);
	}

	while (1){
		clock_gettime(CLOCK_MONOTONIC, &spec);
		curSampled_time = round(spec.tv_nsec / 1.0e6);

		prevSampling_time = Sampling_time;
		Sampling_time = (float)curSampled_time - (float)prevSampled_time;

		if (Sampling_time < 0) // to prevent negative sampling time
			Sampling_time = prevSampling_time;

		D1 = CONV_read(fd, CONV_D1_4096);
		D2 = CONV_read(fd, CONV_D2_4096);

		dT = D2 - (uint32_t)C[5] * pow(2, 8);
		TEMP = (2000 + (dT * (int64_t)C[5] / pow(2, 23)));

		OFF = (int64_t)C[2] * pow(2, 16) + (dT*C[4]) / pow(2, 7);
		SENS = (int32_t)C[1] * pow(2, 15) + dT*C[3] / pow(2, 8);

		/*
		SECOND ORDER TEMPARATURE COMPENSATION
		*/
		if (TEMP < 2000) // if temperature lower than 20 Celsius 
		{
			int32_t T1 = 0;
			int64_t OFF1 = 0;
			int64_t SENS1 = 0;

			T1 = pow((double)dT, 2) / 2147483648;
			OFF1 = 5 * pow(((double)TEMP - 2000), 2) / 2;
			SENS1 = 5 * pow(((double)TEMP - 2000), 2) / 4;

			if (TEMP < -1500) // if temperature lower than -15 Celsius 
			{
				OFF1 = OFF1 + 7 * pow(((double)TEMP + 1500), 2);
				SENS1 = SENS1 + 11 * pow(((double)TEMP + 1500), 2) / 2;
			}

			TEMP -= T1;
			OFF -= OFF1;
			SENS -= SENS1;
		}


		P = ((((int64_t)D1*SENS) / pow(2, 21) - OFF) / pow(2, 15));

		//Temparature = (double)TEMP / (double)100;
		Pressure = (double)P / (double)100;

		if (prevSampled_time == 0)
		{
			//fltd_Temparature = Temparature;
			fltd_Pressure = Pressure;
		}

		//fltd_Temparature = alpha * fltd_Temparature + (1 - alpha) * Temparature;
		fltd_Pressure = beta * fltd_Pressure + (1 - beta) * Pressure;

		//printf("Temparature : %.2f C", fltd_Temparature);
		//printf("  Pressure : %.2f mbar", fltd_Pressure);

		Altitude = 44330.0f * (1.0f - pow((double)fltd_Pressure / (double)SEA_LEVEL_PRESSURE, 0.1902949f));

		if (prevSampled_time == 0)
		{
			pre_Altitude = Altitude;
		}

		roc = (int)(100000 * (Altitude - pre_Altitude) / Sampling_time);

		if (prevSampled_time == 0)
		{
			fltd_roc = roc;
		}

		fltd_roc = gamma * fltd_roc + (1 - gamma) * roc;

		pre_Altitude = Altitude;

		printf("Altitude : %.2f   Rate of Climb : %d cm/s\n", Altitude, roc);

		prevSampled_time = curSampled_time;
	}
}