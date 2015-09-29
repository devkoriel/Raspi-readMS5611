# Raspi-readMS5611
This project writes/reads MS5611 TEMP/P sensor.

compile with "sudo gcc -o readMS5611 readMS5611.c -lm" (-lm is option to use math.h library in Linux OS)
<br>execute with "sudo ./readMS5611"

# Warnings
default dev-file is set as "/dev/i2c-1". You should change this to "/dev/i2c-0" if your configuration is different with mine.
You can figure out your configuration in "/dev" (check "/dev/i2c-'number'")

default dev-address is set as 0x77. You should change this if you have to.

please email me(peterjinsoo@gmail.com) if you have comments.

# License
This project is licensed under the MIT license (see the included license file for more information).

Note: The libraries used in this project might not be licensed under the MIT license. I've made sure that the usage of all libraries as a part of this project, my modifications and the redistribution of the modified and unmodified libraries are permitted by the respective licenses, but the permissions of the MIT license might not apply, especially in regard to relicensing.
