
C_SOURCES +=  \
Drivers/MPU6050/mpu6050.c\
Drivers/VL6180X/platform/cci-i2c/vl6180x_i2c.c\
Drivers/VL6180X/core/src/vl6180x_api.c

C_INCLUDES +=  \
-IDrivers/MPU6050\
-IDrivers/VL6180X/config/proximity\
-IDrivers/VL6180X/core/inc\
-IDrivers/VL6180X/platform/cci-i2c\
-IDrivers/VL6180X/platform/template