F3_TARGETS  += $(TARGET)
FEATURES    = VCP

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/barometer_bmp085.c \
            drivers/barometer_ms5611.c \
            drivers/barometer_bmp280.c \
            drivers/compass_ak8975.c \
            drivers/compass_hmc5883l.c \
            drivers/compass_mag3110.c \
            drivers/serial_usb_vcp.c \
            drivers/sonar_hcsr04.c \
            drivers/sonar_srf10.c

