
IMU_IMU3000_CFLAGS  = -DUSE_IMU
IMU_IMU3000_CFLAGS += -DIMU_TYPE_H=\"modules/ins/ins_IMU3000.h\"

IMU_IMU3000_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                      $(SRC_MODULES)/ins/ins_IMU3000.c


IMU_IMU3000_CFLAGS += -DUSE_I2C
ifeq ($(ARCH), stm32)
	IMU_IMU3000_CFLAGS += -DUSE_I2C2
	IMU_IMU3000_CFLAGS += -DIMU3000_I2C_DEVICE=i2c2
else ifeq ($(ARCH), lpc21)
	IMU_IMU3000_CFLAGS += -DUSE_I2C0
	IMU_IMU3000_CFLAGS += -DIMU3000_I2C_DEVICE=i2c0
endif

ap.CFLAGS += $(IMU_IMU3000_CFLAGS)
ap.srcs   += $(IMU_IMU3000_SRCS)

ap.CFLAGS += -DUSE_GPS

