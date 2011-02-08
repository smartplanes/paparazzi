// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  I2c GPS class for ArduIMU.
//	Code by Steve Joyce, smartplanes.se
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
#ifndef AP_GPS_I2C_h
#define AP_GPS_I2C_h

#include <GPS.h>


class AP_GPS_I2C : public GPS
{
public:
    // Methods
	AP_GPS_I2C(Stream *s = NULL);  // there is no serial stream here but
	void		init(void);
	void		update();
	void		fill_buffer(uint8_t*);

private:
	// protocol essentials
#pragma pack(1)
	struct i2c_msg {
		uint32_t	speed_2d;
		int32_t	heading_2d;
		uint8_t	fix_type;
		uint8_t	fix_status;
	};
#pragma pack(pop)

	enum ubx_nav_fix_type {
		FIX_NONE = 0,
		FIX_DEAD_RECKONING = 1,
		FIX_2D = 2,
		FIX_3D = 3,
		FIX_GPS_DEAD_RECKONING = 4,
		FIX_TIME = 5
	};
	enum ubx_nav_status_bits {
		NAV_STATUS_FIX_VALID = 1
	};

	// Receive buffer
	union {
		i2c_msg	i2cmsg;
		uint8_t	bytes[];
	} _buffer;


	// Buffer parse & GPS state update
	void		_parse_gps();

      bool _dataReceived;
};

#endif
