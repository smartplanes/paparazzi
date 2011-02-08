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

#include "AP_GPS_I2C.h"
#include "WProgram.h"

// Constructors ////////////////////////////////////////////////////////////////

AP_GPS_I2C::AP_GPS_I2C(Stream *s) : GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void AP_GPS_I2C::init(void)
{

 _dataReceived = 0;

}

// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
void AP_GPS_I2C::update(void)
{
  if(_dataReceived)
   _parse_gps();					 // Parse the new GPS packet
   _dataReceived = 0;
}

void AP_GPS_I2C::fill_buffer(byte *data)
{
 int i;

 for (i=0;i<sizeof(_buffer);i++)
   _buffer.bytes[i] = data[i];

 _dataReceived = 1;

}

// Private Methods /////////////////////////////////////////////////////////////

void
AP_GPS_I2C::_parse_gps(void)
{
	fix	= (_buffer.i2cmsg.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.i2cmsg.fix_type == FIX_3D);
	ground_speed = _buffer.i2cmsg.speed_2d;			// cm/s
	ground_course = _buffer.i2cmsg.heading_2d / 1000;	// Heading 2D deg * 100000 rescaled to deg * 100
	_setTime();
	valid_read = 1;
	new_data = 1;
}
