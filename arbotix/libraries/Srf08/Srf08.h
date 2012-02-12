/*
  Srf08.h - Arduino Library for Devantech SRF-08 Sonar
  Copyright (c) 2009 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SRF08_h
#define SRF08_h

/* Base address = 0xE0. 0xE0-0xFF valid. addr = base + (2 * device_no) */
#define SRF_ADDR        0x70

#define SRF_COMMAND     0x00        // command register
#define SRF_LIGHT       0x01        // light sensor register
#define SRF_ECHO_H      0x02        // 1st echo high byte
#define SRF_ECHO_L      0x03        // 1st echo low byte
#define SRF_2ND_H       0x04        // 2nd echo high byte
#define SRF_2ND_L       0x05        // 2nd echo low byte

#define SRF_CMD_CM      0x51        // Ranging mode - result in centimeters

class Srf08
{
  public:
	Srf08(unsigned char device);
    /* = a reading of distance, in CM */    
    int getData();
    int getVersion();
    void ping();
  private:
	unsigned char _device;
    unsigned long _lastping;
};

#endif

#endif
