/*
  Tpa81.h - Arduino Library for TPA-81 ThermoPile Sensor
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

#ifndef TPA81_h
#define TPA81_h

/* Base address = 0xD0. 0xD0-0xDE valid. addr = base + (2 * device) */
#define TPA_ADDR        0x68 

#define TPA_VER_REG     0x00
#define TPA_AMBIENT     0x01
// pixels are reg 2-9

class Tpa81
{
  public:
    Tpa81(unsigned char device);
    /* = a reading of ambient temp, and loads pixel values into buffer */
    int getData(unsigned char * pixels);
  private:
    unsigned char _device;
};

#endif
