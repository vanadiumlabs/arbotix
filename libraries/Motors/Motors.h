/*
  Motors.h - Hardware Motor Library for Arduino, using Timer0
  Created by Michael Ferguson, using modified components of the 
  Seattle Robotics Society Workshop robot Level1 Samples:
    http://seattlerobotics.org/WorkshopRobot/

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

#ifndef Motors_h
#define Motors_h

#define RIGHT_DIR   4
#define LEFT_PWM    5
#define RIGHT_PWM   6
#define LEFT_DIR    7

/* Motors Class */
class Motors
{
  public:
    Motors() {};
    void init();
    void left(int pwm);
    void right(int pwm);
    void set(int lpwm, int rpwm);
  private:
    unsigned char lastx;
};

#endif
