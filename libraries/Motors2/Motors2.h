/*
  Motors2.h - Hardware Motor Library for Arduino/Sanguino, using Timer2
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

#if defined(__AVR_ATmega644P__)
  #define M2_RIGHT_DIR   18
  #define M2_LEFT_PWM    14
  #define M2_RIGHT_PWM   15
  #define M2_LEFT_DIR    19
#else
  #define M2_RIGHT_DIR   4
  #define M2_LEFT_PWM    5
  #define M2_RIGHT_PWM   6
  #define M2_LEFT_DIR    7
#endif

/* Motors Class */
class Motors2
{
  public:
	Motors2();
    void left(int pwm);
    void right(int pwm);
	void set(int lpwm, int rpwm);
    int getLeft();
    int getRight();

  private:
    int l_pwm;
    int r_pwm;
};

#endif
