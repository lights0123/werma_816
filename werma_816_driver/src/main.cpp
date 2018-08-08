/**
Software License Agreement (BSD)
\file      main.cpp
\authors   Ben Schattinger <developer@lights0123.com>
\copyright Copyright (c) 2018, Ben Schattinger, All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Ben Schattinger nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iomanip>
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "werma_816_msgs/Color.h"

ros::Subscriber color_sub;

class LED {
public:
	LED() : serial_() {};
	bool connected = false;

	void connect(std::string port, int32_t baud) {
		serial::Timeout to(serial::Timeout::simpleTimeout(500));
		serial_.setTimeout(to);
		serial_.setPort(port);
		serial_.setBaudrate(baud);

		for (int tries = 0; tries < 5; tries++) {
			try {
				serial_.open();
			} catch (serial::IOException) {
			}

			if (serial_.isOpen()) {
				connected = true;
				return;
			} else {
				connected = false;
				ROS_WARN("Bad Connection with serial port Error %s", port);
			}
		}

		ROS_WARN("LED not responding.");
	}

	void setColor(uint8_t r, uint8_t g, uint8_t b) {
		if (!connected) return;

		r = map(r, 0, 255, 1, 100);
		g = map(g, 0, 255, 1, 100);
		b = map(b, 0, 255, 1, 100);
		std::stringstream out;
		out << "WR 00 " << std::setfill('0') << std::setw(2) << std::hex << (int) r << ' ' << std::setw(2) << (int) g
		    << ' ' << std::setw(2) << (int) b << '\n';
		serial_.write(out.str());
		serial_.flush();
	}

private:
	serial::Serial serial_;

	// From Arduino's map function
	long map(long x, long in_min, long in_max, long out_min, long out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
};

LED led;

void onColor(const werma_816_msgs::Color &color) {
	led.setColor(color.r, color.g, color.b);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "werma_816_driver");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	std::string port = "/dev/ttyACM0";
	// Technically shouldn't matter, because the IC used in the light has native USB CDC support
	int32_t baud = 9600;
	pn.param<std::string>("port", port, port);
	pn.param<int32_t>("baud", baud, baud);
	ros::Subscriber sub = n.subscribe("/led/color", 10, onColor);
	// Attempt to connect and run.
	while (ros::ok()) {
		ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
		led.connect(port, baud);
		if (led.connected) {
			ROS_DEBUG("Connected!");
			led.setColor(0, 255, 120);
			ros::spin();
		} else {
			ROS_DEBUG("Problem connecting to serial device.");
			ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
			sleep(1);
		}
	}
}