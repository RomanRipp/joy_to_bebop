/*
 * JoyToBebop.h
 *
 *  Created on: May 4, 2015
 *      Author: romanripp
 */

#pragma once

#include "sensor_msgs/Joy.h"

namespace Roman{
namespace BebopDroneApi{

class JoyToBebop
{
public:
	JoyToBebop();
	~JoyToBebop();

	void OnJoystick(const sensor_msgs::Joy& msg) const;

private:
	void _HandleAxes(std::vector<float> axes) const;
	void _HandleButtons(const std::vector<int32_t>& buttons) const;
	void _Exp(std::vector<float>& axes) const;
	bool _IsPressed(const size_t index, const std::vector<int32_t>& newButtons) const;
	bool _IsZero(std::vector<float>& axes) const;
private:
	ros::NodeHandle _handle;
	ros::Subscriber _joystick;

	ros::Publisher _controls;
	ros::Publisher _flag; // Boolean flag to activate roll/pitch movement

	ros::Publisher _emergency;
	ros::Publisher _takeoff;

	float _slope;
	float _deadZone;

	mutable std::vector<int32_t> _buttons;
	mutable std::vector<float> _axis;
};

}
}
