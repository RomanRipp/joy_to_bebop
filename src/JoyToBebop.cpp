/*
 * JoyToBebop.cpp
 *
 *  Created on: May 4, 2015
 *      Author: romanripp
 */

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "JoyToBebop.h"

#include "Utility/Exceptions.h"
#include "Utility/SharedConstants.h"

namespace Roman{
namespace BebopDroneApi{

	JoyToBebop::JoyToBebop() : _joystick(_handle.subscribe("joy0", 50, &JoyToBebop::OnJoystick, this))
		, _controls	(_handle.advertise<nav_msgs::Odometry>(Topics::Controls::RPYG, 50))
		, _flag		(_handle.advertise<std_msgs::Bool>(Topics::Controls::RPSWITCH, 50))
		, _emergency(_handle.advertise<std_msgs::Bool>(Topics::Controls::EMERGENCY, 50))
		, _takeoff	(_handle.advertise<std_msgs::Bool>(Topics::Controls::TAKEOFF, 50))
		, _slope(0.0)
		, _deadZone(0.0)
		, _buttons(20, 0)
		, _axis(4, 0.0)
		, _sendZero(true)
	{
		const auto& axisValueRange = (JoystickParams::Axes::MAX + std::abs(JoystickParams::Axes::MIN));
		const auto& a = std::log10(BebopParams::Controls::MAXTHRUST);
		_slope = 2 * a;
		_slope /= axisValueRange;
		_deadZone = (axisValueRange / 2) * JoystickParams::Axes::DEADZONE;
		ROS_INFO("JoyToBebop initialized");
	}

	JoyToBebop::~JoyToBebop()
	{

	}

	void JoyToBebop::OnJoystick(const sensor_msgs::Joy& msg) const
	{
		try
		{
			_HandleAxes(msg.axes);
			_HandleButtons(msg.buttons);
		}
		catch(Exceptions::Base& e)
		{
			ROS_FATAL("Joystick problem: %s", e.what().c_str());
		}
	}

	void JoyToBebop::_HandleAxes(std::vector<float> axes) const
	{
		if (axes.size() != 4)
			throw Exceptions::Critical("Invalid axes values");

		if (_IsZero(axes))
		{
			if (_sendZero)
			{
				//Publish zeros once;
				nav_msgs::Odometry rpyg;
				rpyg.twist.twist.linear.x = 0.0;
				rpyg.twist.twist.linear.y = 0.0;
				rpyg.twist.twist.linear.z = 0.0;
				rpyg.pose.pose.position.z = 0.0;

				_controls.publish(rpyg);
				_sendZero = false;
			}
		}
		else
		{
			_Exp(axes);

			nav_msgs::Odometry rpyg;
			rpyg.twist.twist.linear.x = axes.at(2);
			rpyg.twist.twist.linear.y = (-1) * axes.at(3);
			rpyg.twist.twist.linear.z = (-1) * axes.at(0);
			rpyg.pose.pose.position.z = (-1) * axes.at(1);

			_controls.publish(rpyg);
			_sendZero = true;
		}
	}

	void JoyToBebop::_HandleButtons(const std::vector<int32_t>& buttons) const
	{
		if (buttons.size() != 20)
			throw Exceptions::Critical("Invalid buttons");

		if (_IsPressed(13, buttons))
		{
			std_msgs::Bool msg;
			msg.data = 1;
			_emergency.publish(msg);
		}
		if (_IsPressed(12, buttons))
		{
			std_msgs::Bool msg;
			msg.data = 1;
			_takeoff.publish(msg);
		}
	}

	void JoyToBebop::_Exp(std::vector<float>& axes) const
	{
		for(float& axis : axes)
		{
			if (axis > 0)
				axis = std::pow(10, (_slope * std::abs(axis))) - 1.0;
			else
				axis = 1.0 - std::pow(10, (_slope * std::abs(axis)));
		}
	}

	bool JoyToBebop::_IsPressed(const size_t index, const std::vector<int32_t>& newButtons) const
	{
		bool pressed(false);
		if (newButtons[index] > 0 && _buttons[index] < 1)
		{
			pressed = true;
		}
		_buttons[index] = newButtons[index];
		return pressed;
	}

	bool JoyToBebop::_IsZero(std::vector<float>& newAxis) const
	{
		bool isZero(true);
		for (size_t i = 0; i < newAxis.size(); i++)
		{
			if (std::abs(newAxis[i]) > _deadZone)
				isZero = false;
			else
				newAxis[i] = 0.0;
		}

		return isZero;
	}
}
}


