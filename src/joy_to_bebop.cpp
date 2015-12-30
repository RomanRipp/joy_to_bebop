/*
 * JoyToBebop.cpp
 *
 *  Created on: May 4, 2015
 *      Author: romanripp
 */

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"

#include "ros/ros.h"
#include "joy_to_bebop/joy_to_bebop.h"

#include "Utility/Exceptions.h"
#include "Utility/SharedConstants.h"

namespace Roman {
namespace BebopDroneApi {

JoyToBebop::JoyToBebop()
    : m_joystick(m_handle.subscribe("joy0", 50, &JoyToBebop::OnJoystick, this)),
      m_controls(
          m_handle.advertise<geometry_msgs::Twist>(Topics::Controls::RPYG, 50)),
      m_emergency(
          m_handle.advertise<std_msgs::Empty>(Topics::Controls::EMERGENCY, 50)),
      m_takeoff(
          m_handle.advertise<std_msgs::Empty>(Topics::Controls::TAKEOFF, 50)),
	  m_land(
	      m_handle.advertise<std_msgs::Empty>(Topics::Controls::LAND, 50)),
      m_slope(0.0), m_deadZone(0.0), m_buttons(20, 0), m_axis(4, 0.0),
      m_tookOff(false) {
  const auto &axisValueRange =
      (JoystickParams::Axes::MAX + std::abs(JoystickParams::Axes::MIN));

  const auto &a = std::log10(BebopParams::Controls::MAXTHRUST);
  m_slope = 2 * a;
  m_slope /= axisValueRange;
  m_deadZone = (axisValueRange / 2) * JoystickParams::Axes::DEADZONE;
  ROS_INFO("JoyToBebop initialized");
}

JoyToBebop::~JoyToBebop() {}

void JoyToBebop::OnJoystick(const sensor_msgs::Joy &msg) const {
  try {
    HandleAxes(msg.axes);
    HandleButtons(msg.buttons);
  } catch (Exceptions::Base &e) {
    ROS_FATAL("Joystick problem: %s", e.what().c_str());
  }
}

void JoyToBebop::HandleAxes(std::vector<float> axes) const {
  if (axes.size() != 4)
    throw Exceptions::Critical("Invalid axes values");

  if (IsZero(axes)) {
      // Publish zeros once;
      geometry_msgs::Twist rpyg;
      rpyg.linear.x = 0.0f;
      rpyg.linear.y = 0.0f;
      rpyg.linear.z = 0.0f;
      rpyg.angular.z = 0.0f;

      m_controls.publish(rpyg);
    //}
  } else {
    Exp(axes);

    geometry_msgs::Twist rpyg;
    rpyg.linear.x = (-1) * axes.at(3);
    rpyg.linear.y = (-1) * axes.at(2);
    rpyg.linear.z = (-1) * axes.at(1);
    rpyg.angular.z = axes.at(0);

    m_controls.publish(rpyg);
  }
}

void JoyToBebop::HandleButtons(const std::vector<int32_t> &buttons) const {
  if (buttons.size() != 20)
    throw Exceptions::Critical("Invalid buttons");

  if (IsPressed(13, buttons)) {
    m_emergency.publish(std_msgs::Empty());
  }
  if (IsPressed(12, buttons)) {
	  if (!m_tookOff)
	  {
		  m_takeoff.publish(std_msgs::Empty());
		  m_tookOff = true;
	  }else
	  {
		  m_land.publish(std_msgs::Empty());
		  m_tookOff = false;
	  }
  }
}

void JoyToBebop::Exp(std::vector<float> &axes) const {
  for (float &axis : axes) {
    if (axis > 0)
      axis = 0.01 * (std::pow(10, (m_slope * std::abs(axis))) - 1.0);
    else
      axis = 0.01 * (1.0 - std::pow(10, (m_slope * std::abs(axis))));
  }
}

bool JoyToBebop::IsPressed(const size_t index,
                            const std::vector<int32_t> &newButtons) const {
  bool pressed(false);
  if (newButtons[index] > 0 && m_buttons[index] < 1) {
    pressed = true;
  }
  m_buttons[index] = newButtons[index];
  return pressed;
}

bool JoyToBebop::IsZero(std::vector<float> &newAxis) const {
  bool isZero(true);
  for (size_t i = 0; i < newAxis.size(); i++) {
    if (std::abs(newAxis[i]) > m_deadZone)
      isZero = false;
    else
      newAxis[i] = 0.0;
  }

  return isZero;
}
}
}
