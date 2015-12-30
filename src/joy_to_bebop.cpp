/*
 * JoyToBebop.cpp
 *
 *  Created on: May 4, 2015
 *      Author: romanripp
 */

#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"

#include "ros/ros.h"
#include "joy_to_bebop/joy_to_bebop.h"
#include "joy_to_bebop/constants.h"

namespace roman {
namespace bebop {

JoyToBebop::JoyToBebop()
    : m_joystick(m_handle.subscribe("joy0", 50, &JoyToBebop::OnJoystick, this)),
      m_controls(
          m_handle.advertise<geometry_msgs::Twist>(topics::RPYG, 50)),
	  m_camera(
		  m_handle.advertise<geometry_msgs::Twist>(topics::CAMERA, 50)),
      m_emergency(
          m_handle.advertise<std_msgs::Empty>(topics::EMERGENCY, 50)),
      m_takeoff(
          m_handle.advertise<std_msgs::Empty>(topics::TAKEOFF, 50)),
	  m_land(
	      m_handle.advertise<std_msgs::Empty>(topics::LAND, 50)),
      m_slope(0.0), m_deadZone(0.0), m_buttons(20, 0), m_axis(4, 0.0),
      m_tookOff(false), m_cameraTwist() {
  const auto &axisValueRange =
      (joystick_params::axes::MAX + std::abs(joystick_params::axes::MIN));

  const auto &a = std::log10(bebop_params::MAX_THRUST);
  m_slope = 2 * a;
  m_slope /= axisValueRange;
  m_deadZone = (axisValueRange / 2) * joystick_params::axes::DEAD_ZONE;

  m_cameraTwist.angular.x = 0.0f;
  m_cameraTwist.angular.y = 0.0f;
  m_cameraTwist.angular.z = 0.0f;
  m_camera.publish(m_cameraTwist);

  ROS_INFO("JoyToBebop initialized");
}

JoyToBebop::~JoyToBebop() {}

void JoyToBebop::OnJoystick(const sensor_msgs::Joy &msg) const {
    HandleAxes(msg.axes);
    HandleButtons(msg.buttons);
}

void JoyToBebop::HandleAxes(std::vector<float> axes) const {
  if (axes.size() != 4) {
	  ROS_ERROR("Invalid axes values");
	  return;
  }

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
  if (buttons.size() != 20) {
    ROS_ERROR("Invalid buttons");
    return;
  }

  if (IsPushed(13, buttons)) {
    m_emergency.publish(std_msgs::Empty());
  }
  if (IsPushed(12, buttons)) {
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

  // Handle camera tilt
  if (IsPushed(8, buttons)) {
	  m_cameraTwist.angular.y = 0.0f;
	  m_cameraTwist.angular.z = 0.0f;
	  m_camera.publish(m_cameraTwist);
  }
  if (IsPressed(4, buttons)) {
	  //tilt camera up
	  if (m_cameraTwist.angular.y > bebop_params::MIN_CAMERA_TILT_VERTICAL) {
		  m_cameraTwist.angular.y--;
		  m_camera.publish(m_cameraTwist);
	  }
  }
  if (IsPressed(5, buttons)) {
	  //tilt camera right
	  if (m_cameraTwist.angular.z > bebop_params::MIN_CAMERA_TILT_HORIZONTAL) {
		  m_cameraTwist.angular.z--;
		  m_camera.publish(m_cameraTwist);
	  }
  }
  if (IsPressed(6, buttons)) {
	  //tilt camera down
	  if (m_cameraTwist.angular.y < bebop_params::MAX_CAMERA_TILT_VERTICAL) {
		  m_cameraTwist.angular.y++;
		  m_camera.publish(m_cameraTwist);
	  }
  }
  if (IsPressed(7, buttons)) {
	  //tilt camera left
	  if (m_cameraTwist.angular.z < bebop_params::MAX_CAMERA_TILT_HORIZONTAL) {
		  m_cameraTwist.angular.z++;
		  m_camera.publish(m_cameraTwist);
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

bool JoyToBebop::IsPushed(const size_t index,
                            const std::vector<int32_t> &newButtons) const {
	bool pushed(false);
	if (newButtons[index] > 0 && m_buttons[index] < 1) {
		pushed = true;
	}
	m_buttons[index] = newButtons[index];
	return pushed;
}

bool JoyToBebop::IsPressed(const size_t index,
                            const std::vector<int32_t> &newButtons) const {
	return (newButtons[index] > 0);
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
