/*
 * JoyToBebop.h
 *
 *  Created on: May 4, 2015
 *      Author: romanripp
 */

#pragma once

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

namespace roman {
namespace bebop {

class JoyToBebop {
public:
  JoyToBebop();
  ~JoyToBebop();

  void OnJoystick(const sensor_msgs::Joy &msg) const;

private:
  void HandleAxes(std::vector<float> axes) const;
  void HandleButtons(const std::vector<int32_t> &buttons) const;
  void Exp(std::vector<float> &axes) const;
  bool IsPushed(const size_t index,
                  const std::vector<int32_t> &newButtons) const;
  bool IsPressed(const size_t index,
                  const std::vector<int32_t> &newButtons) const;

  bool IsZero(std::vector<float> &axes) const;

private:
  ros::NodeHandle m_handle;
  ros::Subscriber m_joystick;

  ros::Publisher m_controls;
  ros::Publisher m_camera;

  ros::Publisher m_emergency;
  ros::Publisher m_takeoff;
  ros::Publisher m_land;

  float m_slope;
  float m_deadZone;

  mutable bool m_tookOff;
  mutable geometry_msgs::Twist m_cameraTwist;
  mutable std::vector<int32_t> m_buttons;
  mutable std::vector<float> m_axis;
};
}
}
