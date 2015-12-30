/*
 * constants.h
 *
 *  Created on: Dec 30, 2015
 *      Author: Roman
 */

#pragma once

namespace roman{
namespace bebop{

namespace joystick_params{
namespace axes {
	static const float DEAD_ZONE(0.1);
	static const float MAX(32767);
	static const float MIN(-32768);
}
}

namespace bebop_params {
	static const size_t MAX_THRUST(100);
	static const float MAX_ALTITIDE(3.0);
	static const float MAX_CAMERA_TILT_HORIZONTAL(80.0);
	static const float MAX_CAMERA_TILT_VERTICAL(50.0);
	static const float MIN_CAMERA_TILT_HORIZONTAL(-80.0);
	static const float MIN_CAMERA_TILT_VERTICAL(-50.0);
}

namespace topics {
	static const std::string RPYG("bebop/cmd_vel");
	static const std::string CAMERA("bebop/camera_control");
	static const std::string EMERGENCY("bebop/reset");
	static const std::string TAKEOFF("bebop/takeoff");
	static const std::string LAND("bebop/land");
}
}
}



