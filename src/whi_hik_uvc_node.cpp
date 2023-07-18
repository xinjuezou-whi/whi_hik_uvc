/******************************************************************
node to handle HIK UVC camera

Features:
- uvc resouces setup logic
- xxx

Dependency:
- sudo ln ~/catkin_workspace/src/whi_hik_uvc/lib/64bits/libHCUSBSDK.so /usr/lib/libHCUSBSDK.so

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-11: Initial version
2022-xx-xx: xxx
******************************************************************/
#include <iostream>
#include <signal.h>
#include <functional>

#include "whi_hik_uvc/whi_hik_uvc.h"

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI HIK UVC Camera VERSION 00.03" << std::endl;
	std::cout << "Copyright © 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    const std::string nodeName("whi_hik_uvc"); 
	ros::init(argc, argv, nodeName);
	auto nodeHandle = std::make_shared<ros::NodeHandle>();

	/// node logic
	auto hikUvc = std::make_unique<whi_hik_uvc::HikUvc>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		hikUvc = nullptr;

		// all the default sigint handler does is call shutdown()
		ros::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
#else
	ros::MultiThreadedSpinner spinner(0);
	spinner.spin();
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
