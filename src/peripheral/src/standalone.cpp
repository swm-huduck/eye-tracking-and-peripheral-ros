// Copyright 2017-2019 Paul Nettle
//
// This file is part of Gobbledegook.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file in the root of the source tree.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// >>
// >>>  INSIDE THIS FILE
// >>
//
// This is an example single-file stand-alone application that runs a Gobbledegook server.
//
// >>
// >>>  DISCUSSION
// >>
//
// Very little is required ("MUST") by a stand-alone application to instantiate a valid Gobbledegook server. There are also some
// things that are reocommended ("SHOULD").
//
// * A stand-alone application MUST:
//
//     * Start the server via a call to `ggkStart()`.
//
//         Once started the server will run on its own thread.
//
//         Two of the parameters to `ggkStart()` are delegates responsible for providing data accessors for the server, a
//         `GGKServerDataGetter` delegate and a 'GGKServerDataSetter' delegate. The getter method simply receives a string name (for
//         example, "battery/level") and returns a void pointer to that data (for example: `(void *)&batteryLevel`). The setter does
//         the same only in reverse.
//
//         While the server is running, you will likely need to update the data being served. This is done by calling
//         `ggkNofifyUpdatedCharacteristic()` or `ggkNofifyUpdatedDescriptor()` with the full path to the characteristic or delegate
//         whose data has been updated. This will trigger your server's `onUpdatedValue()` method, which can perform whatever
//         actions are needed such as sending out a change notification (or in BlueZ parlance, a "PropertiesChanged" signal.)
//
// * A stand-alone application SHOULD:
//
//     * Shutdown the server before termination
//
//         Triggering the server to begin shutting down is done via a call to `ggkTriggerShutdown()`. This is a non-blocking method
//         that begins the asynchronous shutdown process.
//
//         Before your application terminates, it should wait for the server to be completely stopped. This is done via a call to
//         `ggkWait()`. If the server has not yet reached the `EStopped` state when `ggkWait()` is called, it will block until the
//         server has done so.
//
//         To avoid the blocking behavior of `ggkWait()`, ensure that the server has stopped before calling it. This can be done
//         by ensuring `ggkGetServerRunState() == EStopped`. Even if the server has stopped, it is recommended to call `ggkWait()`
//         to ensure the server has cleaned up all threads and other internals.
//
//         If you want to keep things simple, there is a method `ggkShutdownAndWait()` which will trigger the shutdown and then
//         block until the server has stopped.
//
//     * Implement signal handling to provide a clean shut-down
//
//         This is done by calling `ggkTriggerShutdown()` from any signal received that can terminate your application. For an
//         example of this, search for all occurrences of the string "signalHandler" in the code below.
//
//     * Register a custom logging mechanism with the server
//
//         This is done by calling each of the log registeration methods:
//
//             `ggkLogRegisterDebug()`
//             `ggkLogRegisterInfo()`
//             `ggkLogRegisterStatus()`
//             `ggkLogRegisterWarn()`
//             `ggkLogRegisterError()`
//             `ggkLogRegisterFatal()`
//             `ggkLogRegisterAlways()`
//             `ggkLogRegisterTrace()`
//
//         Each registration method manages a different log level. For a full description of these levels, see the header comment
//         in Logger.cpp.
//
//         The code below includes a simple logging mechanism that logs to stdout and filters logs based on a few command-line
//         options to specify the level of verbosity.
//
// >>
// >>>  Building with GOBBLEDEGOOK
// >>
//
// The Gobbledegook distribution includes this file as part of the Gobbledegook files with everything compiling to a single, stand-
// alone binary. It is built this way because Gobbledegook is not intended to be a generic library. You will need to make your
// custom modifications to it. Don't worry, a lot of work went into Gobbledegook to make it almost trivial to customize
// (see Server.cpp).
//
// If it is important to you or your build process that Gobbledegook exist as a library, you are welcome to do so. Just configure
// your build process to build the Gobbledegook files (minus this file) as a library and link against that instead. All that is
// required by applications linking to a Gobbledegook library is to include `include/Gobbledegook.h`.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <thread>
#include <sstream>
#include <queue>
#include "peripheral/Gobbledegook.h"
#include <ros/ros.h>

#include "peripheral/speed.h"
#include "peripheral/navigation_turn_event.h"

#include "peripheral/setting.h"

#include "peripheral/call.h"
#include "peripheral/sms.h"
#include "peripheral/kakao_talk.h"

ros::Publisher speed_publisher;
ros::Publisher navigation_turn_event_publisher;

ros::Publisher setting_publisher;

ros::Publisher call_publisher;
ros::Publisher sms_publisher;
ros::Publisher kakao_talk_publisher;

//
// Constants
//

// Maximum time to wait for any single async process to timeout during initialization
static const int kMaxAsyncInitTimeoutMS = 30 * 1000;

// The battery level ("battery/level") reported by the server (see Server.cpp)
static uint8_t serverDataBatteryLevel = 78;

// The text string ("text/string") used by our custom text string service (see Server.cpp)
static std::string serverDataTextString = "Hello, world!";

static int mtu = 20;

//
// Logging
//

enum LogLevel
{
	Debug,
	Verbose,
	Normal,
	ErrorsOnly
};

// Our log level - defaulted to 'Normal' but can be modified via command-line options
LogLevel logLevel = Normal;

// Our full set of logging methods (we just log to stdout)
//
// NOTE: Some methods will only log if the appropriate `logLevel` is set
void LogDebug(const char *pText)
{
	if (logLevel <= Debug)
	{
		std::cout << "  DEBUG: " << pText << std::endl;
	}
}
void LogInfo(const char *pText)
{
	if (logLevel <= Verbose)
	{
		std::cout << "   INFO: " << pText << std::endl;
	}
}
void LogStatus(const char *pText)
{
	if (logLevel <= Normal)
	{
		std::cout << " STATUS: " << pText << std::endl;
	}
}
void LogWarn(const char *pText) { std::cout << "WARNING: " << pText << std::endl; }
void LogError(const char *pText) { std::cout << "!!ERROR: " << pText << std::endl; }
void LogFatal(const char *pText) { std::cout << "**FATAL: " << pText << std::endl; }
void LogAlways(const char *pText) { std::cout << "..Log..: " << pText << std::endl; }
void LogTrace(const char *pText) { std::cout << "-Trace-: " << pText << std::endl; }

//
// Signal handling
//

// We setup a couple Unix signals to perform graceful shutdown in the case of SIGTERM or get an SIGING (CTRL-C)
void signalHandler(int signum)
{
	switch (signum)
	{
	case SIGINT:
		LogStatus("SIGINT recieved, shutting down");
		ggkTriggerShutdown();
		break;
	case SIGTERM:
		LogStatus("SIGTERM recieved, shutting down");
		ggkTriggerShutdown();
		break;
	}
}

//
// Server data management
//

// Called by the server when it wants to retrieve a named value
//
// This method conforms to `GGKServerDataGetter` and is passed to the server via our call to `ggkStart()`.
//
// The server calls this method from its own thread, so we must ensure our implementation is thread-safe. In our case, we're simply
// sending over stored values, so we don't need to take any additional steps to ensure thread-safety.
const void *dataGetter(const char *pName)
{
	if (nullptr == pName)
	{
		LogError("NULL name sent to server data getter");
		return nullptr;
	}

	std::string strName = pName;

	if (strName == "battery/level")
	{
		return &serverDataBatteryLevel;
	}
	else if (strName == "text/string")
	{
		return serverDataTextString.c_str();
	}
	else if (strName == "mtu") {
		return &mtu;
	}

	LogWarn((std::string("Unknown name for server data getter request: '") + pName + "'").c_str());
	return nullptr;
}

// Called by the server when it wants to update a named value
//
// This method conforms to `GGKServerDataSetter` and is passed to the server via our call to `ggkStart()`.
//
// The server calls this method from its own thread, so we must ensure our implementation is thread-safe. In our case, we're simply
// sending over stored values, so we don't need to take any additional steps to ensure thread-safety.
int dataSetter(const char *pName, const void *pData)
{
	if (nullptr == pName)
	{
		LogError("NULL name sent to server data setter");
		return 0;
	}
	if (nullptr == pData)
	{
		LogError("NULL pData sent to server data setter");
		return 0;
	}

	std::string strName = pName;

	if (strName == "battery/level")
	{
		serverDataBatteryLevel = *static_cast<const uint8_t *>(pData);
		LogDebug((std::string("Server data: battery level set to ") + std::to_string(serverDataBatteryLevel)).c_str());
		return 1;
	}
	else if (strName == "text/string")
	{
		serverDataTextString = static_cast<const char *>(pData);
		unsigned int len = serverDataTextString.size();
		switch (serverDataTextString[0])
		{
		// Speed
		case 's': {
			std::string speed = serverDataTextString.substr(1, len - 1);
			float speedValue = std::stof(speed);
			LogDebug((std::string("Server data: (Speed) value: ") + speed).c_str());

			while(ros::ok()) {
				peripheral::speed s;
				s.value = speedValue;
				speed_publisher.publish(s);
				ros::spinOnce();
				break;
			}
			
			break;
		}

		// Call
		case 'c': {
			std::string name = serverDataTextString.substr(1, len - 2);
			std::string state = serverDataTextString.substr(len - 1, len);
			std::string stateMsg = (state == "0") ? "전화 종료" : (state == "1") ? "전화 옴" : "전화 중";
			unsigned int callType =  (state == "0") ? 0 : (state == "1") ? 1 : 2;
			LogDebug((std::string("Server data: (Call) name: ") + name + ", state: " + stateMsg).c_str());

			while(ros::ok()) {
				peripheral::call c;
				c.name = name;
				c.call_type = callType;
				call_publisher.publish(c);
				ros::spinOnce();
				break;
			}

			break;
		}

		// Message, KakaoTalk
		case 'm': 
		case 'k': {
			std::string type = serverDataTextString[0] == 'm' ? "Message" : "KakaoTalk";
			int separatorIndex = serverDataTextString.find("{]");
			std::string name = serverDataTextString.substr(1, separatorIndex - 1);
			std::string content = serverDataTextString.substr(separatorIndex + 2, len - separatorIndex);
			LogDebug((std::string("Server data: (" + type + ") name: ") + name + ", content: " + content).c_str());

			while(ros::ok()) {
				if(type == "Message") {
					peripheral::sms m;
					m.name = name;
					m.content = content;
					sms_publisher.publish(m);
					ros::spinOnce();
				}
				else if(type == "KakaoTalk") {
					peripheral::kakao_talk k;
					k.name = name;
					k.content = content;
					kakao_talk_publisher.publish(k);
					ros::spinOnce();
				}
				break;
			}

			break;
		}

		// TrunEvent
		case 'n': {
			std::string s = serverDataTextString.substr(1, len);
			std::string delimiter = "{]";

			size_t pos = 0;
			std::string token;
			int idx = 0;

			peripheral::navigation_turn_event event;
			
			while ((pos = s.find(delimiter)) != std::string::npos) {
				token = s.substr(0, pos);
				
				if(idx == 0) {
					event.next_turn_type = std::stoi(token);
				}
				else if(idx == 1) {
					event.next_left_distance = std::stof(token);
				}
				else if(idx == 2) {
					event.next_relational_position_x = std::stof(token);
				}
				else if(idx == 3) {
					event.next_relational_position_y = std::stof(token);
				}
				else if(idx == 4) {
					event.next_next_turn_type = std::stoi(token);
				}
				else if(idx == 5) {
					event.next_next_left_distance = std::stof(token);
				}
				std::cout << idx;
				idx++;
				s.erase(0, pos + delimiter.length());
			}

			while (ros::ok()) {
				navigation_turn_event_publisher.publish(event);
				ros::spinOnce();
				break;
			}
			break;
		}

		case 'p': {
			int separatorIndex = serverDataTextString.find("{]");
			std::string item = serverDataTextString.substr(1, separatorIndex - 1);
			std::string value = serverDataTextString.substr(separatorIndex + 2, len - separatorIndex);
			LogDebug((std::string("Server data: (Setting) item: ") + item + ", value: " + value).c_str());

			if(item == "mtu") {
				mtu = atoi(value.c_str());
				LogDebug(std::string("Server data: (MTU) value: " + value).c_str());
			}

			while(ros::ok()) {
				peripheral::setting s;
				s.item = item;
				s.value = value;
				setting_publisher.publish(s);
				ros::spinOnce();
				break;
			}

			break;
		}

		default:
			break;
		}

		// LogDebug((std::string("Server data: text string set to '") + serverDataTextString + "'").c_str());
		return 1;
	}

	LogWarn((std::string("Unknown name for server data setter request: '") + pName + "'").c_str());

	return 0;
}

//
// Entry point
//


int main(int argc, char **ppArgv)
{
	ros::init(argc, ppArgv, "peripheral");

	ros::NodeHandle nh;

	speed_publisher = nh.advertise<peripheral::speed>("speed", 1);
	navigation_turn_event_publisher = nh.advertise<peripheral::navigation_turn_event>("navigation_turn_event", 1);
	
	setting_publisher = nh.advertise<peripheral::setting>("setting", 1);
	
	call_publisher = nh.advertise<peripheral::call>("call", 1);
	sms_publisher = nh.advertise<peripheral::sms>("sms", 1);
	kakao_talk_publisher = nh.advertise<peripheral::kakao_talk>("kakao_talk", 1);

	ros::Rate rate(30);

	// A basic command-line parser
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = ppArgv[i];
		if (arg == "-q")
		{
			logLevel = ErrorsOnly;
		}
		else if (arg == "-v")
		{
			logLevel = Verbose;
		}
		else if (arg == "-d")
		{
			logLevel = Debug;
		}
		else
		{
			LogFatal((std::string("Unknown parameter: '") + arg + "'").c_str());
			LogFatal("");
			LogFatal("Usage: standalone [-q | -v | -d]");
			return -1;
		}
	}

	logLevel = Debug;

	// Setup our signal handlers
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Register our loggers
	ggkLogRegisterDebug(LogDebug);
	ggkLogRegisterInfo(LogInfo);
	ggkLogRegisterStatus(LogStatus);
	ggkLogRegisterWarn(LogWarn);
	ggkLogRegisterError(LogError);
	ggkLogRegisterFatal(LogFatal);
	ggkLogRegisterAlways(LogAlways);
	ggkLogRegisterTrace(LogTrace);

	// Start the server's ascync processing
	//
	// This starts the server on a thread and begins the initialization process
	//
	// !!!IMPORTANT!!!
	//
	//     This first parameter (the service name) must match tha name configured in the D-Bus permissions. See the Readme.md file
	//     for more information.
	//
	if (!ggkStart("huduck", "HUDuck-001", "HUDuck-001", dataGetter, dataSetter, kMaxAsyncInitTimeoutMS))
	{
		return -1;
	}

	// Wait for the server to start the shutdown process
	//
	// While we wait, every 15 ticks, drop the battery level by one percent until we reach 0
	while (ggkGetServerRunState() < EStopping)
	{
		std::this_thread::sleep_for(std::chrono::seconds(15));

		// serverDataBatteryLevel = std::max(serverDataBatteryLevel - 1, 0);
		// ggkNofifyUpdatedCharacteristic("/com/gobbledegook/battery/level");
	}

	// Wait for the server to come to a complete stop (CTRL-C from the command line)
	if (!ggkWait())
	{
		return -1;
	}

	// Return the final server health status as a success (0) or error (-1)
	return ggkGetServerHealth() == EOk ? 0 : 1;
}
