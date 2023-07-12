// MavTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <conio.h>
#include <winsock2.h>
#include <WS2tcpip.h>
#include <sys/types.h>
//#include <sys/socket.h>

#include <iostream>


#include <common/mavlink.h>
#include <development/development.h>

#include "Autopilot_Interface.h"

WSADATA wsaData;
SOCKET mavSocket = INVALID_SOCKET;

void commands(Autopilot_Interface& api, bool autotakeoff);

int main()
{
    int res = WSAStartup(MAKEWORD(2, 2), &wsaData);

    mavSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (mavSocket == INVALID_SOCKET)
    {
        fprintf(stderr, "Could not create socket: %d\n", WSAGetLastError());
    }
    else
    {
        struct sockaddr_in mavAddr;
        short port = 14550;

        mavAddr.sin_family = AF_INET;
        mavAddr.sin_port = htons(port);

        mavAddr.sin_addr.s_addr = INADDR_ANY;
		memset(&(mavAddr.sin_zero), 0, 8);

        int status = bind(mavSocket, (SOCKADDR*)&mavAddr, sizeof(mavAddr));

		struct timeval readTimeout;
/*
		readTimeout.tv_sec = 0;
		readTimeout.tv_usec = 10;
		setsockopt(mavSocket, SOL_SOCKET, SO_RCVTIMEO, (char *) & readTimeout, sizeof(readTimeout));
*/
		unsigned long mode = 1;  // non-blocking

		status = ioctlsocket(mavSocket, FIONBIO, &mode);

        int bytesReceieved;
        char dataBuf[1024];
        int bufLen = sizeof(dataBuf);

        struct sockaddr_in remAddr;
        int remAddrSize = sizeof(remAddr);

  //      int receievedCnt = recvfrom(mavSocket, dataBuf, bufLen, 0, (SOCKADDR*)&remAddr, &remAddrSize);

        Autopilot_Interface autopilot_interface(mavSocket);

		bool autotakeoff = true;

 //       commands(autopilot_interface, autotakeoff);

        status = status;

		bool done = false;
		int msgCnt = 0;
		bool sendHeartbeat = false;

		while (done != true)
		{
			if (_kbhit() == 1)
			{
				char cmd = _getch();

				switch (cmd)
				{
					case 'a':
						autopilot_interface.arm_disarm(1);
					break;

					case 'd':
						autopilot_interface.arm_disarm(0);
					break;

					case 'H':
						sendHeartbeat = true;
					break;

					case 'h':
						sendHeartbeat = false;
						break;

					case 'O':
						autopilot_interface.enable_offboard_control();
					break;

					case 'o':
						autopilot_interface.disable_offboard_control();
					break;

					case 'r':
						autopilot_interface.SendRequestMessage();
						autopilot_interface.SendSetMessageInterval();

					break;

					case 's':
						autopilot_interface.SendSetMode();
					break;

					case 'q':
						exit(0);
					break;

					default:
					break;
				}
			}
			autopilot_interface.read_messages();

			if (sendHeartbeat == true)
			{
				msgCnt++;

				if (msgCnt == 100)
				{
					msgCnt = 0;

					autopilot_interface.SendHeartbeat();
				}
			}
		}
    }

}

void
commands(Autopilot_Interface& api, bool autotakeoff)
{

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	Sleep(1); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands

	if (autotakeoff)
	{
		// arm autopilot
		api.arm_disarm(true);
		Sleep(1); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command




	// Example 1 - Fly up by to 2m
	set_position(ip.x,       // [m]
		ip.y,       // [m]
		ip.z - 2.0, // [m]
		sp);

	if (autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i = 0; i < 8; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		Sleep(1);
	}


	// Example 2 - Set Velocity
	set_velocity(-1.0, // [m/s]
		-1.0, // [m/s]
		0.0, // [m/s]
		sp);

	// Example 2.1 - Append Yaw Command
	set_yaw(ip.yaw + 90.0 / 180.0 * M_PI, // [rad]
		sp);

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 4 seconds, check position
	for (int i = 0; i < 4; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
		Sleep(1);
	}

	if (autotakeoff)
	{
		// Example 3 - Land using fixed velocity
		set_velocity(0.0, // [m/s]
			0.0, // [m/s]
			1.0, // [m/s]
			sp);

		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;

		// SEND THE COMMAND
		api.update_setpoint(sp);
		// NOW pixhawk will try to move

		// Wait for 8 seconds, check position
		for (int i = 0; i < 8; i++)
		{
			mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
			printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
			Sleep(1);
		}

		printf("\n");

		// disarm autopilot
		api.arm_disarm(false);
		Sleep(1); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands


	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z);

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
	printf("    ap time:     %lu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
	printf("    baro:        %f (mBar) \n", imu.abs_pressure);
	printf("    altitude:    %f (m) \n", imu.pressure_alt);
	printf("    temperature: %f C \n", imu.temperature);

	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}
