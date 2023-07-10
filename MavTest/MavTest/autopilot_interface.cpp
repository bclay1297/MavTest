// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


unsigned int mMsgDataIndex = 0;
int mFullDataLen = 0;

/*

typedef struct timeval 
{
	long tv_sec;
	long tv_usec;
} timeval;
*/
int gettimeofday(struct timeval* tp, struct timezone* tzp)
{
	// Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
	// This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
	// until 00:00:00 January 1, 1970 
	static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

	SYSTEMTIME  system_time;
	FILETIME    file_time;
	uint64_t    time;

	GetSystemTime(&system_time);
	SystemTimeToFileTime(&system_time, &file_time);
	time = ((uint64_t)file_time.dwLowDateTime);
	time += ((uint64_t)file_time.dwHighDateTime) << 32;

	tp->tv_sec = (long)((time - EPOCH) / 10000000L);
	tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
	return 0;
}

//////////////////////////////////////////////////////////////////////////////
void BaseModeToString(unsigned char baseMode, std::vector<std::string>& modeList)
{
	modeList.clear();

	if (baseMode & MAV_MODE_FLAG_SAFETY_ARMED)
	{
		modeList.push_back("sarm");
	}
	else
	{
		modeList.push_back("_NS_");
	}
	if (baseMode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
	{
		modeList.push_back("iena");
	}
	else
	{
		modeList.push_back("_NS_");
	}
	if (baseMode & MAV_MODE_FLAG_HIL_ENABLED)
	{
		modeList.push_back("hena"); // HIL == hardware in the loop
	}
	else
	{
		modeList.push_back("_NS_");
	}
	if (baseMode & MAV_MODE_FLAG_STABILIZE_ENABLED)
	{
		modeList.push_back("sena");
	}
	else
	{
		modeList.push_back("_NS_");
	}

	if (baseMode & MAV_MODE_FLAG_GUIDED_ENABLED)
	{
		modeList.push_back("gena");
	}
	else
	{
		modeList.push_back("_NS_");
	}

	if (baseMode & MAV_MODE_FLAG_AUTO_ENABLED)
	{
		modeList.push_back("aena");
	}
	else
	{
		modeList.push_back("_NS_");
	}

	if (baseMode & MAV_MODE_FLAG_TEST_ENABLED)
	{
		modeList.push_back("tena");
	}
	else
	{
		modeList.push_back("_NS_");
	}

	if (baseMode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
	{
		modeList.push_back("cena");
	}
	else
	{
		modeList.push_back("_NS_");
	}
}
int recv_message(SOCKET mavSocket, int bufSize, char* dataBuf);
int send_message(SOCKET mavSocket, const mavlink_message_t& message);
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	fprintf(stderr, "POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	fprintf(stderr, "VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	fprintf(stderr, "POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(SOCKET mavSocket)
{
	mMavSocket = mavSocket;

	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

//	read_tid  = 0; // read thread id
//	write_tid = 0; // write thread id

	system_id    = 1; // system id
	autopilot_id = 1; // autopilot component id
	companion_id = 190; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

//	port = port_; // port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
//	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
	int bufSize;
	char dataBuf[1024];

	// Blocking wait for new data
//	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;

		bufSize = sizeof(dataBuf);

		int bytesRead = recv_message(mMavSocket, bufSize, dataBuf);

		mCurrMsgIndex = 0;

		if (bytesRead > 0)
		{
			int status = GetNextMessage(bytesRead, dataBuf, message);

			// ----------------------------------------------------------------------
			//   HANDLE MESSAGE
			// ----------------------------------------------------------------------
			while (status == 0)
			{
				DecodeMessage(message);

				status = GetNextMessage(bytesRead, dataBuf, message);
			}

			send_setpoint();
		}

 // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > 0 ) 
		{
			Sleep(10); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

int Autopilot_Interface::GetNextMessage(int dataLen , char *dataBuf, mavlink_message_t& message)
{
	int status = -1;
	bool messageFound = false;
	mavlink_status_t mavStat;
	unsigned int packetSize = 0;

	int channel = 0;

	while ((mCurrMsgIndex < dataLen) && (messageFound != true))
	{
		messageFound = mavlink_parse_char(channel, dataBuf[mCurrMsgIndex++],
			&message, &mavStat);

		if (mavStat.parse_state == MAVLINK_PARSE_STATE_GOT_STX)
		{
		}
		packetSize++;
	}

	if (messageFound == true)
	{
		status = 0;
	}

	return(status);
}
// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = send_message(mMavSocket, message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp;
	{
		sp = current_setpoint.data;
	}

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		fprintf(stderr, "%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		fprintf(stderr, "ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		fprintf(stderr, "\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == 1 )
	{
		fprintf(stderr, "DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		fprintf(stderr, "\n");

	} // end: if offboard_status

}

// ------------------------------------------------------------------------------
//   Arm
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
arm_disarm( bool flag )
{
	if(flag)
	{
		fprintf(stderr, "ARM ROTORS\n");
	}
	else
	{
		fprintf(stderr, "DISARM ROTORS\n");
	}

	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = (float) flag;
	com.param2           = 21196;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------


	fprintf(stderr, "START READ THREAD \n");

	// now we're reading messages
	fprintf(stderr, "\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	fprintf(stderr, "CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		Sleep(500); // check at 2Hz
	}

	fprintf(stderr, "Found\n");

	// now we know autopilot is sending messages
	fprintf(stderr, "\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		fprintf(stderr, "GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		fprintf(stderr, "GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		fprintf(stderr, "\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		Sleep(50);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	fprintf(stderr, "INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	fprintf(stderr, "INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	fprintf(stderr, "\n");


	// now we're streaming setpoint commands
	fprintf(stderr, "\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	fprintf(stderr, "CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;


	// now the read and write threads are closed
	fprintf(stderr, "\n");

	// still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::send_setpoint(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	{
		current_setpoint.data = sp;
	}

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe

/*
	while ( !time_to_exit )
	{
		Sleep(250);   // Stream at 4Hz
		write_setpoint();
	}
*/
	// signal end
	writing_status = false;

	return;

}


void
Autopilot_Interface::DecodeMessage(mavlink_message_t message)
{
	{

		// Store message sysid and compid.
		// Note this doesn't handle multiple message sources.
		current_messages.sysid = message.sysid;
		current_messages.compid = message.compid;

		// Handle Message ID
		switch (message.msgid)
		{

		case MAVLINK_MSG_ID_HEARTBEAT:
		{
			std::vector<std::string> modeList;

			fprintf(stderr, "MAVLINK_MSG_ID_HEARTBEAT\n");
			mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
			current_messages.time_stamps.heartbeat = get_time_usec();
			this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;

			BaseModeToString(current_messages.heartbeat.base_mode, modeList);

			std::vector<std::string> ::iterator modeIter;

			fprintf(stderr, "    HB: ");

			modeIter = modeList.begin();
			while (modeIter != modeList.end())
			{
				fprintf(stderr, " %s,", (*modeIter).c_str());

				modeIter++;
			}

			fprintf(stderr, "\n");
		}
		break;

		case MAVLINK_MSG_ID_SYSTEM_TIME:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SYSTEM_TIME\n");
		}
		break;

		case MAVLINK_MSG_ID_PARAM_VALUE:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_PARAM_VALUE\n");
		}
		break;

		case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
		}
		break;

		case MAVLINK_MSG_ID_SYS_STATUS:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SYS_STATUS\n");
			mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
			current_messages.time_stamps.sys_status = get_time_usec();
			this_timestamps.sys_status = current_messages.time_stamps.sys_status;
		}
		break;

		case MAVLINK_MSG_ID_BATTERY_STATUS:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_BATTERY_STATUS\n");
			mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
			current_messages.time_stamps.battery_status = get_time_usec();
			this_timestamps.battery_status = current_messages.time_stamps.battery_status;
		}
		break;

		case MAVLINK_MSG_ID_RADIO_STATUS:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_RADIO_STATUS\n");
			mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
			current_messages.time_stamps.radio_status = get_time_usec();
			this_timestamps.radio_status = current_messages.time_stamps.radio_status;
		}
		break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
			mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
			current_messages.time_stamps.local_position_ned = get_time_usec();
			this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
		}
		break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
			mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
			current_messages.time_stamps.global_position_int = get_time_usec();
			this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
		}
		break;

		case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
			mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
			current_messages.time_stamps.position_target_local_ned = get_time_usec();
			this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
		}
		break;

		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
			mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
			current_messages.time_stamps.position_target_global_int = get_time_usec();
			this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
		}
		break;

		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_HIGHRES_IMU\n");
			mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
			current_messages.time_stamps.highres_imu = get_time_usec();
			this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
		}
		break;

		case MAVLINK_MSG_ID_ATTITUDE:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_ATTITUDE\n");
			mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
			current_messages.time_stamps.attitude = get_time_usec();
			this_timestamps.attitude = current_messages.time_stamps.attitude;
		}
		break;

		case MAVLINK_MSG_ID_VFR_HUD:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_VFR_HUD\n");

			mavlink_vfr_hud_t vfrHud;
			mavlink_msg_vfr_hud_decode(&message, &vfrHud);
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ACK:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_ACK\n");
			mavlink_mission_ack_t missionAck;
			mavlink_msg_mission_ack_decode(&message, &missionAck);
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_RC_CHANNELS\n");
			mavlink_rc_channels_t rcChannels;
			mavlink_msg_rc_channels_decode(&message, &rcChannels);
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
			mavlink_rc_channels_raw_t rcChannelsRaw;

			mavlink_msg_rc_channels_raw_decode(&message, &rcChannelsRaw);
		}
		break;

		case MAVLINK_MSG_ID_POWER_STATUS:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_POWER_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_CURRENT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_CURRENT\n");
		}
		break;

		case MAVLINK_MSG_ID_RAW_IMU:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_RAW_IMU\n");
		}
		break;


		case MAVLINK_MSG_ID_SCALED_IMU:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_IMU\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_IMU2:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_IMU2\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_IMU3:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_IMU3\n");
		}
		break;

		case MAVLINK_MSG_ID_RAW_PRESSURE:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_RAW_PRESSURE\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_PRESSURE:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_PRESSURE\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_PRESSURE2:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_PRESSURE2\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_GPS_RAW_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_FENCE_STATUS:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_FENCE_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_TERRAIN_REQUEST:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_TERRAIN_REQUEST\n");
		}
		break;

		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT\n");
		}
		break;

		case MAVLINK_MSG_ID_TERRAIN_REPORT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_TERRAIN_REPORT\n");
		}
		break;


		case MAVLINK_MSG_ID_STATUSTEXT:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_STATUSTEXT\n");
		}
		break;

		case MAVLINK_MSG_ID_VIBRATION:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_VIBRATION\n");
		}
		break;

		case MAVLINK_MSG_ID_TIMESYNC:
		{
			fprintf(stderr, "MAVLINK_MSG_ID_TIMESYNC\n");
		}
		break;

		default:
		{
			fprintf(stderr, "Warning, did not handle message id %i\n",message.msgid);
		}
		break;


		} // end: switch msgid

	}
}


// End Autopilot_Interface


int recv_message(SOCKET mavSocket, int bufSize, char *dataBuf)
{
	uint8_t          cp = 0;
	mavlink_status_t status;
	uint8_t          msgReceived = false;
	mavlink_status_t lastStatus;

	int bufLen = sizeof(dataBuf);
	unsigned char channel = 0;
	struct sockaddr_in remAddr;
	int remAddrSize = sizeof(remAddr);
	mavlink_status_t mavStat;
	unsigned int dataLen = 0;


	dataLen = recvfrom(mavSocket, dataBuf, bufSize, 0, (SOCKADDR*)&remAddr, &remAddrSize);

	if (dataLen > 0)
	{
		mMsgDataIndex = 0;
		mFullDataLen = dataLen;

		lastStatus.packet_rx_drop_count = 0;
	}
	else
	{
		// Couldn't read from port

		fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d\n", dataLen, errno);
	}

	return(dataLen);
}

int send_message(SOCKET mavSocket, const mavlink_message_t& message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	unsigned short port = 14550;

	struct sockaddr_in mavAddr;
	mavAddr.sin_family = AF_INET;
	mavAddr.sin_port = htons(port);

	mavAddr.sin_addr.s_addr = inet_addr("10.1.1.1");

	int mavAddrSize = sizeof(mavAddr);

	int result = sendto(mavSocket, buf, len, 0, (SOCKADDR*)&mavAddr, mavAddrSize);

	// Write buffer to UDP port, locks port while writing
//	int bytesWritten = _write_port(buf, len);
	if (result < 0)
	{
		fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d\n", result, errno);
	}

	return result;
}
