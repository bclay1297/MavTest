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
	mMsgSequence = 0;

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

//			send_setpoint();

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
	memset(message.ck, 0, sizeof(message.ck));
	memset(message.signature, 0, sizeof(message.signature));

	unsigned short chkSum;
//	crc_accumulate_buffer(&message.checksum, _MAV_PAYLOAD(&message), message.len);

	mavlink_status_t mavStatus;

	memset(&mavStatus, 0, sizeof(mavStatus));
	mavlink_finalize_message_buffer(&message, 255, 190, &mavStatus, message.len, message.len + 1, 0);

	memcpy(&message.ck, &message.checksum, sizeof(message.checksum));
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
	com.confirmation     = 0;
	com.param1           = (float) flag;
	com.param2           = 21196;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, autopilot_id, &message, &com);

	mavlink_command_long_t command_long;

	mavlink_msg_command_long_decode(&message, &command_long);

	// Send the message
	int len = write_message(message);

	fprintf(stderr, "Outbound message %d bytes\n", len);
	DecodeMessage(message);

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
	com.confirmation     = 0;
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

void Autopilot_Interface::SendHeartbeat()
{
	mavlink_heartbeat_t heartbeat;

	heartbeat.autopilot = 8;
	heartbeat.type = 6;
	heartbeat.mavlink_version = 3;
	heartbeat.system_status = 4;
	heartbeat.base_mode = 192;
	heartbeat.custom_mode = 0;

	mavlink_message_t message; 

	mavlink_msg_heartbeat_encode(255, 190, &message, &heartbeat);


	message.seq = mMsgSequence++;

	if (mMsgSequence >= 255)
	{
		mMsgSequence = 0;
	}

	int len = write_message(message);

	fprintf(stderr, "Outbound message %d bytes\n", len);
	DecodeMessage(message);
}

mavlink_set_mode_t set_mode;

void Autopilot_Interface::SendSetMode()
{
	mavlink_message_t message;
	mavlink_set_mode_t setMode;

	setMode.target_system = 1;
	setMode.base_mode = 217;
	setMode.custom_mode = 5;

	mavlink_msg_set_mode_encode(255, 190, &message, &setMode);

	int len = write_message(message);

	fprintf(stderr, "Outbound message %d bytes\n", len);
	DecodeMessage(message);
}

void Autopilot_Interface::SendRequestMessage()
{
	mavlink_message_t message;
	mavlink_command_long_t commandLong;


	commandLong.target_system = 1;
	commandLong.target_component = 1;
	commandLong.command = MAV_CMD_REQUEST_MESSAGE;
	commandLong.confirmation = 0;
	commandLong.param1 = 148.0f;
	commandLong.param2 = 0.0f;
	commandLong.param3 = 0.0f;
	commandLong.param4 = 0.0f;
	commandLong.param5 = 0.0f;
	commandLong.param6 = 0.0f;
	commandLong.param7 = 0.0f;

	mavlink_msg_command_long_encode(255, 190, &message, &commandLong);

	int len = write_message(message);

	fprintf(stderr, "Outbound message %d bytes\n", len);
	DecodeMessage(message);
}

void Autopilot_Interface::SendSetMessageInterval()
{
	mavlink_message_t message;
	mavlink_command_long_t commandLong;


	commandLong.target_system = 1;
	commandLong.target_component = 1;
	commandLong.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	commandLong.confirmation = 0;
	commandLong.param1 = 242.0f;
	commandLong.param2 = 1000000.0f;
	commandLong.param3 = 0.0f;
	commandLong.param4 = 0.0f;
	commandLong.param5 = 0.0f;
	commandLong.param6 = 0.0f;
	commandLong.param7 = 0.0f;

	mavlink_msg_command_long_encode(255, 190, &message, &commandLong);

	int len = write_message(message);

	fprintf(stderr, "Outbound message %d bytes\n", len);
	DecodeMessage(message);

}

void
Autopilot_Interface::DecodeMessage(mavlink_message_t message)
{
	{

		// Store message sysid and compid.
		// Note this doesn't handle multiple message sources.
		current_messages.sysid = message.sysid;
		current_messages.compid = message.compid;

		fprintf(stderr, "sid: %d, cid: %d, MSG: ", current_messages.sysid, current_messages.compid);
		// Handle Message ID
		switch (message.msgid)
		{

		case MAVLINK_MSG_ID_HEARTBEAT:
		{
			mavlink_heartbeat_t heartbeat;

			std::vector<std::string> modeList;

			fprintf(stderr, "MAVLINK_MSG_ID_HEARTBEAT\n");
			
			mavlink_msg_heartbeat_decode(&message, &heartbeat);

			fprintf(stderr, "    ap: %d\ttyp: %d\tver: %d\tsys: %d\tbmd: %d\tcud: %d\n",
				heartbeat.autopilot, heartbeat.type, heartbeat.mavlink_version,
				heartbeat.system_status, heartbeat.base_mode, heartbeat.custom_mode);

			BaseModeToString(heartbeat.base_mode, modeList);

			std::vector<std::string> ::iterator modeIter;

			fprintf(stderr, "    HB: ");

			modeIter = modeList.begin();
			while (modeIter != modeList.end())
			{
				fprintf(stderr, " %s,", (*modeIter).c_str());

				modeIter++;
			}

			fprintf(stderr, "\n");

	// send a heartbeat message every time we receive one

//			SendHeartbeat();
		}
		break;

		case MAVLINK_MSG_ID_SYS_STATUS:
		{
			mavlink_sys_status_t sys_status;

			mavlink_msg_sys_status_decode(&message, &sys_status);

			fprintf(stderr, "MAVLINK_MSG_ID_SYS_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_SYSTEM_TIME:
		{
			mavlink_system_time_t system_time;

			mavlink_msg_system_time_decode(&message, &system_time);

			fprintf(stderr, "MAVLINK_MSG_ID_SYSTEM_TIME\n");
		}
		break;

		case MAVLINK_MSG_ID_PING:
		{
			mavlink_ping_t ping;

			mavlink_msg_ping_decode(&message, &ping);

			fprintf(stderr, "MAVLINK_MSG_ID_PING\n");
		}
		break;

		case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
		{
			mavlink_change_operator_control_t change_operator_control;

			mavlink_msg_change_operator_control_decode(&message, &change_operator_control);

			fprintf(stderr, "MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
		{
			mavlink_change_operator_control_ack_t change_operator_control_ack;

			mavlink_msg_change_operator_control_ack_decode(&message, &change_operator_control_ack);

			fprintf(stderr, "MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK\n");
		}
		break;

		case MAVLINK_MSG_ID_AUTH_KEY:
		{
			mavlink_auth_key_t auth_key;

			mavlink_msg_auth_key_decode(&message, &auth_key);

			fprintf(stderr, "MAVLINK_MSG_ID_AUTH_KEY\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_MODE:
		{
			mavlink_set_mode_t set_mode;

			mavlink_msg_set_mode_decode(&message, &set_mode);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_MODE\n");

			fprintf(stderr, "    tsid: %d\tbmod: %d\tcmod: %d\n",
				set_mode.target_system, set_mode.base_mode, set_mode.custom_mode);
		}
		break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		{
			mavlink_param_request_read_t param_request_read;

			mavlink_msg_param_request_read_decode(&message, &param_request_read);

			fprintf(stderr, "MAVLINK_MSG_ID_PARAM_REQUEST_READ\n");
		}
		break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
			mavlink_param_request_list_t param_request_list;

			mavlink_msg_param_request_list_decode(&message, &param_request_list);

			fprintf(stderr, "MAVLINK_MSG_ID_PARAM_REQUEST_LIST\n");
		}
		break;

		case MAVLINK_MSG_ID_PARAM_VALUE:
		{
			mavlink_param_value_t param_value;

			mavlink_msg_param_value_decode(&message, &param_value);

			fprintf(stderr, "MAVLINK_MSG_ID_PARAM_VALUE\n");
			param_value.param_id[15] = '\0';

			fprintf(stderr, "    cnt: %d\tidx: %d\ttyp: %d\t val: %s\n",
					param_value.param_count, param_value.param_index,
					param_value.param_type, param_value.param_id);
		}
		break;

		case MAVLINK_MSG_ID_PARAM_SET:
		{
			mavlink_param_set_t param_set;

			mavlink_msg_param_set_decode(&message, &param_set);

			fprintf(stderr, "MAVLINK_MSG_ID_PARAM_SET\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
			mavlink_gps_raw_int_t gps_raw_int;

			mavlink_msg_gps_raw_int_decode(&message, &gps_raw_int);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS_RAW_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS_STATUS:
		{
			mavlink_gps_status_t gps_status;

			mavlink_msg_gps_status_decode(&message, &gps_status);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_IMU:
		{
			mavlink_scaled_imu_t scaled_imu;

			mavlink_msg_scaled_imu_decode(&message, &scaled_imu);

			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_IMU\n");
		}
		break;

		case MAVLINK_MSG_ID_RAW_IMU:
		{
			mavlink_raw_imu_t raw_imu;

			mavlink_msg_raw_imu_decode(&message, &raw_imu);

			fprintf(stderr, "MAVLINK_MSG_ID_RAW_IMU\n");
		}
		break;

		case MAVLINK_MSG_ID_RAW_PRESSURE:
		{
			mavlink_raw_pressure_t raw_pressure;

			mavlink_msg_raw_pressure_decode(&message, &raw_pressure);

			fprintf(stderr, "MAVLINK_MSG_ID_RAW_PRESSURE\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_PRESSURE:
		{
			mavlink_scaled_pressure_t scaled_pressure;

			mavlink_msg_scaled_pressure_decode(&message, &scaled_pressure);

			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_PRESSURE\n");
		}
		break;

		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_attitude_t attitude;

			mavlink_msg_attitude_decode(&message, &attitude);

			fprintf(stderr, "MAVLINK_MSG_ID_ATTITUDE\n");
		}
		break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
		{
			mavlink_attitude_quaternion_t attitude_quaternion;

			mavlink_msg_attitude_quaternion_decode(&message, &attitude_quaternion);

			fprintf(stderr, "MAVLINK_MSG_ID_ATTITUDE_QUATERNION\n");
		}
		break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
			mavlink_local_position_ned_t local_position_ned;

			mavlink_msg_local_position_ned_decode(&message, &local_position_ned);

			fprintf(stderr, "MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
		}
		break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
			mavlink_global_position_int_t global_position_int;

			mavlink_msg_global_position_int_decode(&message, &global_position_int);

			fprintf(stderr, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
		{
			mavlink_rc_channels_scaled_t rc_channels_scaled;

			mavlink_msg_rc_channels_scaled_decode(&message, &rc_channels_scaled);

			fprintf(stderr, "MAVLINK_MSG_ID_RC_CHANNELS_SCALED\n");
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
		{
			mavlink_rc_channels_raw_t rc_channels_raw;

			mavlink_msg_rc_channels_raw_decode(&message, &rc_channels_raw);

			fprintf(stderr, "MAVLINK_MSG_ID_RC_CHANNELS_RAW\n");
		}
		break;

		case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		{
			mavlink_servo_output_raw_t servo_output_raw;

			mavlink_msg_servo_output_raw_decode(&message, &servo_output_raw);

			fprintf(stderr, "MAVLINK_MSG_ID_SERVO_OUTPUT_RAW\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
		{
			mavlink_mission_request_partial_list_t mission_request_partial_list;

			mavlink_msg_mission_request_partial_list_decode(&message, &mission_request_partial_list);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
		{
			mavlink_mission_write_partial_list_t mission_write_partial_list;

			mavlink_msg_mission_write_partial_list_decode(&message, &mission_write_partial_list);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM:
		{
			mavlink_mission_item_t mission_item;

			mavlink_msg_mission_item_decode(&message, &mission_item);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_ITEM\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_REQUEST:
		{
			mavlink_mission_request_t mission_request;

			mavlink_msg_mission_request_decode(&message, &mission_request);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_REQUEST\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		{
			mavlink_mission_set_current_t mission_set_current;

			mavlink_msg_mission_set_current_decode(&message, &mission_set_current);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_SET_CURRENT\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_CURRENT:
		{
			mavlink_mission_current_t mission_current;

			mavlink_msg_mission_current_decode(&message, &mission_current);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_CURRENT\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		{
			mavlink_mission_request_list_t mission_request_list;

			mavlink_msg_mission_request_list_decode(&message, &mission_request_list);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_REQUEST_LIST\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_COUNT:
		{
			mavlink_mission_count_t mission_count;

			mavlink_msg_mission_count_decode(&message, &mission_count);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_COUNT\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		{
			mavlink_mission_clear_all_t mission_clear_all;

			mavlink_msg_mission_clear_all_decode(&message, &mission_clear_all);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_CLEAR_ALL\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
		{
			mavlink_mission_item_reached_t mission_item_reached;

			mavlink_msg_mission_item_reached_decode(&message, &mission_item_reached);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_ITEM_REACHED\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ACK:
		{
			mavlink_mission_ack_t mission_ack;

			mavlink_msg_mission_ack_decode(&message, &mission_ack);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_ACK\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
		{
			mavlink_set_gps_global_origin_t set_gps_global_origin;

			mavlink_msg_set_gps_global_origin_decode(&message, &set_gps_global_origin);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
		{
			mavlink_gps_global_origin_t gps_global_origin;

			mavlink_msg_gps_global_origin_decode(&message, &gps_global_origin);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN\n");
		}
		break;

		case MAVLINK_MSG_ID_PARAM_MAP_RC:
		{
			mavlink_param_map_rc_t param_map_rc;

			mavlink_msg_param_map_rc_decode(&message, &param_map_rc);

			fprintf(stderr, "MAVLINK_MSG_ID_PARAM_MAP_RC\n");
		}
		break;

		case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
		{
			mavlink_safety_set_allowed_area_t safety_set_allowed_area;

			mavlink_msg_safety_set_allowed_area_decode(&message, &safety_set_allowed_area);

			fprintf(stderr, "MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA\n");
		}
		break;

		case MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
		{
			mavlink_safety_allowed_area_t safety_allowed_area;

			mavlink_msg_safety_allowed_area_decode(&message, &safety_allowed_area);

			fprintf(stderr, "MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA\n");
		}
		break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV:
		{
			mavlink_attitude_quaternion_cov_t attitude_quaternion_cov;

			mavlink_msg_attitude_quaternion_cov_decode(&message, &attitude_quaternion_cov);

			fprintf(stderr, "MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV\n");
		}
		break;

		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
		{
			mavlink_nav_controller_output_t nav_controller_output;

			mavlink_msg_nav_controller_output_decode(&message, &nav_controller_output);

			fprintf(stderr, "MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT\n");
		}
		break;

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
		{
			mavlink_global_position_int_cov_t global_position_int_cov;

			mavlink_msg_global_position_int_cov_decode(&message, &global_position_int_cov);

			fprintf(stderr, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV\n");
		}
		break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:
		{
			mavlink_local_position_ned_cov_t local_position_ned_cov;

			mavlink_msg_local_position_ned_cov_decode(&message, &local_position_ned_cov);

			fprintf(stderr, "MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV\n");
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS:
		{
			mavlink_rc_channels_t rc_channels;

			mavlink_msg_rc_channels_decode(&message, &rc_channels);

			fprintf(stderr, "MAVLINK_MSG_ID_RC_CHANNELS\n");
		}
		break;

		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		{
			mavlink_request_data_stream_t request_data_stream;

			mavlink_msg_request_data_stream_decode(&message, &request_data_stream);

			fprintf(stderr, "MAVLINK_MSG_ID_REQUEST_DATA_STREAM\n");

//			request_data_stream.req_stream_id
		}
		break;

		case MAVLINK_MSG_ID_DATA_STREAM:
		{
			mavlink_data_stream_t data_stream;

			mavlink_msg_data_stream_decode(&message, &data_stream);

			fprintf(stderr, "MAVLINK_MSG_ID_DATA_STREAM\n");
		}
		break;

		case MAVLINK_MSG_ID_MANUAL_CONTROL:
		{
			mavlink_manual_control_t manual_control;

			mavlink_msg_manual_control_decode(&message, &manual_control);

			fprintf(stderr, "MAVLINK_MSG_ID_MANUAL_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		{
			mavlink_rc_channels_override_t rc_channels_override;

			mavlink_msg_rc_channels_override_decode(&message, &rc_channels_override);

			fprintf(stderr, "MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE\n");
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM_INT:
		{
			mavlink_mission_item_int_t mission_item_int;

			mavlink_msg_mission_item_int_decode(&message, &mission_item_int);

			fprintf(stderr, "MAVLINK_MSG_ID_MISSION_ITEM_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_VFR_HUD:
		{
			mavlink_vfr_hud_t vfr_hud;

			mavlink_msg_vfr_hud_decode(&message, &vfr_hud);

			fprintf(stderr, "MAVLINK_MSG_ID_VFR_HUD\n");
		}
		break;

		case MAVLINK_MSG_ID_COMMAND_INT:
		{
			mavlink_command_int_t command_int;

			mavlink_msg_command_int_decode(&message, &command_int);

			fprintf(stderr, "MAVLINK_MSG_ID_COMMAND_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
			mavlink_command_long_t command_long;

			mavlink_msg_command_long_decode(&message, &command_long);

			fprintf(stderr, "MAVLINK_MSG_ID_COMMAND_LONG\n");

			fprintf(stderr, "    tsid: %d\ttcmp: %d\tcmd: %d\tp1: %f\tp2: %f\tconf: %d\n",
				command_long.target_system, command_long.target_component, command_long.command,
				command_long.param1, command_long.param2, command_long.confirmation);
		}
		break;

		case MAVLINK_MSG_ID_COMMAND_ACK:
		{
			mavlink_command_ack_t command_ack;

			mavlink_msg_command_ack_decode(&message, &command_ack);

			fprintf(stderr, "MAVLINK_MSG_ID_COMMAND_ACK\n");

			fprintf(stderr, "    tsid: %d\ttcomp: %d\tcmd: %d\tprog: %d\tres: %d\tres2: %d\n",
				command_ack.target_system, command_ack.target_component,
				command_ack.command, command_ack.progress, command_ack.result,
				command_ack.result_param2);
		}
		break;

		case MAVLINK_MSG_ID_MANUAL_SETPOINT:
		{
			mavlink_manual_setpoint_t manual_setpoint;

			mavlink_msg_manual_setpoint_decode(&message, &manual_setpoint);

			fprintf(stderr, "MAVLINK_MSG_ID_MANUAL_SETPOINT\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		{
			mavlink_set_attitude_target_t set_attitude_target;

			mavlink_msg_set_attitude_target_decode(&message, &set_attitude_target);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_ATTITUDE_TARGET\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		{
			mavlink_set_position_target_local_ned_t set_position_target_local_ned;

			mavlink_msg_set_position_target_local_ned_decode(&message, &set_position_target_local_ned);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED\n");
		}
		break;

		case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		{
			mavlink_position_target_local_ned_t position_target_local_ned;

			mavlink_msg_position_target_local_ned_decode(&message, &position_target_local_ned);

			fprintf(stderr, "MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
		{
			mavlink_set_position_target_global_int_t set_position_target_global_int;

			mavlink_msg_set_position_target_global_int_decode(&message, &set_position_target_global_int);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		{
			mavlink_position_target_global_int_t position_target_global_int;

			mavlink_msg_position_target_global_int_decode(&message, &position_target_global_int);

			fprintf(stderr, "MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
		{
			mavlink_local_position_ned_system_global_offset_t local_position_ned_system_global_offset;

			mavlink_msg_local_position_ned_system_global_offset_decode(&message, &local_position_ned_system_global_offset);

			fprintf(stderr, "MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_STATE:
		{
			mavlink_hil_state_t hil_state;

			mavlink_msg_hil_state_decode(&message, &hil_state);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_STATE\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_CONTROLS:
		{
			mavlink_hil_controls_t hil_controls;

			mavlink_msg_hil_controls_decode(&message, &hil_controls);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_CONTROLS\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
		{
			mavlink_hil_rc_inputs_raw_t hil_rc_inputs_raw;

			mavlink_msg_hil_rc_inputs_raw_decode(&message, &hil_rc_inputs_raw);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW\n");
		}
		break;

		case MAVLINK_MSG_ID_OPTICAL_FLOW:
		{
			mavlink_optical_flow_t optical_flow;

			mavlink_msg_optical_flow_decode(&message, &optical_flow);

			fprintf(stderr, "MAVLINK_MSG_ID_OPTICAL_FLOW\n");
		}
		break;

		case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
		{
			mavlink_global_vision_position_estimate_t global_vision_position_estimate;

			mavlink_msg_global_vision_position_estimate_decode(&message, &global_vision_position_estimate);

			fprintf(stderr, "MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE\n");
		}
		break;

		case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		{
			mavlink_vision_position_estimate_t vision_position_estimate;

			mavlink_msg_vision_position_estimate_decode(&message, &vision_position_estimate);

			fprintf(stderr, "MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE\n");
		}
		break;

		case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
		{
			mavlink_vision_speed_estimate_t vision_speed_estimate;

			mavlink_msg_vision_speed_estimate_decode(&message, &vision_speed_estimate);

			fprintf(stderr, "MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE\n");
		}
		break;

		case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
		{
			mavlink_vicon_position_estimate_t vicon_position_estimate;

			mavlink_msg_vicon_position_estimate_decode(&message, &vicon_position_estimate);

			fprintf(stderr, "MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE\n");
		}
		break;

		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
			mavlink_highres_imu_t highres_imu;

			mavlink_msg_highres_imu_decode(&message, &highres_imu);

			fprintf(stderr, "MAVLINK_MSG_ID_HIGHRES_IMU\n");
		}
		break;

		case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		{
			mavlink_optical_flow_rad_t optical_flow_rad;

			mavlink_msg_optical_flow_rad_decode(&message, &optical_flow_rad);

			fprintf(stderr, "MAVLINK_MSG_ID_OPTICAL_FLOW_RAD\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_SENSOR:
		{
			mavlink_hil_sensor_t hil_sensor;

			mavlink_msg_hil_sensor_decode(&message, &hil_sensor);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_SENSOR\n");
		}
		break;

		case MAVLINK_MSG_ID_SIM_STATE:
		{
			mavlink_sim_state_t sim_state;

			mavlink_msg_sim_state_decode(&message, &sim_state);

			fprintf(stderr, "MAVLINK_MSG_ID_SIM_STATE\n");
		}
		break;

		case MAVLINK_MSG_ID_RADIO_STATUS:
		{
			mavlink_radio_status_t radio_status;

			mavlink_msg_radio_status_decode(&message, &radio_status);

			fprintf(stderr, "MAVLINK_MSG_ID_RADIO_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
		{
			mavlink_file_transfer_protocol_t file_transfer_protocol;

			mavlink_msg_file_transfer_protocol_decode(&message, &file_transfer_protocol);

			fprintf(stderr, "MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL\n");
		}
		break;

		case MAVLINK_MSG_ID_TIMESYNC:
		{
			mavlink_timesync_t timesync;

			mavlink_msg_timesync_decode(&message, &timesync);

			fprintf(stderr, "MAVLINK_MSG_ID_TIMESYNC\n");
		}
		break;

		case MAVLINK_MSG_ID_CAMERA_TRIGGER:
		{
			mavlink_camera_trigger_t camera_trigger;

			mavlink_msg_camera_trigger_decode(&message, &camera_trigger);

			fprintf(stderr, "MAVLINK_MSG_ID_CAMERA_TRIGGER\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_GPS:
		{
			mavlink_hil_gps_t hil_gps;

			mavlink_msg_hil_gps_decode(&message, &hil_gps);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_GPS\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		{
			mavlink_hil_optical_flow_t hil_optical_flow;

			mavlink_msg_hil_optical_flow_decode(&message, &hil_optical_flow);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_OPTICAL_FLOW\n");
		}
		break;

		case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		{
			mavlink_hil_state_quaternion_t hil_state_quaternion;

			mavlink_msg_hil_state_quaternion_decode(&message, &hil_state_quaternion);

			fprintf(stderr, "MAVLINK_MSG_ID_HIL_STATE_QUATERNION\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_IMU2:
		{
			mavlink_scaled_imu2_t scaled_imu2;

			mavlink_msg_scaled_imu2_decode(&message, &scaled_imu2);

			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_IMU2\n");
		}
		break;

		case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		{
			mavlink_log_request_list_t log_request_list;

			mavlink_msg_log_request_list_decode(&message, &log_request_list);

			fprintf(stderr, "MAVLINK_MSG_ID_LOG_REQUEST_LIST\n");
		}
		break;

		case MAVLINK_MSG_ID_LOG_ENTRY:
		{
			mavlink_log_entry_t log_entry;

			mavlink_msg_log_entry_decode(&message, &log_entry);

			fprintf(stderr, "MAVLINK_MSG_ID_LOG_ENTRY\n");
		}
		break;

		case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		{
			mavlink_log_request_data_t log_request_data;

			mavlink_msg_log_request_data_decode(&message, &log_request_data);

			fprintf(stderr, "MAVLINK_MSG_ID_LOG_REQUEST_DATA\n");
		}
		break;

		case MAVLINK_MSG_ID_LOG_DATA:
		{
			mavlink_log_data_t log_data;

			mavlink_msg_log_data_decode(&message, &log_data);

			fprintf(stderr, "MAVLINK_MSG_ID_LOG_DATA\n");
		}
		break;

		case MAVLINK_MSG_ID_LOG_ERASE:
		{
			mavlink_log_erase_t log_erase;

			mavlink_msg_log_erase_decode(&message, &log_erase);

			fprintf(stderr, "MAVLINK_MSG_ID_LOG_ERASE\n");
		}
		break;

		case MAVLINK_MSG_ID_LOG_REQUEST_END:
		{
			mavlink_log_request_end_t log_request_end;

			mavlink_msg_log_request_end_decode(&message, &log_request_end);

			fprintf(stderr, "MAVLINK_MSG_ID_LOG_REQUEST_END\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS_INJECT_DATA:
		{
			mavlink_gps_inject_data_t gps_inject_data;

			mavlink_msg_gps_inject_data_decode(&message, &gps_inject_data);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS_INJECT_DATA\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS2_RAW:
		{
			mavlink_gps2_raw_t gps2_raw;

			mavlink_msg_gps2_raw_decode(&message, &gps2_raw);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS2_RAW\n");
		}
		break;

		case MAVLINK_MSG_ID_POWER_STATUS:
		{
			mavlink_power_status_t power_status;

			mavlink_msg_power_status_decode(&message, &power_status);

			fprintf(stderr, "MAVLINK_MSG_ID_POWER_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_SERIAL_CONTROL:
		{
			mavlink_serial_control_t serial_control;

			mavlink_msg_serial_control_decode(&message, &serial_control);

			fprintf(stderr, "MAVLINK_MSG_ID_SERIAL_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS_RTK:
		{
			mavlink_gps_rtk_t gps_rtk;

			mavlink_msg_gps_rtk_decode(&message, &gps_rtk);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS_RTK\n");
		}
		break;

		case MAVLINK_MSG_ID_GPS2_RTK:
		{
			mavlink_gps2_rtk_t gps2_rtk;

			mavlink_msg_gps2_rtk_decode(&message, &gps2_rtk);

			fprintf(stderr, "MAVLINK_MSG_ID_GPS2_RTK\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_IMU3:
		{
			mavlink_scaled_imu3_t scaled_imu3;

			mavlink_msg_scaled_imu3_decode(&message, &scaled_imu3);

			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_IMU3\n");
		}
		break;

		case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
		{
			mavlink_data_transmission_handshake_t data_transmission_handshake;

			mavlink_msg_data_transmission_handshake_decode(&message, &data_transmission_handshake);

			fprintf(stderr, "MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE\n");
		}
		break;

		case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
		{
			mavlink_encapsulated_data_t encapsulated_data;

			mavlink_msg_encapsulated_data_decode(&message, &encapsulated_data);

			fprintf(stderr, "MAVLINK_MSG_ID_ENCAPSULATED_DATA\n");
		}
		break;

		case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		{
			mavlink_distance_sensor_t distance_sensor;

			mavlink_msg_distance_sensor_decode(&message, &distance_sensor);

			fprintf(stderr, "MAVLINK_MSG_ID_DISTANCE_SENSOR\n");
		}
		break;

		case MAVLINK_MSG_ID_TERRAIN_REQUEST:
		{
			mavlink_terrain_request_t terrain_request;

			mavlink_msg_terrain_request_decode(&message, &terrain_request);

			fprintf(stderr, "MAVLINK_MSG_ID_TERRAIN_REQUEST\n");
		}
		break;

		case MAVLINK_MSG_ID_TERRAIN_DATA:
		{
			mavlink_terrain_data_t terrain_data;

			mavlink_msg_terrain_data_decode(&message, &terrain_data);

			fprintf(stderr, "MAVLINK_MSG_ID_TERRAIN_DATA\n");
		}
		break;

		case MAVLINK_MSG_ID_TERRAIN_CHECK:
		{
			mavlink_terrain_check_t terrain_check;

			mavlink_msg_terrain_check_decode(&message, &terrain_check);

			fprintf(stderr, "MAVLINK_MSG_ID_TERRAIN_CHECK\n");
		}
		break;

		case MAVLINK_MSG_ID_TERRAIN_REPORT:
		{
			mavlink_terrain_report_t terrain_report;

			mavlink_msg_terrain_report_decode(&message, &terrain_report);

			fprintf(stderr, "MAVLINK_MSG_ID_TERRAIN_REPORT\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_PRESSURE2:
		{
			mavlink_scaled_pressure2_t scaled_pressure2;

			mavlink_msg_scaled_pressure2_decode(&message, &scaled_pressure2);

			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_PRESSURE2\n");
		}
		break;

		case MAVLINK_MSG_ID_ATT_POS_MOCAP:
		{
			mavlink_att_pos_mocap_t att_pos_mocap;

			mavlink_msg_att_pos_mocap_decode(&message, &att_pos_mocap);

			fprintf(stderr, "MAVLINK_MSG_ID_ATT_POS_MOCAP\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
		{
			mavlink_set_actuator_control_target_t set_actuator_control_target;

			mavlink_msg_set_actuator_control_target_decode(&message, &set_actuator_control_target);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET\n");
		}
		break;

		case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
		{
			mavlink_actuator_control_target_t actuator_control_target;

			mavlink_msg_actuator_control_target_decode(&message, &actuator_control_target);

			fprintf(stderr, "MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET\n");
		}
		break;

		case MAVLINK_MSG_ID_ALTITUDE:
		{
			mavlink_altitude_t altitude;

			mavlink_msg_altitude_decode(&message, &altitude);

			fprintf(stderr, "MAVLINK_MSG_ID_ALTITUDE\n");
		}
		break;

		case MAVLINK_MSG_ID_RESOURCE_REQUEST:
		{
			mavlink_resource_request_t resource_request;

			mavlink_msg_resource_request_decode(&message, &resource_request);

			fprintf(stderr, "MAVLINK_MSG_ID_RESOURCE_REQUEST\n");
		}
		break;

		case MAVLINK_MSG_ID_SCALED_PRESSURE3:
		{
			mavlink_scaled_pressure3_t scaled_pressure3;

			mavlink_msg_scaled_pressure3_decode(&message, &scaled_pressure3);

			fprintf(stderr, "MAVLINK_MSG_ID_SCALED_PRESSURE3\n");
		}
		break;

		case MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE:
		{
			mavlink_control_system_state_t control_system_state;

			mavlink_msg_control_system_state_decode(&message, &control_system_state);

			fprintf(stderr, "MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE\n");
		}
		break;

		case MAVLINK_MSG_ID_BATTERY_STATUS:
		{
			mavlink_battery_status_t battery_status;

			mavlink_msg_battery_status_decode(&message, &battery_status);

			fprintf(stderr, "MAVLINK_MSG_ID_BATTERY_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
		{
			mavlink_autopilot_version_t autopilot_version;

			mavlink_msg_autopilot_version_decode(&message, &autopilot_version);

			fprintf(stderr, "MAVLINK_MSG_ID_AUTOPILOT_VERSION\n");
		}
		break;

		case MAVLINK_MSG_ID_LANDING_TARGET:
		{
			mavlink_landing_target_t landing_target;

			mavlink_msg_landing_target_decode(&message, &landing_target);

			fprintf(stderr, "MAVLINK_MSG_ID_LANDING_TARGET\n");
		}
		break;

#ifdef LATER
		case MAVLINK_MSG_ID_SENSOR_OFFSETS:
		{
			mavlink_sensor_offsets_t sensor_offsets;

			mavlink_msg_sensor_offsets_decode(&message, &sensor_offsets);

			fprintf(stderr, "MAVLINK_MSG_ID_SENSOR_OFFSETS\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
		{
			mavlink_set_mag_offsets_t set_mag_offsets;

			mavlink_msg_set_mag_offsets_decode(&message, &set_mag_offsets);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_MAG_OFFSETS\n");
		}
		break;

		case MAVLINK_MSG_ID_MEMINFO:
		{
			mavlink_meminfo_t meminfo;

			mavlink_msg_meminfo_decode(&message, &meminfo);

			fprintf(stderr, "MAVLINK_MSG_ID_MEMINFO\n");
		}
		break;

		case MAVLINK_MSG_ID_AP_ADC:
		{
			mavlink_ap_adc_t ap_adc;

			mavlink_msg_ap_adc_decode(&message, &ap_adc);

			fprintf(stderr, "MAVLINK_MSG_ID_AP_ADC\n");
		}
		break;

		case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
		{
			mavlink_digicam_configure_t digicam_configure;

			mavlink_msg_digicam_configure_decode(&message, &digicam_configure);

			fprintf(stderr, "MAVLINK_MSG_ID_DIGICAM_CONFIGURE\n");
		}
		break;

		case MAVLINK_MSG_ID_DIGICAM_CONTROL:
		{
			mavlink_digicam_control_t digicam_control;

			mavlink_msg_digicam_control_decode(&message, &digicam_control);

			fprintf(stderr, "MAVLINK_MSG_ID_DIGICAM_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
		{
			mavlink_mount_configure_t mount_configure;

			mavlink_msg_mount_configure_decode(&message, &mount_configure);

			fprintf(stderr, "MAVLINK_MSG_ID_MOUNT_CONFIGURE\n");
		}
		break;

		case MAVLINK_MSG_ID_MOUNT_CONTROL:
		{
			mavlink_mount_control_t mount_control;

			mavlink_msg_mount_control_decode(&message, &mount_control);

			fprintf(stderr, "MAVLINK_MSG_ID_MOUNT_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_MOUNT_STATUS:
		{
			mavlink_mount_status_t mount_status;

			mavlink_msg_mount_status_decode(&message, &mount_status);

			fprintf(stderr, "MAVLINK_MSG_ID_MOUNT_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_FENCE_POINT:
		{
			mavlink_fence_point_t fence_point;

			mavlink_msg_fence_point_decode(&message, &fence_point);

			fprintf(stderr, "MAVLINK_MSG_ID_FENCE_POINT\n");
		}
		break;

		case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
		{
			mavlink_fence_fetch_point_t fence_fetch_point;

			mavlink_msg_fence_fetch_point_decode(&message, &fence_fetch_point);

			fprintf(stderr, "MAVLINK_MSG_ID_FENCE_FETCH_POINT\n");
		}
		break;
#endif // LATER

		case MAVLINK_MSG_ID_FENCE_STATUS:
		{
			mavlink_fence_status_t fence_status;

			mavlink_msg_fence_status_decode(&message, &fence_status);

			fprintf(stderr, "MAVLINK_MSG_ID_FENCE_STATUS\n");
		}
		break;

#ifdef LATER
		case MAVLINK_MSG_ID_AHRS:
		{
			mavlink_ahrs_t ahrs;

			mavlink_msg_ahrs_decode(&message, &ahrs);

			fprintf(stderr, "MAVLINK_MSG_ID_AHRS\n");
		}
		break;

		case MAVLINK_MSG_ID_SIMSTATE:
		{
			mavlink_simstate_t simstate;

			mavlink_msg_simstate_decode(&message, &simstate);

			fprintf(stderr, "MAVLINK_MSG_ID_SIMSTATE\n");
		}
		break;

		case MAVLINK_MSG_ID_HWSTATUS:
		{
			mavlink_hwstatus_t hwstatus;

			mavlink_msg_hwstatus_decode(&message, &hwstatus);

			fprintf(stderr, "MAVLINK_MSG_ID_HWSTATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_RADIO:
		{
			mavlink_radio_t radio;

			mavlink_msg_radio_decode(&message, &radio);

			fprintf(stderr, "MAVLINK_MSG_ID_RADIO\n");
		}
		break;

		case MAVLINK_MSG_ID_LIMITS_STATUS:
		{
			mavlink_limits_status_t limits_status;

			mavlink_msg_limits_status_decode(&message, &limits_status);

			fprintf(stderr, "MAVLINK_MSG_ID_LIMITS_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_WIND:
		{
			mavlink_wind_t wind;

			mavlink_msg_wind_decode(&message, &wind);

			fprintf(stderr, "MAVLINK_MSG_ID_WIND\n");
		}
		break;

		case MAVLINK_MSG_ID_DATA16:
		{
			mavlink_data16_t data16;

			mavlink_msg_data16_decode(&message, &data16);

			fprintf(stderr, "MAVLINK_MSG_ID_DATA16\n");
		}
		break;

		case MAVLINK_MSG_ID_DATA32:
		{
			mavlink_data32_t data32;

			mavlink_msg_data32_decode(&message, &data32);

			fprintf(stderr, "MAVLINK_MSG_ID_DATA32\n");
		}
		break;

		case MAVLINK_MSG_ID_DATA64:
		{
			mavlink_data64_t data64;

			mavlink_msg_data64_decode(&message, &data64);

			fprintf(stderr, "MAVLINK_MSG_ID_DATA64\n");
		}
		break;

		case MAVLINK_MSG_ID_DATA96:
		{
			mavlink_data96_t data96;

			mavlink_msg_data96_decode(&message, &data96);

			fprintf(stderr, "MAVLINK_MSG_ID_DATA96\n");
		}
		break;

		case MAVLINK_MSG_ID_RANGEFINDER:
		{
			mavlink_rangefinder_t rangefinder;

			mavlink_msg_rangefinder_decode(&message, &rangefinder);

			fprintf(stderr, "MAVLINK_MSG_ID_RANGEFINDER\n");
		}
		break;

		case MAVLINK_MSG_ID_AIRSPEED_AUTOCAL:
		{
			mavlink_airspeed_autocal_t airspeed_autocal;

			mavlink_msg_airspeed_autocal_decode(&message, &airspeed_autocal);

			fprintf(stderr, "MAVLINK_MSG_ID_AIRSPEED_AUTOCAL\n");
		}
		break;

		case MAVLINK_MSG_ID_RALLY_POINT:
		{
			mavlink_rally_point_t rally_point;

			mavlink_msg_rally_point_decode(&message, &rally_point);

			fprintf(stderr, "MAVLINK_MSG_ID_RALLY_POINT\n");
		}
		break;

		case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
		{
			mavlink_rally_fetch_point_t rally_fetch_point;

			mavlink_msg_rally_fetch_point_decode(&message, &rally_fetch_point);

			fprintf(stderr, "MAVLINK_MSG_ID_RALLY_FETCH_POINT\n");
		}
		break;

		case MAVLINK_MSG_ID_COMPASSMOT_STATUS:
		{
			mavlink_compassmot_status_t compassmot_status;

			mavlink_msg_compassmot_status_decode(&message, &compassmot_status);

			fprintf(stderr, "MAVLINK_MSG_ID_COMPASSMOT_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_AHRS2:
		{
			mavlink_ahrs2_t ahrs2;

			mavlink_msg_ahrs2_decode(&message, &ahrs2);

			fprintf(stderr, "MAVLINK_MSG_ID_AHRS2\n");
		}
		break;

		case MAVLINK_MSG_ID_CAMERA_STATUS:
		{
			mavlink_camera_status_t camera_status;

			mavlink_msg_camera_status_decode(&message, &camera_status);

			fprintf(stderr, "MAVLINK_MSG_ID_CAMERA_STATUS\n");
		}
		break;

		case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
		{
			mavlink_camera_feedback_t camera_feedback;

			mavlink_msg_camera_feedback_decode(&message, &camera_feedback);

			fprintf(stderr, "MAVLINK_MSG_ID_CAMERA_FEEDBACK\n");
		}
		break;

		case MAVLINK_MSG_ID_BATTERY2:
		{
			mavlink_battery2_t battery2;

			mavlink_msg_battery2_decode(&message, &battery2);

			fprintf(stderr, "MAVLINK_MSG_ID_BATTERY2\n");
		}
		break;

		case MAVLINK_MSG_ID_AHRS3:
		{
			mavlink_ahrs3_t ahrs3;

			mavlink_msg_ahrs3_decode(&message, &ahrs3);

			fprintf(stderr, "MAVLINK_MSG_ID_AHRS3\n");
		}
		break;

		case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
		{
			mavlink_autopilot_version_request_t autopilot_version_request;

			mavlink_msg_autopilot_version_request_decode(&message, &autopilot_version_request);

			fprintf(stderr, "MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST\n");
		}
		break;

		case MAVLINK_MSG_ID_LED_CONTROL:
		{
			mavlink_led_control_t led_control;

			mavlink_msg_led_control_decode(&message, &led_control);

			fprintf(stderr, "MAVLINK_MSG_ID_LED_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
		{
			mavlink_mag_cal_progress_t mag_cal_progress;

			mavlink_msg_mag_cal_progress_decode(&message, &mag_cal_progress);

			fprintf(stderr, "MAVLINK_MSG_ID_MAG_CAL_PROGRESS\n");
		}
		break;
#endif  // LATER

		case MAVLINK_MSG_ID_MAG_CAL_REPORT:
		{
			mavlink_mag_cal_report_t mag_cal_report;

			mavlink_msg_mag_cal_report_decode(&message, &mag_cal_report);

			fprintf(stderr, "MAVLINK_MSG_ID_MAG_CAL_REPORT\n");
		}
		break;

#ifdef LATER
		case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
		{
			mavlink_ekf_status_report_t ekf_status_report;

			mavlink_msg_ekf_status_report_decode(&message, &ekf_status_report);

			fprintf(stderr, "MAVLINK_MSG_ID_EKF_STATUS_REPORT\n");
		}
		break;

		case MAVLINK_MSG_ID_PID_TUNING:
		{
			mavlink_pid_tuning_t pid_tuning;

			mavlink_msg_pid_tuning_decode(&message, &pid_tuning);

			fprintf(stderr, "MAVLINK_MSG_ID_PID_TUNING\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_REPORT:
		{
			mavlink_gimbal_report_t gimbal_report;

			mavlink_msg_gimbal_report_decode(&message, &gimbal_report);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_REPORT\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_CONTROL:
		{
			mavlink_gimbal_control_t gimbal_control;

			mavlink_msg_gimbal_control_decode(&message, &gimbal_control);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_CONTROL\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_RESET:
		{
			mavlink_gimbal_reset_t gimbal_reset;

			mavlink_msg_gimbal_reset_decode(&message, &gimbal_reset);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_RESET\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS:
		{
			mavlink_gimbal_axis_calibration_progress_t gimbal_axis_calibration_progress;

			mavlink_msg_gimbal_axis_calibration_progress_decode(&message, &gimbal_axis_calibration_progress);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_SET_HOME_OFFSETS:
		{
			mavlink_gimbal_set_home_offsets_t gimbal_set_home_offsets;

			mavlink_msg_gimbal_set_home_offsets_decode(&message, &gimbal_set_home_offsets);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_SET_HOME_OFFSETS\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT:
		{
			mavlink_gimbal_home_offset_calibration_result_t gimbal_home_offset_calibration_result;

			mavlink_msg_gimbal_home_offset_calibration_result_decode(&message, &gimbal_home_offset_calibration_result);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS:
		{
			mavlink_gimbal_set_factory_parameters_t gimbal_set_factory_parameters;

			mavlink_msg_gimbal_set_factory_parameters_decode(&message, &gimbal_set_factory_parameters);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_FACTORY_PARAMETERS_LOADED:
		{
			mavlink_gimbal_factory_parameters_loaded_t gimbal_factory_parameters_loaded;

			mavlink_msg_gimbal_factory_parameters_loaded_decode(&message, &gimbal_factory_parameters_loaded);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_FACTORY_PARAMETERS_LOADED\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG:
		{
			mavlink_gimbal_erase_firmware_and_config_t gimbal_erase_firmware_and_config;

			mavlink_msg_gimbal_erase_firmware_and_config_decode(&message, &gimbal_erase_firmware_and_config);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS:
		{
			mavlink_gimbal_perform_factory_tests_t gimbal_perform_factory_tests;

			mavlink_msg_gimbal_perform_factory_tests_decode(&message, &gimbal_perform_factory_tests);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS\n");
		}
		break;

		case MAVLINK_MSG_ID_GIMBAL_REPORT_FACTORY_TESTS_PROGRESS:
		{
			mavlink_gimbal_report_factory_tests_progress_t gimbal_report_factory_tests_progress;

			mavlink_msg_gimbal_report_factory_tests_progress_decode(&message, &gimbal_report_factory_tests_progress);

			fprintf(stderr, "MAVLINK_MSG_ID_GIMBAL_REPORT_FACTORY_TESTS_PROGRESS\n");
		}
		break;

		case MAVLINK_MSG_ID_GOPRO_POWER_ON:
		{
			mavlink_gopro_power_on_t gopro_power_on;

			mavlink_msg_gopro_power_on_decode(&message, &gopro_power_on);

			fprintf(stderr, "MAVLINK_MSG_ID_GOPRO_POWER_ON\n");
		}
		break;

		case MAVLINK_MSG_ID_GOPRO_POWER_OFF:
		{
			mavlink_gopro_power_off_t gopro_power_off;

			mavlink_msg_gopro_power_off_decode(&message, &gopro_power_off);

			fprintf(stderr, "MAVLINK_MSG_ID_GOPRO_POWER_OFF\n");
		}
		break;

		case MAVLINK_MSG_ID_GOPRO_COMMAND:
		{
			mavlink_gopro_command_t gopro_command;

			mavlink_msg_gopro_command_decode(&message, &gopro_command);

			fprintf(stderr, "MAVLINK_MSG_ID_GOPRO_COMMAND\n");
		}
		break;

		case MAVLINK_MSG_ID_GOPRO_RESPONSE:
		{
			mavlink_gopro_response_t gopro_response;

			mavlink_msg_gopro_response_decode(&message, &gopro_response);

			fprintf(stderr, "MAVLINK_MSG_ID_GOPRO_RESPONSE\n");
		}
		break;

		case MAVLINK_MSG_ID_RPM:
		{
			mavlink_rpm_t rpm;

			mavlink_msg_rpm_decode(&message, &rpm);

			fprintf(stderr, "MAVLINK_MSG_ID_RPM\n");
		}
		break;
#endif // LATER

		case MAVLINK_MSG_ID_VIBRATION:
		{
			mavlink_vibration_t vibration;

			mavlink_msg_vibration_decode(&message, &vibration);

			fprintf(stderr, "MAVLINK_MSG_ID_VIBRATION\n");
		}
		break;

		case MAVLINK_MSG_ID_HOME_POSITION:
		{
			mavlink_home_position_t home_position;

			mavlink_msg_home_position_decode(&message, &home_position);

			fprintf(stderr, "MAVLINK_MSG_ID_HOME_POSITION\n");
		}
		break;

		case MAVLINK_MSG_ID_SET_HOME_POSITION:
		{
			mavlink_set_home_position_t set_home_position;

			mavlink_msg_set_home_position_decode(&message, &set_home_position);

			fprintf(stderr, "MAVLINK_MSG_ID_SET_HOME_POSITION\n");
		}
		break;

		case MAVLINK_MSG_ID_MESSAGE_INTERVAL:
		{
			mavlink_message_interval_t message_interval;

			mavlink_msg_message_interval_decode(&message, &message_interval);

			fprintf(stderr, "MAVLINK_MSG_ID_MESSAGE_INTERVAL\n");
		}
		break;

		case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
		{
			mavlink_extended_sys_state_t extended_sys_state;

			mavlink_msg_extended_sys_state_decode(&message, &extended_sys_state);

			fprintf(stderr, "MAVLINK_MSG_ID_EXTENDED_SYS_STATE\n");
		}
		break;

		case MAVLINK_MSG_ID_ADSB_VEHICLE:
		{
			mavlink_adsb_vehicle_t adsb_vehicle;

			mavlink_msg_adsb_vehicle_decode(&message, &adsb_vehicle);

			fprintf(stderr, "MAVLINK_MSG_ID_ADSB_VEHICLE\n");
		}
		break;

		case MAVLINK_MSG_ID_V2_EXTENSION:
		{
			mavlink_v2_extension_t v2_extension;

			mavlink_msg_v2_extension_decode(&message, &v2_extension);

			fprintf(stderr, "MAVLINK_MSG_ID_V2_EXTENSION\n");
		}
		break;

		case MAVLINK_MSG_ID_MEMORY_VECT:
		{
			mavlink_memory_vect_t memory_vect;

			mavlink_msg_memory_vect_decode(&message, &memory_vect);

			fprintf(stderr, "MAVLINK_MSG_ID_MEMORY_VECT\n");
		}
		break;

		case MAVLINK_MSG_ID_DEBUG_VECT:
		{
			mavlink_debug_vect_t debug_vect;

			mavlink_msg_debug_vect_decode(&message, &debug_vect);

			fprintf(stderr, "MAVLINK_MSG_ID_DEBUG_VECT\n");
		}
		break;

		case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		{
			mavlink_named_value_float_t named_value_float;

			mavlink_msg_named_value_float_decode(&message, &named_value_float);

			fprintf(stderr, "MAVLINK_MSG_ID_NAMED_VALUE_FLOAT\n");
		}
		break;

		case MAVLINK_MSG_ID_NAMED_VALUE_INT:
		{
			mavlink_named_value_int_t named_value_int;

			mavlink_msg_named_value_int_decode(&message, &named_value_int);

			fprintf(stderr, "MAVLINK_MSG_ID_NAMED_VALUE_INT\n");
		}
		break;

		case MAVLINK_MSG_ID_STATUSTEXT:
		{
			mavlink_statustext_t statustext;

			mavlink_msg_statustext_decode(&message, &statustext);

			fprintf(stderr, "MAVLINK_MSG_ID_STATUSTEXT\n");
		}
		break;

		case MAVLINK_MSG_ID_DEBUG:
		{
			mavlink_debug_t debug;

			mavlink_msg_debug_decode(&message, &debug);

			fprintf(stderr, "MAVLINK_MSG_ID_DEBUG\n");
		}
		break;

#ifdef LATER
		case MAVLINK_MSG_ID_EXTENDED_MESSAGE:
		{
			mavlink_extended_message_t extended_message;

			mavlink_msg_extended_message_decode(&message, &extended_message);

			fprintf(stderr, "MAVLINK_MSG_ID_EXTENDED_MESSAGE\n");
		}
		break;
#endif // LATER

		default:
		{
			fprintf(stderr, "Warning, did not handle message id %i\n",message.msgid);
		}
		break;


		} // end: switch msgid

	}
}

int Autopilot_Interface::recv_message(SOCKET mavSocket, int bufSize, char *dataBuf)
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
	int dataLen = 0;


	dataLen = recvfrom(mavSocket, dataBuf, bufSize, 0, (SOCKADDR*)&remAddr, &remAddrSize);

	if (dataLen > 0)
	{
		mSendPort = ntohs(remAddr.sin_port);

		fprintf(stderr, "Received %d bytes on port %d\n", dataLen, remAddr.sin_port);

		mMsgDataIndex = 0;
		mFullDataLen = dataLen;

		lastStatus.packet_rx_drop_count = 0;
	}
	else
	{
		Sleep(10);
		// Couldn't read from port

//		fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d\n", dataLen, errno);
	}

	return(dataLen);
}

int Autopilot_Interface::send_message(SOCKET mavSocket, const mavlink_message_t& message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	unsigned short port = 14550;

	struct sockaddr_in mavAddr;
	mavAddr.sin_family = AF_INET;
	mavAddr.sin_port = htons(mSendPort);

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

// End Autopilot_Interface
 
