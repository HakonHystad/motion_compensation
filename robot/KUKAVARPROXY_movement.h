/*
Source code for the KUKA robot controller communication client.
Author: Eirik B. Njaastad.
NTNU 2015

Communicates with the KUKAVARPROXY server made by
Massimiliano Fago - massimiliano.fago@gmail.com
*/

#ifndef BOOSTCLIENTCROSS
#define BOOSTCLIENTCROSS

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>


// THIS CODE WAS PROVIDED BY IVAR ERIKSEN
typedef unsigned char BYTE;

class BoostClientCross{
private:
    boost::asio::io_service iosClientCross;
    boost::system::error_code socketError;
    boost::asio::ip::tcp::socket *socketClientCross;

public:
  BoostClientCross(){
    socketClientCross = new boost::asio::ip::tcp::socket(iosClientCross);

  }

	// Function for opening a socket connection and initiate the server connection:
	void connectSocket(std::string ipAddress, std::string portNumber){
		socketClientCross->connect(
			boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ipAddress),
			boost::lexical_cast<unsigned>(portNumber)));
	}
	// For writing a variable to the robot controller, the message to send must contain
	// a variable name (varName) and a value to write (varValue).
	std::vector<unsigned char> formatWriteMsg(std::vector<unsigned char> varName, std::vector<unsigned char> varValue, int messageId = 05){
		std::vector<unsigned char> header, block;
		int varNameLength, varValueLength, blockSize;
//		int messageId;
		BYTE hbyte, lbyte, hbytemsg, lbytemsg;

		varNameLength = varName.size();
		varValueLength = varValue.size();
//		messageId = 05;

		hbyte = (BYTE)((varNameLength >> 8) & 0xff00);
		lbyte = (BYTE)(varNameLength & 0x00ff);

		block.push_back((unsigned char)1);
		block.push_back((unsigned char)hbyte);
		block.push_back((unsigned char)lbyte);

		for (int i = 0; i != varNameLength; ++i) {
			block.push_back(varName[i]);
		}

		hbyte = (BYTE)((varValueLength >> 8) & 0xff00);
		lbyte = (BYTE)(varValueLength & 0x00ff);

		block.push_back((unsigned char)hbyte);
		block.push_back((unsigned char)lbyte);

		for (int i = 0; i != varValueLength; ++i) {
			block.push_back(varValue[i]);
		}

		blockSize = block.size();
		hbyte = (BYTE)((blockSize >> 8) & 0xff00);
		lbyte = (BYTE)(blockSize & 0x00ff);

		hbytemsg = (BYTE)((messageId >> 8) & 0xff00);
		lbytemsg = (BYTE)(messageId & 0x00ff);

		header.push_back((unsigned char)hbytemsg);
		header.push_back((unsigned char)lbytemsg);
		header.push_back((unsigned char)hbyte);
		header.push_back((unsigned char)lbyte);

		block.insert(block.begin(), header.begin(), header.end());
		return block;
	}
	// For reading a variable from the robot controller, the message to send must contain
	// the desired variable name (varName).
	std::vector<unsigned char> formatReadMsg(std::vector<unsigned char> varName, int messageId = 05){
		std::vector<unsigned char> header, block;
		int varNameLength, blockSize;
//		int messageId;
		BYTE hbyte, lbyte, hbytemsg, lbytemsg;

		varNameLength = varName.size();
//		messageId = 05;

		hbyte = (BYTE)((varNameLength >> 8) & 0xff00);
		lbyte = (BYTE)(varNameLength & 0x00ff);

		block.push_back((unsigned char)0);
		block.push_back((unsigned char)hbyte);
		block.push_back((unsigned char)lbyte);

		for (int i = 0; i != varNameLength; ++i) {
			block.push_back(varName[i]);
		}

		blockSize = block.size();

		hbyte = (BYTE)((blockSize >> 8) & 0xff00);
		lbyte = (BYTE)(blockSize & 0x00ff);

		hbytemsg = (BYTE)((messageId >> 8) & 0xff00);
		lbytemsg = (BYTE)(messageId & 0x00ff);

		header.push_back((unsigned char)hbytemsg);
		header.push_back((unsigned char)lbytemsg);
		header.push_back((unsigned char)hbyte);
		header.push_back((unsigned char)lbyte);

		block.insert(block.begin(), header.begin(), header.end());
		return block;
	}
	// Send the formatted message and recieve server response: 
	std::vector<unsigned char> sendMsg(std::vector<unsigned char> message){
		// Send message:
		const size_t bytes = boost::asio::write(*socketClientCross, boost::asio::buffer(message));
		
		// Read answer:
		boost::array<unsigned char, 7> recheader;
		size_t sendLen = socketClientCross->read_some(boost::asio::buffer(recheader), socketError); // Header
		int messageLength = recheader[3];
		std::vector<unsigned char> recblock(messageLength);
		size_t recLen = socketClientCross->read_some(boost::asio::buffer(recblock), socketError); // Message

		// Error handling:
		if (socketError == boost::asio::error::eof)
			std::cout << "Connection closed cleanly by peer" << std::endl;
		else if (socketError)
			throw boost::system::system_error(socketError); // Some other error.	

	
//		 Print results (alternative):
//		 std::cout << "received:: " << std::endl;
//		 for (int i = 0; i != recLen; ++i){
//			std::cout << recblock[i];
//  		}
		recblock.erase(recblock.end()-7,recblock.end());
		return recblock;
	}
	// Function for terminating the socket and thus disconnect from server:
	void disconnectSocket(){
		socketClientCross->shutdown(boost::asio::ip::tcp::socket::shutdown_both, socketError);
		socketClientCross->close();

		// Error handling:
		if (socketError)
			throw boost::system::system_error(socketError);
	}
}; 
// END OF CODE

namespace KUKA
{

// Robot Pose
// x, y and z are Cartesian coordinates
// a, b and c are the angles of the z, y and x axis respectively.
struct RobotPose {
	double x;
	double y;
	double z;
	double a;
	double b;
	double c;

	// Checks if the pose is within the error range.
	bool withinRange(struct RobotPose pose, double error){
		double tot_error = 0;
		tot_error += pow(x-pose.x,2);
		tot_error += pow(y-pose.y,2);
		tot_error += pow(z-pose.z,2);
		tot_error += pow(a-pose.a,2);
		tot_error += pow(b-pose.b,2);
		tot_error += pow(c-pose.c,2);
		return tot_error < error;
	}

	// Prints the pose to string
	std::string toString() {
		return "Position:\n\tX: " + std::to_string(x) +
				"\n\tY: " + std::to_string(y) +
				"\n\tZ: " + std::to_string(z) +
				"\n\tA: " + std::to_string(a) +
				"\n\tB: " + std::to_string(b) +
				"\n\tC: " + std::to_string(c); 
	}
};

// Class for robot movement
class Kuka {
private:
	// Variable for the target position to move to
	std::string target_pos_name_string = "TARGET_POS";
	std::vector<unsigned char> target_pos_name;

	// Variable for the current position of the robot
	std::string current_pos_name_string = "$POS_ACT";
	std::vector<unsigned char> current_pos_name;

	BoostClientCross robot;	

	// Double to string with 2 digit precition
	std::string dtostring(double number) {
		std::stringstream stream;
		stream << std::fixed << std::setprecision(2) << number;
		return stream.str();
	}

	// Creates a string format of the pose which KUKAVARPROXY can read
	std::string poseToString(struct RobotPose pose) {
		return "{E6POS: X " + dtostring(pose.x) + 
				", Y " + dtostring(pose.y) + 
				", Z " + dtostring(pose.z) + 
				", A " + dtostring(pose.a) + 	
				", B " + dtostring(pose.b) + 
				", C " + dtostring(pose.c) + 
				"}";
	}

	// Reads a KUKAVARPROXY string and converts it to a pose.
	struct RobotPose stringToPose(std::string string) {
		struct RobotPose res;
		size_t x_pos = string.find("X") + 2;
		res.x = stod(string.substr(x_pos));

		size_t y_pos = string.find("Y") + 2;
		res.y = stod(string.substr(y_pos));

		size_t z_pos = string.find("Z") + 2;
		res.z = stod(string.substr(z_pos));

		size_t a_pos = string.find("A") + 2;
		res.a = stod(string.substr(a_pos));

		size_t b_pos = string.find("B") + 2;
		res.b = stod(string.substr(b_pos));

		size_t c_pos = string.find("C") + 2;
		res.c = stod(string.substr(c_pos));
		return res;
	}

public:
	// Initializes the KUKAVARPROXY robot
	// Parameter: IP address of the robot
	Kuka(std::string kuka_ip = "192.168.250.120")
	{
	// Port of the IP address of the robot
	std::string kuka_port = "7000";
	
	std::vector<unsigned char> target_pos_temp(target_pos_name_string.begin(),target_pos_name_string.end());
	std::vector<unsigned char> current_pos_temp(current_pos_name_string.begin(),current_pos_name_string.end());

	// Creates a variable which contains the variable names in a KUKAVARPROXY format.
	target_pos_name  = robot.formatReadMsg(target_pos_temp,1);
	current_pos_name = robot.formatReadMsg(current_pos_temp,2);		

	robot.connectSocket(kuka_ip, kuka_port);
}

	// Gets the current position of the robot
	struct RobotPose CurrentPosition(){
		std::vector<unsigned char> res = robot.sendMsg(current_pos_name);
		std::string data(reinterpret_cast<char*>(res.data()));
		return stringToPose(data);
	}

	// Sets a position which the robot will move to
	// The movement is PTP, and the robot will complete the movement 
	// before reading any new incomming positions.
	// The robot will also write over any previous movements if a new one is given.
	void GoToPosition(struct RobotPose pose) {
		std::string pose_string = poseToString(pose);
		std::vector<unsigned char> target_param(pose_string.begin(),pose_string.end());
		std::vector<unsigned char> target_name(target_pos_name_string.begin(),target_pos_name_string.end());
		std::vector<unsigned char> send_pose_msg = robot.formatWriteMsg(target_name, target_param, 1);

		robot.sendMsg(send_pose_msg);
	}

	// Checks if the specific pose has been reached.
	bool PositionReached(struct RobotPose pose) {
		auto curr_pose = CurrentPosition();
		return pose.withinRange(curr_pose, 10);
	}

	// Disconnects the robot.
	void Disconnect() {
		robot.disconnectSocket();
	}
};

}// namespace KUKA

#endif

