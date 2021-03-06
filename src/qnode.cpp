/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <iostream>
#include <mydrone_gui_keyboard/qnode.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <time.h>


#define PI 3.14159265358979323846


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mydrone_gui_keyboard {

	using namespace std;

		double positionX=0;
		double positionY=0;
 		double positionZ=0;
		double yaw_rate=0;
		double roll=0;
		double pitch=0;
		double yaw=0;
		double velocity_x=0;
		double velocity_y=0;
		double velocity_z=0;

//offboard mode로 진입했을때 position 컨트롤 초기값//
		float target_x=0;
		float target_y=0;
		float target_z=0;

		float targetO_x=0;
		float targetO_y=0;
		float targetO_z=0;
		float targetO_w=0;



//


		//시간함수 : connect이후부터 시간을 잽니다.//
 		ros::WallTime startTime;



	  QVariant new_row; // log함수에서 사용

 		mavros_msgs::State current_state; // 현재 드론 상태 받는 변수


	//서브스크라이버 콜백함수
	void state_cb(const mavros_msgs::State::ConstPtr& msg){
			current_state = *msg;
	}

	void position_cb(const nav_msgs::Odometry::ConstPtr& msg){


		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);


		positionX = msg->pose.pose.position.x;
		positionY = msg->pose.pose.position.y;
		positionZ = msg->pose.pose.position.z;
	}

	void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
		velocity_x = msg->twist.linear.x;
		velocity_y = msg->twist.linear.y;
		velocity_z = msg->twist.linear.z;
	}

	void odom_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
		target_x = msg->pose.position.x;
		target_y = msg->pose.position.y;
		target_z = msg->pose.position.z;

		targetO_x = msg->pose.orientation.x;
		targetO_y = msg->pose.orientation.y;
		targetO_z = msg->pose.orientation.z;
		targetO_w = msg->pose.orientation.w;

	}

/*****************************************************************************
** Implementation
*****************************************************************************/



QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
		arming = false;
		offboard = false;
		bool_recoding = false;
		bool_tracking = false;
		firstWrite = true;
		filePath = "";

	}

void QNode::inputToTextfile(){ // msgThread.cpp 에 의해 지속적으로 기록
	if(writeFile.is_open() &&
					bool_recoding == true) {
			//ROS_INFO("저장됨");
			writeFile << ros::WallTime::now()-startTime << " " << positionX << " " << positionY << " " << positionZ
			<< " " << roll << " " << pitch << " " << yaw
			<< " " << target_x << " " << target_y << " " << target_z
			<< " " << velocity_x << " " << velocity_y << " " << velocity_z
			<< "\n";
		}
	}

QNode::~QNode() {
    if(ros::isStarted()) {

//텍스트 파일 저장 종료//
			if( writeFile.is_open()){
				writeFile.close();
			}
			////


      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}



bool QNode::init() {


	ros::init(init_argc,init_argv,"mydrone_gui_keyboard");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;


	// 파라미터 read //


	cout << ros::param::has("~txt_location") << endl;
	if(ros::param::get("~txt_location",filePath)){
	std::cout << "파일 이름 : " << filePath << std::endl;
	}

	startTime = ros::WallTime::now();//시작시간 기록//


	// Add your ros communications here.
	state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	position_sub = n.subscribe("mavros/local_position/odom", 10, position_cb);
	velocity_sub = n.subscribe("mavros/local_position/velocity_body",10,velocity_cb);
	odom_sub = n.subscribe<geometry_msgs::PoseStamped>("/odom_visualization/pose", 10, odom_cb);


	local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); // arming 미션//
	set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");// offboard 설정//
	arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming"); // arming
	land_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land"); // land

	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) { // qnode 수동설정으로 init

	return true;
}

void QNode::run() { // 여기서 계속 publish loop

	ros::Rate rate(20.0);

	while(ros::ok() && !current_state.connected){

    ros::spinOnce();// 콜백함수 호출을 위한 함수로써, 메시지가 수신되기를 대기, 수신되었을 경우 콜백함수를 실행한다
    rate.sleep();

	}

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "body";
	/*이건 ~setpoint_raw/local (mavros_msgs/PositionTarget) 이거 사용할때 쓰던 옵션
	pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	pose.type_mask =
	 mavros_msgs::PositionTarget::IGNORE_PZ |
	 mavros_msgs::PositionTarget::IGNORE_YAW;// | mavros_msgs::PositionTarget::추가하고싶은거//
	pose.velocity.x = veloX;
	pose.velocity.y = veloY;
	pose.velocity.z = veloZ;
	pose.yaw_rate = yaw_rate;
	*/


	for(int i = 100; ros::ok() && i > 0; --i){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
	mavros_msgs::CommandTOL land_cmd;


    ros::Time last_request = ros::Time::now();

		while(ros::ok()){
			MODELogging(std::string(current_state.mode));
			if(offboard == true){ // offboard 명령을 받으면//
				TARGETLogging(target_x,target_y,target_z); //target 로깅//
				//offboard 모드가 아니면 지속적으로 offboard 모드 진입 명령을 보냄//
				offb_set_mode.request.custom_mode = "OFFBOARD";
					if( current_state.mode != "OFFBOARD" &&
							(ros::Time::now() - last_request > ros::Duration(5.0))){
								if( set_mode_client.call(offb_set_mode) &&
									offb_set_mode.response.mode_sent){
										log(Info,"Offboard enabled");
									}
									last_request = ros::Time::now();
					}else{ // offboard로 모드가 바뀌면 //
						//arming이나 disarming 수행
						arm_cmd.request.value = arming;
						if(arm_cmd.request.value == true){ // arm 명령을 줄때 //
            	if( !current_state.armed &&
                	(ros::Time::now() - last_request > ros::Duration(5.0))){
                	if( arming_client.call(arm_cmd) && // arming 신호를 보냄//
                    	arm_cmd.response.success){
                    	log(Info,"Vehicle armed");
                	}
                	last_request = ros::Time::now();// 일정시간마다 service 날림//
            	}
						}else{ // disarm 명령을 줄 때 //
							// land후에 disarmed
							land_cmd.request.altitude = 0;
							if((ros::Time::now() - last_request > ros::Duration(5.0))){
								if(!land_cmd.response.success){
									land_client.call(land_cmd);
								}else{
									if(current_state.armed){
										if(arming_client.call(arm_cmd) && arm_cmd.response.success){
											last_request = ros::Time::now();
											log(Info,"Vehicle disarmed");
										}
									}
								}
								last_request = ros::Time::now(); // 일정시간마다 service 날림//
							}
						}
					}
			}else{ // onboard 명령을 받으면 //
				//STABILIZED 상태로 진입//
				offb_set_mode.request.custom_mode = "STABILIZED";
				if( current_state.mode == "OFFBOARD" &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
							if( set_mode_client.call(offb_set_mode) &&
								offb_set_mode.response.mode_sent){
									log(Info,"Onboard enabled");
								}
								last_request = ros::Time::now();
				}
			}


				//위치를 publish//
				if(offboard == true) {//offboard상태일때만 위치 publish
					if(bool_tracking == true){ // txt를 읽어 궤도를 따라 갈 때//
						if(ros::Time::now()-last_request >ros::Duration(0.1)){
								last_request = ros::Time::now();
								cin >> target_x;
								cin >> target_x >> target_y
								>> target_z;
								double r,p,y;
								cin >> r >> p >> y;
								tf2::Quaternion q;
								q.setRPY(r,p,y);
								targetO_x = q[0];
								targetO_y = q[1];
								targetO_z = q[2];
								targetO_w = q[3];
								//나머지 버리기
								string t;
								getline(cin,t);
						}
					}
						pose.pose.position.x = target_x;
						pose.pose.position.y = target_y;
						pose.pose.position.z = target_z;

						pose.pose.orientation.x = targetO_x;
						pose.pose.orientation.y = targetO_y;
						pose.pose.orientation.z = targetO_z;
						pose.pose.orientation.w = targetO_w;
						local_pos_pub.publish(pose);
				}

        ros::spinOnce();
        rate.sleep();
    }

}


void QNode::CommandOffboard(bool boolean){
	if(boolean == true){ //OFFBOARD 모드 명령이 떨어지면 TARGET X,Y,Z를 기본설정해줌//
		target_x = positionX;
		target_y = positionY;
		target_z = positionZ;
	}
	bool_tracking = false;
	offboard = boolean;
}

void QNode::CommandArm(bool boolean){
	arming = boolean;
}

void QNode::setPosZ(float a){
	target_z +=a;
}

void QNode::setPosX(float a){
	target_y += a;
}

void QNode::setPosY(float a){
	target_x += a;
}

/* 일단 안쓰는것 (속도 컨트롤 할때 썼던 것임)
void QNode::setYaw(float a){
	yaw_rate = a;
}
*/

float QNode::changeToRad(float a){
	return a*PI/180;
}

float QNode::ChangeToDegree(float a){
	return a*(180/PI);
}

void QNode::myLocationLogging(){
	/*
	std::stringstream ss;
	ss.str("");
	ss << "x :" << std::setprecision(2) <<positionX << " y : " << positionY << " z : " << positionZ;
	log(Info,ss.str());
	*/

	Q_EMIT sendNOWXYZ(positionX,positionY,positionZ);
	Q_EMIT sendPlotPoint(positionX,positionY,positionZ);

}

void QNode::myRPYLogging(){
	Q_EMIT sendRPY(ChangeToDegree(roll),ChangeToDegree(pitch),ChangeToDegree(yaw));
}

void QNode::MODELogging(std::string mode){
	Q_EMIT sendMODE(mode);
}

void QNode::TARGETLogging(float x,float y,float z){
	Q_EMIT sendTARGET(x,y,z);
}

void QNode::clearLogging(){
	logging_model.removeRows(0, logging_model.rowCount());
}

void QNode::startRecord(){
	if(firstWrite == true){ // 첫 startRecord버튼 누를시 //
		//파일 새로 생성 //
		//텍스트 파일 저장 관련 변수
				writeFile.open(filePath.data());
				writeFile << "time(sec) "<<"position_x " << "position_y " << "position_z "
				<< "roll " << "pitch " << "yaw "
				<< "target_x " << "target_y " << "target_z "
				<< "velocity_x " << "velocity_y " << "velocity_z "
				<< "\n";
				firstWrite = false;
		// 끝
	}
	bool_recoding = true;
}

void QNode::stopRecord(){
	bool_recoding = false;
}

void QNode::startTracking(){
	if(freopen(filePath.c_str(),"r",stdin) == NULL){
		ROS_WARN_STREAM("cannot find file location parameter");
		return;
	}

	string t;
	getline(cin,t);
	bool_tracking = true;
	return;
}

void QNode::stopTracking(){
	target_x = positionX;
	target_y = positionY;
	target_z = positionZ;
	bool_tracking = false;
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	ros::WallTime timeNow = ros::WallTime::now();
	std::stringstream logging_model_msg;
	logging_model.insertRows(logging_model.rowCount(),1);
	logging_model_msg.str("");
	switch ( level ) {
		case(Debug) : {
				logging_model_msg << "[DEBUG] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Info) : {
				logging_model_msg << "[INFO] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Warn) : {
				logging_model_msg << "[INFO] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Error) : {
				logging_model_msg << "[ERROR] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Fatal) : {
				logging_model_msg << "[FATAL] [" << timeNow-startTime << "]: " << msg;
				break;
		}
	}

	new_row = QString(logging_model_msg.str().c_str());
	//segmentation fault로 고생했었음, 이전에는 다음과 같은 코드를 썼는데
	//QVariant new_row(QString(logging_model_msg.str().c_str()));
//위 코드는 아래와 같은 의미
//QVariant new_row = new QVariant(QString(logging_model_msg.str().c_str()));
// 계속 new QVarient를 하니까 동적메모리 할당을 계속 하게되니 segmentation fault가 뜸//

	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // 로그 업데이트 시그널을 줘서 main_window.cpp의 자동스크롤 함수(슬롯)를 call
}

}
