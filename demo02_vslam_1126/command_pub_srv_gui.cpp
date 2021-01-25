#include "ros/ros.h"
#include <std_msgs/String.h>
#include "navigation_controller/command.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>

////////////// This is for demo 02 : /map ////////////////////

class CommandPubSrvGui {
public:
	CommandPubSrvGui();
	~CommandPubSrvGui();

private:
	ros::NodeHandle nh_;
	ros::Subscriber place_cmd_;
	ros::ServiceClient client_;
	ros::Publisher reach_pub_;


	ros::Publisher flag_switch;
	std_msgs::Bool cmd_switch;


	void get_place(const std_msgs::String::ConstPtr& place);
};

CommandPubSrvGui::CommandPubSrvGui()
{
	place_cmd_ = nh_.subscribe<std_msgs::String>("place", 1000, boost::bind(&CommandPubSrvGui::get_place, this, _1));
	client_ = nh_.serviceClient<navigation_controller::command>("pos_cmd");

	flag_switch = nh_.advertise<std_msgs::Bool>("yourturn", 1);
	//cmd_switch.data
}

CommandPubSrvGui::~CommandPubSrvGui()
{
	
}

/////////////// Demo02 ////////////////////
#include <boost/math/constants/constants.hpp>
const double pi = boost::math::constants::pi<double>();
#define STOP_INDEX 8
double loc_x[8] = {2.1, 2.1, 0,    0, 2.1, 2.1, 0,    0};
double loc_y[8] = {0,   1.2, 1.2,  0,   0, 1.2, 1.2,  0};
double loc_theta[8] = {pi/2, pi, pi*3/2, 0, pi/2, pi, pi*3/2, 0}; //radian 
int loc_index = 0;
bool not_finish = true;


void CommandPubSrvGui::get_place(const std_msgs::String::ConstPtr& place)
{
	int num = 0;
	int flag=0;
	int wait=0;
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	navigation_controller::command srv;

	std::string path = "/home/locobot/low_cost_ws_my/src/pyrobot/robots/LoCoBot/navigation_controller/cmd_txt/cmd" + place->data + ".txt";
	std::fstream myfile;
	myfile.open(path.c_str());

	// while (myfile >> num && ros::ok()) {
	while (ros::ok()) {        
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		
 
  ////////////////////////////////TODO///////////////////////////////
 
        x = loc_x[loc_index];
        y = loc_y[loc_index];
        theta = loc_theta[loc_index];
 
  ////////////////////////////////TODO///////////////////////////////
 
 
		ROS_INFO_STREAM("x = " << x << " y = " << y << " theta = " << theta);
		if(not_finish) srv.request.type = 2;
        else srv.request.type = 0;
		srv.request.x = x;
		srv.request.y = y;
		srv.request.theta = theta;

		if(client_.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) { // still running.
				usleep(100000);
				ROS_INFO("hello servie");
				client_.call(srv);
			}
            // reach point. Send another place.
            // loc_index++;
            // if(loc_index==4){
            //     loc_index = 4;
            // }
            // else{
            //     loc_index++;
            // }
			sleep(wait);
		}
		else {
			ROS_INFO("call service fail");
		}

		if(not_finish) srv.request.type = 1;
        else srv.request.type = 0;
    
		if(client_.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) { // still running.
				usleep(100000);
				ROS_INFO("hello servie");
				client_.call(srv);
			}
            // reach point. Send another place.
            // loc_index++;
            if(loc_index== STOP_INDEX){
                not_finish = false;
                loc_index = STOP_INDEX;
            }
            else{
                loc_index++;
            }
			sleep(wait);
		}
		else {
			ROS_INFO("call service fail");
		}


	}


	cmd_switch.data = true;
	for(int i=0;i<3;i++){
		flag_switch.publish(cmd_switch);
		ROS_ERROR("-----------------------------------");
		ros::Duration(1.0).sleep();
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub_srv_gui");
	CommandPubSrvGui commandpubsrvgui;

	ros::spin();	
	return 0;
}
