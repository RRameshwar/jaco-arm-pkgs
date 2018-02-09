#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class thingy {
public:
	ros::NodeHandle n;

	ros::Subscriber subMiddle;
	ros::Subscriber subPointer;
	ros::Subscriber subThumb;

	ros::Publisher joint_pub;

	sensor_msgs::JointState pos;

	std::list<float> thumbAngles;
	std::list<float> pointerAngles;
	std::list<float> middleAngles;


	thingy(){
		subMiddle = n.subscribe("middlePos", 1000, &thingy::MiddlePosCallback, this);
		subPointer = n.subscribe("pointerPos", 1000, &thingy::PointerPosCallback, this);
		subThumb = n.subscribe("thumbPos", 1000, &thingy::ThumbPosCallback, this);

  		joint_pub = n.advertise<sensor_msgs::JointState>("/jaco/joint_control", 1000);

  		pos.name.push_back("jaco_finger_joint_0");
  		pos.position.push_back(0);
  		
  		pos.name.push_back("jaco_finger_joint_2");
  		pos.position.push_back(0);
  		
  		pos.name.push_back("jaco_finger_joint_4");
  		pos.position.push_back(0);

/*  		pos.velocity.push_back(0.3);
  		pos.velocity.push_back(0.3);
  		pos.velocity.push_back(0.3);*/

  		//pos.name = ['jaco_finger_joint_0', 'jaco_finger_joint_2', 'jaco_finger_joint_4'];
		//pos.position = [0, 0, 0];
	}

	float calculateAvg(std::list<float> list)
	{
	    double avg = 0;
	    std::list<float>::iterator it;
	    for(it = list.begin(); it != list.end(); it++) avg += *it;
	    avg /= list.size();
		return avg;
	}

	void MiddlePosCallback(const std_msgs::Int16& msg){
	  int curve =  msg.data;

	  float angle = (curve - 750.0)/(-
	  	670.0);

  	  if(angle < 0.001){
	  	angle = 0;
	  }
	  else if(angle >= 1){
	  	angle = 1;
	  }

	  if(middleAngles.size()<5){
	  	middleAngles.push_back(angle);
	  }
	  else{
	  	pos.position[2] = calculateAvg(middleAngles);
	  	middleAngles.clear();
	  	middleAngles.push_back(angle); 
	  }

	  pos.position[2] = angle;

	  std::cout << angle << std::endl;
	}

	
	void PointerPosCallback(const std_msgs::Int16& msg){
	  int curve =  msg.data;

	  float angle = (curve - 750.0)/(-500.0);


	  if(angle < 0.001){
	  	angle = 0;
	  }
	  else if(angle >= 1){
	  	angle = 1;
	  }

	  if(pointerAngles.size()<5){
	  	pointerAngles.push_back(angle);
	  }
	  else{
	  	pos.position[2] = calculateAvg(pointerAngles);
	  	pointerAngles.clear();
	  	pointerAngles.push_back(angle); 
	  }

	  pos.position[1] = angle;
	  std::cout << angle << std::endl;

	  	
	}

	void ThumbPosCallback(const std_msgs::Int16& msg){
	  int curve =  msg.data;

	  float angle = (curve - 750.0)/(-250.0);

  	  if(angle < 0.001){
	  	angle = 0;
	  }
	  else if(angle >= 1){
	  	angle = 1;
	  }

	  if(thumbAngles.size()<5){
	  	thumbAngles.push_back(angle);
	  }
	  else{
	  	pos.position[2] = calculateAvg(thumbAngles);
	  	thumbAngles.clear();
	  	thumbAngles.push_back(angle); 
	  }



	  pos.position[0] = angle;
	  std::cout << angle << std::endl;
	}
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  thingy thing;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    thing.joint_pub.publish(thing.pos);

    loop_rate.sleep();
    ++count;
  }

  return 0;
}