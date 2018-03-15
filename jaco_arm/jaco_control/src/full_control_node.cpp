#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>
#include <cmath>

class JacoFingerControl {
public:
	ros::NodeHandle n;

	ros::Subscriber subFingers;

	ros::Subscriber subWrist;
	ros::Subscriber subForearm;
	ros::Subscriber subUpperarm;

	ros::Subscriber imuCalibrator;
	ros::Subscriber fingerCalibrator;

	ros::Publisher joint_pub;

	sensor_msgs::JointState pos;

	std::list<float> thumbAngles;
	std::list<float> pointerAngles;
	std::list<float> middleAngles;

	std::vector<float> jointHomes;

	std_msgs::Float32MultiArray IMUoffsets;
	std_msgs::Float32MultiArray fingerOffsets;

	bool IMUcalibrated;
	bool fingerCalibrated;

	float upperX;
	float upperY;

	JacoFingerControl(){
		
		subFingers = n.subscribe("fingersPos", 1000, &JacoFingerControl::FingersPosCallback, this);

        subWrist = n.subscribe("imu1", 1000, &JacoFingerControl::WristPosCallback, this);
        subForearm = n.subscribe("imu2", 1000, &JacoFingerControl::ForearmPosCallback, this);
        subUpperarm = n.subscribe("imu3", 1000,  &JacoFingerControl::UpperarmPosCallback, this);

        imuCalibrator = n.subscribe("imuCalibrator", 1000, &JacoFingerControl::IMUCalibrateCallback, this);
        fingerCalibrator = n.subscribe("fingerCalibrator", 1000, &JacoFingerControl::FingerCalibrateCallback, this);


        pos.name.push_back("jaco_arm_0_joint");
        pos.name.push_back("jaco_arm_1_joint");
        pos.name.push_back("jaco_arm_2_joint");
        pos.name.push_back("jaco_arm_3_joint");
        pos.name.push_back("jaco_arm_4_joint");
        pos.name.push_back("jaco_arm_5_joint");

        pos.name.push_back("jaco_finger_joint_0");
        pos.name.push_back("jaco_finger_joint_2");
        pos.name.push_back("jaco_finger_joint_4");

        /*pos.position.push_back(-1.7299817256187797);
        pos.position.push_back(-1.7326067624496215 + (3.1415/2));
        pos.position.push_back(0.7036460166034555);
        pos.position.push_back(-0.8174465286212945);
        pos.position.push_back(1.5064013197669412);
        pos.position.push_back(3.135943485311694);*/

        pos.position.push_back(-0.83 - (3.1415/4));
        pos.position.push_back(0.0);
        pos.position.push_back(-1.6);
        pos.position.push_back(0.03);
        pos.position.push_back(-3.14);
        pos.position.push_back(-0.05);

        pos.position.push_back(0);
        pos.position.push_back(0);
        pos.position.push_back(0);
        

  		joint_pub = n.advertise<sensor_msgs::JointState>("/jaco/joint_control", 1000);

  		/*jointHomes.push_back(-1.7299817256187797);
        jointHomes.push_back(-1.7326067624496215 + (3.1415/2));
        jointHomes.push_back(0.7036460166034555);
        jointHomes.push_back(-0.8174465286212945);
        jointHomes.push_back(1.5064013197669412);
        jointHomes.push_back(3.135943485311694);*/

        jointHomes.push_back(-0.83 - (3.1415/4));
        jointHomes.push_back(0.0);
        jointHomes.push_back(-1.6);
        jointHomes.push_back(0.03);
        jointHomes.push_back(-3.14);
        jointHomes.push_back(-0.05);

        IMUcalibrated = false;
        fingerCalibrated = false;

        for (int i = 0; i<9; i++){
        	IMUoffsets.data.push_back(0.0);
        }

        for (int i = 0; i<3; i++){
        	fingerOffsets.data.push_back(0.0);
        }

        upperX = 0;
        upperY = 0;

	}

	float calculateAvg(std::list<float> list)
	{
	    double avg = 0;
	    std::list<float>::iterator it;
	    for(it = list.begin(); it != list.end(); it++) avg += *it;
	    avg /= list.size();
		return avg;
	}

	float map(float value, float home, std::vector<int> sensor_range, std::vector<float> joint_range){ 

		if(value == 0){return home;}
		float num = (((value-sensor_range[0])/(sensor_range[1]-sensor_range[0]))*(joint_range[1]-joint_range[0]))+joint_range[0];

/*		std::cout << "Lower Joint Limit " << joint_range[0] << std::endl; 
		std::cout << "Upper Joint Limit " << joint_range[1] << std::endl;

		std::cout << "Lower IMU Limit " << sensor_range[0] << std::endl; 
		std::cout << "Upper IMU Limit " << sensor_range[1] << std::endl; 

		std::cout << "Value Read " << value << std::endl;
		std::cout << "Value Mapped " << num << std::endl;*/

		if (num > joint_range[1])
			{
				return  joint_range[1];
			}
		else if (num < joint_range[0])
			{
				return  joint_range[0];
			}
		else
			{
				return num;
			}
	}

	float fingerMap(int value, float max, float min){
		float angle = (value - max)/-(max-min);
		
		if(angle < 0.001){
			angle = 0;
		}
		
		else if(angle >= 1){
		  angle = 1;
		}

		return angle;
	}

	void IMUCalibrateCallback(const std_msgs::Float32MultiArray& msg){
		IMUoffsets.data = msg.data;
		IMUcalibrated = 1;
	}

	void FingerCalibrateCallback(const std_msgs::Float32MultiArray& msg){
		fingerOffsets.data = msg.data;
		fingerCalibrated = 1;
	}

	void WristPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x - IMUoffsets.data[0];
		float y = msg.y - IMUoffsets.data[1];
		float z = msg.z - IMUoffsets.data[2];

		std::vector<int> xrange;
		xrange.push_back(-25);
		xrange.push_back(25);

		std::vector<int> yrange;
		yrange.push_back(-50);
		yrange.push_back(50);

		std::vector<int> zrange;
		zrange.push_back(45);
		zrange.push_back(-45);

		std::vector<float> xJointrange;
		xJointrange.push_back(jointHomes[5]-1);
		xJointrange.push_back(jointHomes[5]+1);

		std::vector<float> yJointrange;
		yJointrange.push_back(jointHomes[6]-1);
		yJointrange.push_back(jointHomes[6]+1);

		std::vector<float> zJointrange;
		zJointrange.push_back(-3.14);
		zJointrange.push_back(-1.68);

		float ansz = map(z, jointHomes[4], zrange, zJointrange);

		pos.position[4]=ansz;
	}

	void ForearmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x - IMUoffsets.data[3];
		float y = msg.y - IMUoffsets.data[4];

		float x0 = upperX;
		float y0 = upperY;

		if (x >= 180){
			x = x - 360;
		}

		if (y >= 180){
			y = y - 360;
		}

		if (x0 >= 180){
			x0 = x0 - 360;
		}

		if (y0 >= 180){
			y0 = y0 - 360;
		}

		std::vector<int> yrange;
		yrange.push_back(-3); //arm up
		yrange.push_back(-65); //arm down

		std::vector<float> rangeJoint2;
		rangeJoint2.push_back(jointHomes[2]);
		rangeJoint2.push_back(jointHomes[2]+1.5);

		float num = (x - x0) /*+ (y - y0)*/;

		std::cout << "DIFFERENCE: " << x - x0 << std::endl;

		float ans_joint2 = map(num, jointHomes[2], yrange, rangeJoint2);

		pos.position[2]= ans_joint2;
	}

	void UpperarmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x - IMUoffsets.data[0];
		float y = msg.y - IMUoffsets.data[1] + 180;
		float z = msg.z - IMUoffsets.data[2] + 180;

		upperY = msg.y - IMUoffsets.data[1];
		upperX = msg.x - IMUoffsets.data[0];

		std::vector<int> xrange;
		//std::cout << "X OFFSET" <<IMUoffsets.data[0] << std::endl;
		xrange.push_back(-45); //40, 130
		xrange.push_back(45);

		std::vector<int> yrange;
		yrange.push_back(-52 +180); //45, -60
		yrange.push_back(+52 +180);

		std::vector<int> zrange;
		zrange.push_back(IMUoffsets.data[2] - 15 +180); //5, -25
		zrange.push_back(IMUoffsets.data[2] + 15 +180);

		std::vector<float> rangeJoint0;
		rangeJoint0.push_back(jointHomes[0]-1);
		rangeJoint0.push_back(jointHomes[0]+1);

		std::vector<float> rangeJoint1;
		rangeJoint1.push_back(jointHomes[1]-.7);
		rangeJoint1.push_back(jointHomes[1]+.7);

		float ans_joint0 = map((xrange[1]-x), jointHomes[0], xrange, rangeJoint0);

		float yzrangemin = sqrt(pow(yrange[0],2)+pow(zrange[0],2)); // = 7.8
		float yzrangemax = sqrt(pow(yrange[1],2)+pow(zrange[1],2)); // = 69.6

		std::vector<int> yzrange;
		yzrange.push_back(yzrangemin);
		yzrange.push_back(yzrangemax);
		
		float num = sqrt(pow(y,2)+pow(z,2));

		float ans_joint1 = map(y, jointHomes[1], yrange, rangeJoint1);

		pos.position[0]=ans_joint0;
		pos.position[1]=ans_joint1;
	}

	void FingersPosCallback(const geometry_msgs::Vector3& msg){
	  int thumb =  msg.x;
	  int pointer = msg.y;
	  int middle = msg.z;

	  float ansThumb = fingerMap(thumb, fingerOffsets.data[0], fingerOffsets.data[3]);
	  float ansPointer = fingerMap(pointer, fingerOffsets.data[1], fingerOffsets.data[4]);
	  float ansMiddle = fingerMap(middle, fingerOffsets.data[2], fingerOffsets.data[5]);

	  pos.position[6] = ansThumb;
	  pos.position[7] = ansPointer;
	  pos.position[8] = ansMiddle;

	}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "asdf");

  JacoFingerControl jc;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    //jc.joint_pub.publish(jc.pos);

    if (jc.IMUcalibrated and jc.fingerCalibrated){
    	jc.joint_pub.publish(jc.pos);
    	//std::cout << "publishing" << std::endl;
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}