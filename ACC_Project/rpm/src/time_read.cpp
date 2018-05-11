#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <string>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<rpm/datarpm.h>
using namespace std;
int main(int argc, char **argv){
    ros::init(argc, argv, "rpm");
    ros::NodeHandle n;
    //ros::Publisher rpm_pub = n.advertise<std_msgs::String>("RPM_TPOIC", 1000);
    ros::Publisher rpm_pub = n.advertise<rpm::datarpm>("RPM_TPOIC", 1000);
    ros::Rate loop_rate(10);
    cout << "Begin to read the velocity..." << endl;
    while((ros::ok())){
        int receive;
        std_msgs::String msg;
        rpm::datarpm msg2;
        stringstream stream;
        int ret, fd;
        fd = open("/dev/RPM", O_RDWR);
        if(fd<0){
            perror("Failed to open the device...");
            return errno;
        }
        ret = read(fd, &receive, sizeof(int));
        int temp;
        temp = receive;
        stream << temp;
        msg.data = stream.str();
        msg2.R = temp;
	//cout<< temp <<endl;
	//ROS_INFO("temp = %d", temp);
	
        ROS_INFO("Velocity = %s", msg.data.c_str());
        rpm_pub.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

