#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "transform.h"

int main(int argc , char **argv)
{
	ros::init(argc, argv, "product");
	ros::NodeHandle nh;                    /* create a ROS Node */
	std::string product;

	nh.getParam("product", product);

	LiPkg * lidar;
	
	int32_t ver=0;

	if(product=="LD00")
	{
		ver = 0;
		lidar = new LD00_LiPkg; 
	}else if(product=="LD03")
	{
	    ver = 3;
		lidar = new LD03_LiPkg;
	}
	else if(product=="LD08")
	{
		ver = 8;
		lidar = new LD08_LiPkg;
	}else if(product=="LD09")
	{
		ver = 9;
		lidar = new LD09_LiPkg; 
	}
	 
	char product_ver[5]={0};   				/*production version*/
	strcpy(product_ver,product.c_str());
	char topic_name[20]={0};
	strcat(topic_name,product_ver);
	strcat(topic_name,"/LDLiDAR");
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 1); /*create a ROS topic */

    CmdInterfaceLinux cmd_port(ver);
    std::vector<std::pair<std::string, std::string> > device_list;
    std::string port_name;
    cmd_port.GetCmdDevices(device_list);
    for (auto n : device_list)
    {
        std::cout << n.first << "    " << n.second << std::endl;
        if(strstr(n.second.c_str(),"CP2102"))
        {
            port_name = n.first;
        }
    }

	if(port_name.empty())
	{
		std::cout<<"Can't find LiDAR"<< product << std::endl;
	}

	std::cout<<"FOUND LiDAR_" <<  product_ver  <<std::endl;
	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		if(lidar->Parse((uint8_t*)byte, len))
		{
			lidar->AssemblePacket();  
		}
	});
	cmd_port.Open(port_name);

	while (ros::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub.publish(lidar->GetLaserScan());
			lidar->ResetFrameReady();
		}
	}

    return 0;
}

