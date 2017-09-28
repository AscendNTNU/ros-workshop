#include "AscendDrone.hpp"
#include <ros/ros.h>

constexpr char NODE_NAME[] = "nodename";

int main(int argc, char** argv) {
    //Initialiserer ROS
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    //Lag grensesnitt mot de andre gruppene!
    //Send dronen til riktig mål!
    while(ros::ok()) {
        //Main loop

        ros::spinOnce();
    }
    
    //La ROS gjøre magien sin!  
    ros::spin();
    return 0;
}
