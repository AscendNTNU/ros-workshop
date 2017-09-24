#include "AscendDrone.hpp"

constexpr char NODE_NAME[] = "nodename";

int main(int argc, char** argv) {
	//Initialiserer ROS
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	
	//Enkelt grensesnitt mot dronen for å hente ut posisjon
	PerceptionDrone drone;

	//Sett opp grensesnitt mot de andre gruppene!

	ros::Rate loop_rate(30); //30 Hz
	while(ros::ok()) {
			
		loop_rate.sleep(); //Sørger for at while-løkken kjører med 30Hz
		ros::spinOnce(); //La ROS gjøre magien sin
	}
	return 0;
}
