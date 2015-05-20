#include "InterfaceROS.h"

/*=================================================================================*/
/*-----------------------		InterfaceROS::InterfaceROS()		-----------------------*/
/*=================================================================================*/

InterfaceROS::InterfaceROS(int argc, char* argv[], QWidget* parent)
: QThread(parent)
{
	if( !ros::isInitialized() ){
		ROS_INFO("[Interface ROS-GUI node] Initialization of node : Interface_ROS-GUI_node");
		ros::init(argc, argv, "Interface_ROS_GUI_node");
	}	
	n = new ros::NodeHandle;
	loop_rate = new ros::Rate(ROS_GUI_FREQ);
	endThreadRos = false;
}

/*==================================================================================*/
/*--------------------		InterfaceROS::callback_getRFData()		--------------------*/
/*==================================================================================*/

void InterfaceROS::callback_getRFData(const rf_riddle::RF &rf_data)
{
	ROS_INFO("-------------------------------------------------------------");
	ROS_INFO("[Interface ROS-GUI] Received data from rf_riddle n %d", rf_data.index);
/*
	ROS_INFO("===========================");
	ROS_INFO("Data:");

	for(int j = 0 ; j < rf_data.sizeData ; ++j)
		ROS_INFO("%f",rf_data.intensity[j]);

	ROS_INFO("===========================");
*/
	QVector<double> x,y;
	for(int i = 0; i < rf_data.sizeData ; ++i){
		x.push_back(rf_data.phi[i]);
		y.push_back(rf_data.intensity[i]);
	}
	
	emit newInputData(rf_data.index,x,y);;
}

/*==================================================================*/
/*---------------		InterfaceROS::disableThread()		----------------*/
/*==================================================================*/

void InterfaceROS::disableThread()
{
	endThreadRos = true;
}

/*==================================================================*/
/*--------------------		InterfaceROS::run()		--------------------*/
/*==================================================================*/

void InterfaceROS::run()
{
	ROS_INFO("Starting Thread Interface ROS-GUI");

	qRegisterMetaType< QVector<float> >("QVector<float>");

	ROS_INFO("Subscribing to rf_riddle_intensity_map topic.");	
	chatter_pub_gauss = n->subscribe("rf_riddle_intensity_map", 100, &InterfaceROS::callback_getRFData, this);

	if(!chatter_pub_gauss){	
		ROS_INFO("Subscribing to rf_riddle_intensity_map FAILED.");
	}

	while(ros::ok() && chatter_pub_gauss && (!endThreadRos)){	
		ros::spinOnce();
		loop_rate->sleep();
	}
	this->exec();
	ROS_INFO("[Interface ROS-GUI node] Shutdown of node : Interface_ROS-GUI_node");	
}
