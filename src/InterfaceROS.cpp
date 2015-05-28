#include "InterfaceROS.h"

/*=================================================================================*/
/*-----------------------		InterfaceROS::InterfaceROS()		-----------------------*/
/*=================================================================================*/

InterfaceROS::InterfaceROS(int argc, char* argv[], QWidget* parent)
: QThread(parent)
{
	argc_ = argc;
	argv_ = argv;

	nodeROS = new nodeROSGUI;
	nodeROS->moveToThread(this);	
	qRegisterMetaType< QVector<double> >("QVector<double>");
	connect(nodeROS, 	SIGNAL(newInputDataFromNodeROS(const int&, const QVector<double>&, const QVector<double>&, const QVector<double>&, const QVector<double>&)),
				 	this, 		SLOT(newDataFromNodeROS(const int&, const QVector<double>&, const QVector<double>&, const QVector<double>&, const QVector<double>&))
					);

	endThreadRos = false;
}

/*==================================================================*/
/*---------------		InterfaceROS::disableThread()		----------------*/
/*==================================================================*/

void InterfaceROS::disableThread()
{
	endThreadRos = true;
}
void InterfaceROS::newDataFromNodeROS(int index, const QVector<double> &x_phi, const QVector<double> &y_phi, const QVector<double> &x_theta, const QVector<double> &y_theta)
{
	emit transfertInputDataToGUI(index,x_phi,y_phi,x_theta,y_theta);
}
/*==================================================================*/
/*--------------------		InterfaceROS::run()		--------------------*/
/*==================================================================*/

void InterfaceROS::run()
{
	ROS_INFO("Starting Thread Interface ROS-GUI");	
	ROS_INFO("[Interface ROS-GUI node] Initialization of node : Interface_ROS-GUI_node");
	ros::init(argc_, argv_, "Interface_ROS_GUI_node");
  
	n = new ros::NodeHandle;

	ROS_INFO("Subscribing to rf_riddle_intensity_map topic.");	
	chatter_pub_gauss = n->subscribe("rf_riddle_intensity_map", 100, &nodeROSGUI::callback_getRFData, nodeROS);

	if(!chatter_pub_gauss){	
		ROS_INFO("Subscribing to rf_riddle_intensity_map FAILED.");
	}
	
	//ros::spin();
	this->exec();

	ROS_INFO("[Interface ROS-GUI node] Shutdown of node : Interface_ROS-GUI_node");
	delete n;	
}
