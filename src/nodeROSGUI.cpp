#include "nodeROSGUI.h"

nodeROSGUI::nodeROSGUI(QWidget* parent)
: QObject(parent)
{
}

/*==================================================================================*/
/*--------------------		nodeROSGUI::callback_getRFData()		--------------------*/
/*==================================================================================*/

void nodeROSGUI::callback_getRFData(const rf_riddle::RF &rf_data)
{
//	ROS_INFO("-------------------------------------------------------------");
//	ROS_INFO("[Interface ROS-GUI] Received data from rf_riddle n %d", rf_data.index);
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
	
	emit newInputDataFromNodeROS(rf_data.index,x,y);
}
