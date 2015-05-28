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
	QVector<double> x_phi,y_phi;
	QVector<double> x_theta,y_theta;

	for (int k = 0 ; k < 2 ; ++k){ 	//For Phi & Theta

			//If angleID == true -> Phi else (angleID == false) -> Theta
		if(rf_data.rfData[k].angleID){
				//Phi
			for(int i = 0; i < rf_data.rfData[k].sizeData ; ++i){
				x_phi.push_back(rf_data.rfData[k].angle[i]);
				y_phi.push_back(rf_data.rfData[k].intensity[i]);
			}

		}else{ //rf_data.rfData.angleID == false
				//Theta
			for(int i = 0; i < rf_data.rfData[k].sizeData ; ++i){
				x_theta.push_back(rf_data.rfData[k].angle[i]);
				y_theta.push_back(rf_data.rfData[k].intensity[i]);
			}
		}
	}
	emit newInputDataFromNodeROS(rf_data.index,x_phi,y_phi,x_theta,y_theta);
}
