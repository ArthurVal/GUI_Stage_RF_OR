#include "InterfaceROS.h"

/*=================================================================================*/
/*-----------------------		InterfaceROS::InterfaceROS()		-----------------------*/
/*=================================================================================*/

InterfaceROS::InterfaceROS(int argc, char* argv[], QWidget* parent)
: QThread(parent)
{
	argc_ = argc;
	argv_ = argv;

	checkBoxConnected = false;

	nodeROS = new nodeROSGUI;	
	qRegisterMetaType< QVector<double> >("QVector<double>");

		//Ros Topic or Service input
	connect(nodeROS, 	SIGNAL(newInputDataFromNodeROS(const int&, const QVector<double>&, const QVector<double>&, const QVector<double>&, const QVector<double>&)),
				 	this, 		SLOT(newDataFromNodeROS(const int&, const QVector<double>&, const QVector<double>&, const QVector<double>&, const QVector<double>&))
					);

	connect(this, 	SIGNAL(transfertStartRFToNode(const double&,
																						const double&,
																						const double&,
																						const double&,
																						const double&,
																						const unsigned int&)),
				 nodeROS	,	SLOT(getDataRF(	const double&,
																		const double&,
																		const double&,
																		const double&,
																		const double&,
																		const unsigned int&))
					, Qt::QueuedConnection);

	nodeROS->moveToThread(this);

	endThreadRos = false;
}

/*==================================================================*/
/*---------------		InterfaceROS::disableThread()		----------------*/
/*==================================================================*/

void InterfaceROS::disableThread()
{
	endThreadRos = true;
}

/*========================================================================*/
/*---------------		InterfaceROS::newDataFromNodeROS()		----------------*/
/*========================================================================*/

void InterfaceROS::newDataFromNodeROS(int index, const QVector<double> &x_phi, const QVector<double> &y_phi, const QVector<double> &x_theta, const QVector<double> &y_theta)
{
	emit transfertInputDataToGUI(index,x_phi,y_phi,x_theta,y_theta);
}

/*========================================================================*/
/*---------------		InterfaceROS::transfertStartRF()		----------------*/
/*========================================================================*/

void InterfaceROS::transfertStartRF(	const double &minPhi,
														const double &maxPhi,
														const double &minTheta,
														const double &maxTheta,
														const double &AcTime,
														const unsigned int &Npts)
{
	emit transfertStartRFToNode(minPhi,maxPhi,minTheta,maxTheta,AcTime,Npts);
}

/*========================================================================*/
/*---------------		InterfaceROS::transfertStartRF()		----------------*/
/*========================================================================*/
void InterfaceROS::connectCheckBox(QWidget* checkRemote, QWidget* checkTheta)
{
	connect(checkRemote, SIGNAL(stateChanged(int)), nodeROS, SLOT(stateChangedIsRemote(const int&)));
	connect(checkTheta, SIGNAL(stateChanged(int)), nodeROS, SLOT(stateChangedThetaDisable(const int&)));
	checkBoxConnected = true;
}

/*==================================================================*/
/*--------------------		InterfaceROS::run()		--------------------*/
/*==================================================================*/

void InterfaceROS::run()
{

	nodeROS->init_ROS(argc_,argv_);
	//ros::spin();
	exec();
	nodeROS->end_ROS();
}
