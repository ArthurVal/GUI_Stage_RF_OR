#include "nodeROSGUI.h"

nodeROSGUI::nodeROSGUI(QWidget* parent)
: QObject(parent)
{
	isRemote = false;
	thetaDisable = true;

	requestSend = false;

	minTheta = 0;	
	maxTheta = 180;
	minPhi = -180;
	maxPhi = 180;

	acquisitionTime = 1;
	nPoint = 360;
}
/*======================================================================*/
/*--------------------		nodeROSGUI::init_ROS()		--------------------*/
/*======================================================================*/

void nodeROSGUI::init_ROS(int argc, char** argv){
	ROS_INFO("Starting Thread Interface ROS-GUI");	
	ROS_INFO("[Interface ROS-GUI node] Initialization of node : Interface_ROS-GUI_node");
	ros::init(argc, argv, "Interface_ROS_GUI_node");
  
	n = new ros::NodeHandle;
	
	chatter_client_gauss = n->serviceClient<rf_riddle::getRFData>("rf_riddle_intensity_map_srv");
	if(chatter_client_gauss){
		ROS_INFO("Connection to rf_riddle_intensity_map_srv Service (getRFData).");
	}

	chatter_client_param = n->serviceClient<rf_riddle::setRFParam>("rf_riddle_set_param");
	if(chatter_client_param){
		ROS_INFO("Connection to rf_riddle_set_param Service (setRFParam).");
	}

	ROS_INFO("Subscribing to rf_riddle_intensity_map_topic topic.");	
	chatter_pub_gauss = n->subscribe("rf_riddle_intensity_map_topic", 100, &nodeROSGUI::callback_getRFData, this);
	if(!chatter_pub_gauss){	
		ROS_INFO("Subscribing to rf_riddle_intensity_map FAILED.");
	}
	
}

/*======================================================================*/
/*--------------------		nodeROSGUI::end_ROS()		--------------------*/
/*======================================================================*/

void nodeROSGUI::end_ROS(){
	delete n;
	ROS_INFO("[Interface ROS-GUI node] Shutdown of node : Interface_ROS-GUI_node");
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


/*---------------------------------------------------------------------------------------*/
/*--------------------------------       SLOT      --------------------------------------*/
/*---------------------------------------------------------------------------------------*/

/*=================================================================*/
/*-----------		SLOT : nodeROSGUI::getDataRF()		-----------------*/
/*=================================================================*/
void nodeROSGUI::getDataRF(	const double &minP,
														const double &maxP,
														const double &minT,
														const double &maxT,
														const double &AcTime,
														const unsigned int &Npts)
{
	
	//ROS_INFO("[Interface ROS-GUI node] Slot called");

	while(requestSend){};


	minTheta = minT;	
	maxTheta = maxT;
	minPhi = minP;
	maxPhi = maxP;

	acquisitionTime = AcTime;
	nPoint = Npts;

	if(!isRemote){

		setParamRF();

	}else{
		requestSend = true;

		rf_riddle::getRFData getRFSrv;
	
		getRFSrv.request.rfSetupNeeded.thetaMin = minTheta;
		getRFSrv.request.rfSetupNeeded.thetaMax = maxTheta;
		getRFSrv.request.rfSetupNeeded.phiMin = minPhi;
		getRFSrv.request.rfSetupNeeded.phiMax = maxPhi;
		getRFSrv.request.rfSetupNeeded.acquisitionTime = acquisitionTime;
		getRFSrv.request.rfSetupNeeded.nPoints = nPoint;

		if(chatter_client_gauss.call(getRFSrv)){
			ROS_INFO("[Interface ROS-GUI node] Service Call succeed.");/*
			ROS_INFO("[Interface ROS-GUI node] Setup send =");		
			ROS_INFO("[Interface ROS-GUI node] ThetaMin = %f", minTheta);		
			ROS_INFO("[Interface ROS-GUI node] ThetaMax = %f", maxTheta);			
			ROS_INFO("[Interface ROS-GUI node] PhiMin = %f", minPhi);			
			ROS_INFO("[Interface ROS-GUI node] PhiMax = %f", maxPhi);			
			ROS_INFO("[Interface ROS-GUI node] Acquisition time = %f", acquisitionTime);			
			ROS_INFO("[Interface ROS-GUI node] N_Points = %d", nPoint);	*/
			this->callback_getRFData(getRFSrv.response.RFOutput);
		}else{		
			ROS_INFO("[Interface ROS-GUI node] Service Call failed /!\\ : ");
			ROS_INFO("[Interface ROS-GUI node] -> Be sure that the rf_riddle_node has been started with --remote args ");
			ROS_INFO("[Interface ROS-GUI node] -> Check services with \"rosservice list\" command on a terminal");
			ROS_INFO("[Interface ROS-GUI node] \t-> rf_riddle_intensity_map_srv service = service to get RF data (Input = Setup / Output = RF data obtained)");
		}
		requestSend = false;

	}//isRemote
				
}
/*=================================================================*/
/*-----------		SLOT : nodeROSGUI::setParamRF()		-----------------*/
/*=================================================================*/
void nodeROSGUI::setParamRF()
{
	while(requestSend){};
					
	rf_riddle::setRFParam setRFParamSrv;
	
	setRFParamSrv.request.rfSetupNeeded.thetaMin = minTheta;
	setRFParamSrv.request.rfSetupNeeded.thetaMax = maxTheta;
	setRFParamSrv.request.rfSetupNeeded.phiMin = minPhi;
	setRFParamSrv.request.rfSetupNeeded.phiMax = maxPhi;
	setRFParamSrv.request.rfSetupNeeded.acquisitionTime = acquisitionTime;
	setRFParamSrv.request.rfSetupNeeded.nPoints = nPoint;
	setRFParamSrv.request.remote = isRemote;
	setRFParamSrv.request.thetaDis = thetaDisable;

	if(chatter_client_param.call(setRFParamSrv)){
		ROS_INFO("[Interface ROS-GUI node] Service Call succeed.");/*
		ROS_INFO("[Interface ROS-GUI node] Setup send =");		
		ROS_INFO("[Interface ROS-GUI node] ThetaMin = %f", minTheta);		
		ROS_INFO("[Interface ROS-GUI node] ThetaMax = %f", maxTheta);			
		ROS_INFO("[Interface ROS-GUI node] PhiMin = %f", minPhi);			
		ROS_INFO("[Interface ROS-GUI node] PhiMax = %f", maxPhi);			
		ROS_INFO("[Interface ROS-GUI node] Acquisition time = %f", acquisitionTime);			
		ROS_INFO("[Interface ROS-GUI node] N_Points = %d", nPoint);	*/
	}else{		
		ROS_INFO("[Interface ROS-GUI node] Service Call failed /!\\ : ");
		ROS_INFO("[Interface ROS-GUI node] -> Be sure that the rf_riddle_node has been started with args ");
		ROS_INFO("[Interface ROS-GUI node] -> Check services with \"rosservice list\" command on a terminal");
		ROS_INFO("[Interface ROS-GUI node] \t-> rf_riddle_set_param service = service to set RF parameters (Input = Setup (angle max/min, time, nPoints, remote & theta) / Output = none");		
	}
					
}

/*===========================================================================*/
/*-----------		SLOT : nodeROSGUI::stateChangedIsRemote()		-----------------*/
/*===========================================================================*/
void nodeROSGUI::stateChangedIsRemote(const int &newStateRemote){
	switch(newStateRemote){
		case 0: //Unchecked			
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] isRemote change call : false");
			isRemote = false;
		break;

		case 1: //PartiallyChecked
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] isRemote change call : false");
			isRemote = false;
		break;

		case 2: //Checked
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] isRemote change call : true");
			isRemote = true;
		break;

		default:
			isRemote = false;
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] isRemote change call : false");
	}
	this->setParamRF();
}

/*===============================================================================*/
/*-----------		SLOT : nodeROSGUI::stateChangedThetaDisable()		-----------------*/
/*===============================================================================*/
void nodeROSGUI::stateChangedThetaDisable(const int &newStateThetaDis){
	switch(newStateThetaDis){
		case 0: //Unchecked
			thetaDisable = true;
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] thetaDis change call : true");
		break;

		case 1: //PartiallyChecked
			thetaDisable = true;
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] thetaDis change call : true");
		break;

		case 2: //Checked
			thetaDisable = false;
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] thetaDis change call : false");
		break;

		default:
			thetaDisable = true;
			ROS_INFO("[Interface ROS-GUI node] [DEBUG] thetaDis change call : true");
	}
	this->setParamRF();
}


