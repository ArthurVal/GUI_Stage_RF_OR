#include "GuiObjectDetection.h"

#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <string> 
#include <iostream>

/*=============================================================================================*/
/*-----------------------		 GuiObjectDetection::GuiObjectDetection()		-----------------------*/
/*=============================================================================================*/

GuiObjectDetection::GuiObjectDetection(int argc, char* argv[], unsigned int n_label, unsigned int n_rbutton, QWidget* parent)
: QWidget(parent)
{
	mainGridBox = new QGridLayout();

		//Setup the interface ROS/Qt
	InterfaceROSGUI = new InterfaceROS(argc,argv,this);

	connect(InterfaceROSGUI, 	SIGNAL(transfertInputDataToGUI(	const int&, 	
																														const QVector<double>&, 
																														const QVector<double>&, 
																														const QVector<double>&, 
																														const QVector<double>&)),
				 	this, 						SLOT(updateRFData(const int&, 
																							const QVector<double>&, 
																							const QVector<double>&, 
																							const QVector<double>&, 
																							const QVector<double>&))
					);	

	connect(this, 	SIGNAL(startRFAcquisition(const double&,
																						const double&,
																						const double&,
																						const double&,
																						const double&,
																						const unsigned int&)),
				 	InterfaceROSGUI,	SLOT(transfertStartRF(	const double&,
																										const double&,
																										const double&,
																										const double&,
																										const double&,
																										const unsigned int&))
					);
	
		//Create Rviz panel
	rvizPanel = new rviz::VisualizationFrame();

		//Create Plotting panel for Phi angle (bottom of the screen)
	rfPlotIntensity = new QwtPlot(QwtText("Intensity map from RF sensor (Phi)"));
	rfPlotIntensity->setAxisTitle(QwtPlot::xBottom, "<FONT color=#0000ff  face=Arial size=2><B>  Horizontal Angle Phi (Degree)  </FONT>");
	rfPlotIntensity->setAxisScale(QwtPlot::xBottom, 180, -180);
	rfPlotIntensity->setAxisTitle(QwtPlot::yLeft, "<FONT color=#0000ff  face=Arial size=2><B>Intensity</FONT>");
	rfPlotIntensity->setAxisScale(QwtPlot::yLeft, 0, 1);
	rfPlotIntensity->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  //rfPlotIntensity->setAutoReplot(true);

	curveIntensity = new QwtPlotCurve("Intensity Curve");
	curveIntensity->setPen(QPen(Qt::red));
	curveIntensity->attach(rfPlotIntensity);

		//Create Plotting panel for theta angle (right of the screen)
	rfPlotIntensityTheta = new QwtPlot(QwtText("Intensity map from RF sensor (Theta)"));
	rfPlotIntensityTheta->enableAxis(QwtPlot::yRight,true);
	rfPlotIntensityTheta->enableAxis(QwtPlot::yLeft,false);
	rfPlotIntensityTheta->setAxisTitle(QwtPlot::yRight, "<FONT color=#0000ff  face=Arial size=2><B>  Vertical Angle Theta (Degree)  </FONT>");
	rfPlotIntensityTheta->setAxisScale(QwtPlot::yRight, 180, 0);
	rfPlotIntensityTheta->setAxisScale(QwtPlot::yLeft, 180, 0);
	rfPlotIntensityTheta->setAxisTitle(QwtPlot::xBottom, "<FONT color=#0000ff  face=Arial size=2><B>Intensity</FONT>");
	rfPlotIntensityTheta->setAxisScale(QwtPlot::xBottom, 1, 0);
	rfPlotIntensityTheta->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  //rfPlotIntensity->setAutoReplot(true);

	curveIntensityTheta = new QwtPlotCurve("Intensity Curve");
	curveIntensityTheta->setPen(QPen(Qt::red));
	curveIntensityTheta->attach(rfPlotIntensityTheta);



	if(n_rbutton > N_RBUTTON_MAX)
		n_rbutton = N_RBUTTON_MAX;

	sizeRButtons = n_rbutton ;

	for(int i=0 ; i<sizeRButtons-1 ; ++i){
		std::stringstream strg;		
		strg << i;
		std::string num; 
		num = strg.str();
		rButtons[i] = new QRadioButton(num.c_str());
	}

	if(n_label > N_LABEL_MAX)
		n_label = N_LABEL_MAX;

	sizeLabels = n_label ;

	for(int i=0 ; i<sizeLabels-1 ; ++i){
		std::stringstream strg;		
		strg << i;
		std::string num;
		num = strg.str();
		labels[i] = new QLabel(num.c_str());
	}
	
	gridBoxParamRF = new QGridLayout();
	vBoxParamVision = new QVBoxLayout();
	vBoxParamFusion = new QVBoxLayout();

	tabParam = new QTabWidget();

	tabPageRF = new QWidget();

	checkBoxRF[0] = new QCheckBox("Remote Control");
	checkBoxRF[1] = new QCheckBox("Theta enable");
	connect(checkBoxRF[0], SIGNAL(stateChanged(int)), this, SLOT(updateIsRemote(const int)));
	connect(checkBoxRF[1], SIGNAL(stateChanged(int)), this, SLOT(updateThetaDisable(const int)));

	tabPageVision = new QWidget();
	tabPageFusion = new QWidget();

	minTheta = 0;	
	maxTheta = 180;
	minPhi = -180;
	maxPhi = 180;

	acquisitionTime = 1;
	nPoint = 360;

	timerRF = new QTimer(this);

	isRemote = true;
	thetaDisable = true;

	timerRFtimeout = false;
	isRunningRFAcquisition = false;
		
	GUI_OK = 0;
}

/*=======================================================================================*/
/*-----------------------		 GuiObjectDetection::setTextLabel()		-----------------------*/
/*=======================================================================================*/

int GuiObjectDetection::setTextLabel(unsigned int num_label, char* text){
	if(num_label > (sizeLabels-1))
		return -1;
	
	(labels[num_label])->setText(text);
	return 0;
}

/*===========================================================================================*/
/*-----------------------		 GuiObjectDetection::setTextRButtons()		-----------------------*/
/*===========================================================================================*/

int GuiObjectDetection::setTextRButtons(unsigned int num_label, char* text){
	if(num_label > (sizeRButtons-1))
		return -1;
	
	(rButtons[num_label])->setText(text);
	return 0;
}

/*=====================================================================================*/
/*-----------------------		 GuiObjectDetection::setupGUI_1()		-----------------------*/
/*=====================================================================================*/

void GuiObjectDetection::setupGUI_1(char* path_rviz_config_file){
		//Loading Rviz with config file
	rvizPanel->initialize(path_rviz_config_file);

		//Setup Plotting panel

		//Set buttons texts
	for(int i=0 ; i<sizeRButtons-1 ; ++i){
		std::stringstream strg;
		std::string text;
		strg << "Param n " << i;
		text = strg.str();
		rButtons[i]->setText(text.c_str());
	}
		//Set Labels texts
	labels[0]->setText("RF Parameters");
	labels[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	labels[1]->setText("=== TBD (No parameters yet) ===");
	labels[1]->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	labels[1]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	labels[1]->setStyleSheet("QLabel { background-color : white; color : black; }");

	labels[2]->setText("=== TBD (No parameters yet) ===");
	labels[2]->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	labels[2]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	labels[2]->setStyleSheet("QLabel { background-color : white; color : black; }");


		//Setup tab Param

			// ---- RF Param -----
				// => Title
	gridBoxParamRF->addWidget(labels[0],0,0,1,3);
	QLabel *labelProgressBar = new QLabel();
	labelProgressBar->setText("RF Acquisiiton Progress :");
	labelProgressBar->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

				// => CheckBox Remote & Theta
	QWidget* tmpWidgetCheckBox = new QWidget();
	QVBoxLayout* tmpVBoxLayout = new QVBoxLayout();
	InterfaceROSGUI->connectCheckBox(checkBoxRF[0],checkBoxRF[1]);
	tmpVBoxLayout->addWidget(checkBoxRF[0]);
	tmpVBoxLayout->addWidget(checkBoxRF[1]);
	tmpWidgetCheckBox->setLayout(tmpVBoxLayout);
	gridBoxParamRF->addWidget(tmpWidgetCheckBox,0,3,1,2);

				// => ProgressBar
	progressBarRF = new QProgressBar();
	progressBarRF->setRange(0,1);
	progressBarRF->setValue(1);
	gridBoxParamRF->addWidget(labelProgressBar,1,0,1,3);
	gridBoxParamRF->addWidget(progressBarRF,1,3,1,2);


				// => Text for Edit
	QLabel *label_text[6];
	label_text[0] = new QLabel();
	label_text[0]->setText("min");
	label_text[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	label_text[1] = new QLabel();
	label_text[1]->setText("max");
	label_text[1]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	label_text[2] = new QLabel();
	label_text[2]->setText("Duree (s)");
	label_text[2]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	label_text[3] = new QLabel();
	label_text[3]->setText("N_Points");
	label_text[3]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	label_text[4] = new QLabel();	
	label_text[4]->setText(QChar(0x03C6)); //phi
	label_text[4]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	label_text[5] = new QLabel();
	label_text[5]->setText(QChar(0x03B8)); //Theta
	label_text[5]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	gridBoxParamRF->addWidget(label_text[0],2,1,1,1);
	gridBoxParamRF->addWidget(label_text[1],2,2,1,1);
	gridBoxParamRF->addWidget(label_text[2],2,3,1,1);
	gridBoxParamRF->addWidget(label_text[3],2,4,1,1);

	gridBoxParamRF->addWidget(label_text[4],3,0,1,1);
	gridBoxParamRF->addWidget(label_text[5],4,0,1,1);



				// => Edit windows with parameters & validators	
	QDoubleValidator *validator[5];
	QIntValidator *validatorPoints;
 
		//min Phi
	line[0] = new QLineEdit("-180");
	line[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[0] = new QDoubleValidator(-180, 180, 2); //from -180 to 180 degrees with 2 decimals
	line[0]->setValidator(validator[0]);
	connect(line[0], SIGNAL(editingFinished()), this, SLOT(updateMinPhi()));

	gridBoxParamRF->addWidget(line[0],3,1,1,1);
 
		//max Phi
	line[1] = new QLineEdit("180");
	line[1]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[1] = new QDoubleValidator(-180, 180, 2); //from -180 to 180 degrees with 2 decimals
	line[1]->setValidator(validator[1]);
	connect(line[1], SIGNAL(editingFinished()), this, SLOT(updateMaxPhi()));

	gridBoxParamRF->addWidget(line[1],3,2,1,1);
 
		//min Theta
	line[2] = new QLineEdit("0");
	line[2]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[2] = new QDoubleValidator(0, 180, 2); //from 0 to 180 degrees with 2 decimals
	line[2]->setValidator(validator[2]);
	connect(line[2], SIGNAL(editingFinished()), this, SLOT(updateMinTheta()));

	gridBoxParamRF->addWidget(line[2],4,1,1,1);
 
		//max Theta
	line[3] = new QLineEdit("180");
	line[3]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[3] = new QDoubleValidator(0, 180, 2); //from 0 to 180 degrees with 2 decimals
	line[3]->setValidator(validator[3]);
	connect(line[3], SIGNAL(editingFinished()), this, SLOT(updateMaxTheta()));

	gridBoxParamRF->addWidget(line[3],4,2,1,1);
 
		//Duree
	line[4] = new QLineEdit("1");
	line[4]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[4] = new QDoubleValidator(0, 60, 2); // from 0 -> 60s 
	line[4]->setValidator(validator[4]);
	connect(line[4], SIGNAL(editingFinished()), this, SLOT(updateAcTime()));

	gridBoxParamRF->addWidget(line[4],3,3,1,1);
 
		//N_point
	line[5] = new QLineEdit("360");
	line[5]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validatorPoints = new QIntValidator(0, 720); 
	line[5]->setValidator(validatorPoints);
	connect(line[5], SIGNAL(editingFinished()), this, SLOT(updateNPoints()));

	gridBoxParamRF->addWidget(line[5],3,4,1,1);


				// => Start Button
	StartRFButton = new QPushButton("Start RF");
	connect( StartRFButton, SIGNAL(clicked()), this, SLOT(startAcquisition()) );

	gridBoxParamRF->addWidget(StartRFButton,4,3,1,2);

	gridBoxParamRF->setColumnStretch(0,1);
	for(int i = 1 ; i < 5 ; ++i )		
		gridBoxParamRF->setColumnStretch(i,3);

	for(int i = 0 ; i < 4 ; ++i )		
		gridBoxParamRF->setRowStretch(i,1);

		//Set Timer for singleshot
	timerRF->setSingleShot(true);
	connect(timerRF,SIGNAL(timeout()),this,SLOT(timerRFTimeout()));


			// ---- ORK Param ---- TODO	
	vBoxParamVision->addWidget(labels[1]);	
			// ---- Fusion Param ---- TODO
	vBoxParamFusion->addWidget(labels[2]);	

		//Setup Tab Widget
	tabPageRF->setLayout(gridBoxParamRF);
	tabPageVision->setLayout(vBoxParamVision);
	tabPageFusion->setLayout(vBoxParamFusion);

	tabParam->addTab(tabPageRF, "RF");
	tabParam->addTab(tabPageVision, "Vision");
	tabParam->addTab(tabPageFusion, "Fusion");

	mainGridBox->setColumnMinimumWidth(1,2);
	mainGridBox->setColumnMinimumWidth(0,2);
	mainGridBox->setRowMinimumHeight(1,2);
	mainGridBox->setRowMinimumHeight(0,2);

	mainGridBox->setColumnStretch(0,4);
	mainGridBox->setColumnStretch(1,1);
	mainGridBox->setRowStretch(0,4);
	mainGridBox->setRowStretch(1,1);

		//Setup Main Layout
	mainGridBox->addWidget(rvizPanel,0,0,1,1);
	mainGridBox->addWidget(rfPlotIntensityTheta,0,1,1,1);
	mainGridBox->addWidget(rfPlotIntensity,1,0,1,1);
	mainGridBox->addWidget(tabParam,1,1,1,1);

	//rfPlotIntensity->updateCanvasMargins();

	this->setLayout(mainGridBox);	

	checkBoxRF[0]->setCheckState(Qt::Checked);
	checkBoxRF[1]->setCheckState(Qt::Unchecked);

	GUI_OK = 1;
}

/*---------------------------------------------------------------------------------------*/
/*--------------------------------       SLOT      --------------------------------------*/
/*---------------------------------------------------------------------------------------*/

/*=======================================================================================*/
/*-----------		SLOT : GuiObjectDetection::startInterfaceROSThread()		-----------------*/
/*=======================================================================================*/

void GuiObjectDetection::startInterfaceROSThread()
{
	std::cout << "[GUI RIDDLE] Starting Thread Interface ROS"<< std::endl;
	InterfaceROSGUI->start();
	std::cout << "[GUI RIDDLE] Starting Thread Interface ROS: DONE"<< std::endl;
}

void GuiObjectDetection::stopInterfaceROSThread()
{
	
	std::cout << "[GUI RIDDLE] Stoping Thread Interface ROS:"<< std::endl;
	InterfaceROSGUI->disableThread();
	InterfaceROSGUI->quit();
	InterfaceROSGUI->wait();
	std::cout << "[GUI RIDDLE] Stoping Thread Interface ROS: DONE"<< std::endl;
}


/*=======================================================================================*/
/*-------------------		SLOT : GuiObjectDetection::updateRFData()		---------------------*/
/*=======================================================================================*/

void GuiObjectDetection::updateRFData(int index, const QVector<double> &x_phi, const QVector<double> &y_phi, const QVector<double> &x_theta, const QVector<double> &y_theta)
{
	this->runningRFActivity(false);

	curveIntensity->setSamples(x_phi,y_phi);
	rfPlotIntensity->replot();

	curveIntensityTheta->setSamples(y_theta,x_theta);
	rfPlotIntensityTheta->replot();
}

/*=======================================================================================*/
/*-------------------		SLOT : GuiObjectDetection::runningRFActivity()		---------------------*/
/*=======================================================================================*/

void GuiObjectDetection::runningRFActivity(const bool &isRunning)
{
	if(isRunning && !timerRFtimeout){ //Ask for running & no timer active (normal start activity)
		isRunningRFAcquisition = true;
		checkBoxRF[0]->setEnabled(false);
		checkBoxRF[1]->setEnabled(false);
		StartRFButton->setEnabled(false);
		progressBarRF->setRange(0,0);
		timerRF->start((int)(2 * acquisitionTime*1000));

	}else{

		if(!isRunning && !timerRFtimeout){ //Ask for stop running & timer active (data received)
			isRunningRFAcquisition = false;
			checkBoxRF[0]->setEnabled(true);
			checkBoxRF[1]->setEnabled(true);
			StartRFButton->setEnabled(true);
			progressBarRF->setRange(0,1);
			progressBarRF->setValue(1);
			timerRF->stop();

		}else{

			if(!isRunning && timerRFtimeout){ //Ask for stop running & timer timeout (=timeout)
				isRunningRFAcquisition = false;
				checkBoxRF[0]->setEnabled(true);
				checkBoxRF[1]->setEnabled(true);
				StartRFButton->setEnabled(true);
				progressBarRF->setRange(0,1);
				progressBarRF->setValue(0);
			
			}else{//Ask for running & timer timeout (impossible) => restart timer
				isRunningRFAcquisition = true;
				checkBoxRF[0]->setEnabled(false);
				checkBoxRF[1]->setEnabled(false);
				StartRFButton->setEnabled(false);
				progressBarRF->setRange(0,0);
				timerRF->start((int)(2 * acquisitionTime*1000));
			}
		}
	}


}

/*=======================================================================================*/
/*-------------------		SLOT : GuiObjectDetection::timerRFTimeout()		---------------------*/
/*=======================================================================================*/

void GuiObjectDetection::timerRFTimeout()
{
	if(isRunningRFAcquisition){
		timerRFtimeout = true;
		this->runningRFActivity(false);
		timerRFtimeout = false;
	}
}

/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateMinTheta()		-----------------*/
/*=============================================================================*/

void GuiObjectDetection::updateMinTheta()
{
	QString minT = line[2]->text();
 	double Theta = minT.toDouble();
	if(Theta < maxTheta){
		minTheta = Theta;
	}else{
		if(Theta > maxTheta){
			maxTheta = Theta;
				//Put theta in max theta for auto-update		
			line[3]->setText(minT);
				//rewrite min theta in lineEdit
			QString str;
			str.setNum(minTheta);
			line[2]->setText(str);
		}else{ //Theta = maxTheta 
			//rewrite min theta in lineEdit (do nothing)
			QString str;
			str.setNum(minTheta);
			line[2]->setText(str);
		}
	}

	//std::cout << "[GUI RIDDLE] [DEBUG] Slot GUI -> Theta : "<< Theta << "  ->minTheta : " << minTheta << std::endl;	
}

/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateMaxTheta()		-----------------*/
/*=============================================================================*/

void GuiObjectDetection::updateMaxTheta()
{
	QString maxT = line[3]->text();
 	double Theta = maxT.toDouble();
	if(Theta > minTheta){
		maxTheta = Theta;
	}else{
		if(Theta < minTheta){
			minTheta = Theta;
				//Put theta in min theta for auto-update		
			line[2]->setText(maxT);
				//rewrite max theta in lineEdit
			QString str;
			str.setNum(maxTheta);
			line[3]->setText(str);
		}else{ //Theta = minTheta 
			//rewrite max theta in lineEdit (do nothing)
			QString str;
			str.setNum(maxTheta);
			line[3]->setText(str);
		}
	}

	//std::cout << "[GUI RIDDLE] [DEBUG] Slot GUI -> Theta : "<< Theta << "  ->maxTheta : " << maxTheta << std::endl;	
}


/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateMinPhi()		-----------------*/
/*=============================================================================*/

void GuiObjectDetection::updateMinPhi()
{
	QString minP = line[0]->text();
 	double Phi = minP.toDouble();
	if(Phi < maxPhi){
		minPhi = Phi;
	}else{
		if(Phi > maxPhi){
			maxPhi = Phi;
				//Put theta in max theta for auto-update		
			line[1]->setText(minP);
				//rewrite min theta in lineEdit
			QString str;
			str.setNum(minPhi);
			line[0]->setText(str);
		}else{ //Theta = maxTheta 
			//rewrite min theta in lineEdit (do nothing)
			QString str;
			str.setNum(minPhi);
			line[0]->setText(str);
		}
	}

	//std::cout << "[GUI RIDDLE] [DEBUG] Slot GUI -> Phi : "<< Phi << "  ->minPhi : " << minPhi << std::endl;	
}

/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateMaxPhi()		-----------------*/
/*=============================================================================*/

void GuiObjectDetection::updateMaxPhi()
{
	QString maxP = line[1]->text();
 	double Phi = maxP.toDouble();
	if(Phi > minPhi){
		maxPhi = Phi;
	}else{
		if(Phi < minPhi){
			minPhi = Phi;
				//Put theta in min theta for auto-update		
			line[0]->setText(maxP);
				//rewrite max theta in lineEdit
			QString str;
			str.setNum(maxPhi);
			line[1]->setText(str);
		}else{ //Theta = minTheta 
			//rewrite max theta in lineEdit (do nothing)
			QString str;
			str.setNum(maxPhi);
			line[1]->setText(str);
		}
	}

	//std::cout << "[GUI RIDDLE] [DEBUG] Slot GUI -> Phi : "<< Phi << "  ->maxPhi : " << maxPhi << std::endl;	
}

/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateAcTime()		-----------------*/
/*=============================================================================*/

void GuiObjectDetection::updateAcTime()
{
	QString ATime = line[4]->text();
 	acquisitionTime = ATime.toDouble();
	//std::cout << "[GUI RIDDLE] [DEBUG] Slot GUI -> acquisitionTime : "<< acquisitionTime << std::endl;	
}

/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateNPoints()		-----------------*/
/*=============================================================================*/

void GuiObjectDetection::updateNPoints()
{
	QString NPts = line[5]->text();
 	nPoint = NPts.toInt();
	//std::cout << "[GUI RIDDLE] [DEBUG] Slot GUI -> nPoint : "<< nPoint << std::endl;	
}

/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateIsRemote()		-----------------*/
/*=============================================================================*/
void GuiObjectDetection::updateIsRemote(const int &stateRemote)
{	
	switch(stateRemote){
		case 0: //Unchecked
			isRemote = false;
			StartRFButton->setText("Update Params");
		break;

		case 1: //PartiallyChecked
			isRemote = false;
			StartRFButton->setText("Update Params");
		break;

		case 2: //Checked
			isRemote = true;
			StartRFButton->setText("Start RF");
		break;

		default:
			isRemote = false;
			StartRFButton->setText("Update Params");
	}
}

/*=================================================================================*/
/*-----------		SLOT : GuiObjectDetection::updateThetaDisable()		-----------------*/
/*=================================================================================*/
void GuiObjectDetection::updateThetaDisable(const int &stateThetaDis)
{
	switch(stateThetaDis){
		case 0: //Unchecked
			thetaDisable = true;
		break;

		case 1: //PartiallyChecked
			thetaDisable = true;
		break;

		case 2: //Checked
			thetaDisable = false;
		break;

		default:
			thetaDisable = true;
	}	
}
/*=============================================================================*/
/*-----------		SLOT : GuiObjectDetection::startAcquisition()		---------------*/
/*=============================================================================*/

void GuiObjectDetection::startAcquisition()
{
	std::cout << "[GUI RIDDLE] [DEBUG] Bouton clicked" << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] Data transmitted" << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] minPhi = " << minPhi << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] maxPhi = " << maxPhi << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] minTheta = " << minTheta << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] maxTheta = " << maxTheta << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] acquisitionTime = " << acquisitionTime << std::endl;
	std::cout << "[GUI RIDDLE] [DEBUG] nPoint = " << nPoint << std::endl;

	if(isRemote)
		this->runningRFActivity(true);

	emit startRFAcquisition(minPhi,maxPhi,minTheta,maxTheta,acquisitionTime,nPoint);
}
