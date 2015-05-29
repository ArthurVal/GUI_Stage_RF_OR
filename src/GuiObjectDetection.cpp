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
	InterfaceROSGUI = new InterfaceROS(argc,argv);

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
	rfPlotIntensityTheta->setAxisTitle(QwtPlot::yRight, "<FONT color=#0000ff  face=Arial size=2><B>  Vertival Angle Theta (Degree)  </FONT>");
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
	tabPageVision = new QWidget();
	tabPageFusion = new QWidget();
	
		
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

	labels[1]->setText("=== TBD ===");
	labels[1]->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	labels[1]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	labels[1]->setStyleSheet("QLabel { background-color : white; color : black; }");

	labels[2]->setText("=== TBD ===");
	labels[2]->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	labels[2]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	labels[2]->setStyleSheet("QLabel { background-color : white; color : black; }");


		//Setup Vlayout

			// ---- RF Param -----
	gridBoxParamRF->addWidget(labels[0],0,0,1,5);

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

	gridBoxParamRF->addWidget(label_text[0],1,1,1,1);
	gridBoxParamRF->addWidget(label_text[1],1,2,1,1);
	gridBoxParamRF->addWidget(label_text[2],1,3,1,1);
	gridBoxParamRF->addWidget(label_text[3],1,4,1,1);

	gridBoxParamRF->addWidget(label_text[4],2,0,1,1);
	gridBoxParamRF->addWidget(label_text[5],3,0,1,1);
	
	QLineEdit *line[6];
	QDoubleValidator *validator[5];
	QIntValidator *validatorPoints;
 
		//min Phi
	line[0] = new QLineEdit("-180");
	line[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[0] = new QDoubleValidator(-180, 180, 2); //from -180 to 180 degrees with 2 decimals
	line[0]->setValidator(validator[0]);

	gridBoxParamRF->addWidget(line[0],2,1,1,1);
 
		//max Phi
	line[1] = new QLineEdit("180");
	line[1]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[1] = new QDoubleValidator(-180, 180, 2); //from -180 to 180 degrees with 2 decimals
	line[1]->setValidator(validator[1]);

	gridBoxParamRF->addWidget(line[1],2,2,1,1);
 
		//min Theta
	line[2] = new QLineEdit("0");
	line[2]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[2] = new QDoubleValidator(0, 180, 2); //from 0 to 180 degrees with 2 decimals
	line[2]->setValidator(validator[2]);

	gridBoxParamRF->addWidget(line[2],3,1,1,1);
 
		//max Theta
	line[3] = new QLineEdit("180");
	line[3]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[3] = new QDoubleValidator(0, 180, 2); //from 0 to 180 degrees with 2 decimals
	line[3]->setValidator(validator[3]);

	gridBoxParamRF->addWidget(line[3],3,2,1,1);
 
		//Duree
	line[4] = new QLineEdit("10");
	line[4]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validator[4] = new QDoubleValidator(0, 60, 2); // from 0 -> 60s 
	line[4]->setValidator(validator[4]);

	gridBoxParamRF->addWidget(line[4],2,3,1,1);
 
		//N_point
	line[5] = new QLineEdit("360");
	line[5]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	validatorPoints = new QIntValidator(0, 720); 
	line[5]->setValidator(validatorPoints);

	gridBoxParamRF->addWidget(line[5],2,4,1,1);

	QPushButton *StartButton = new QPushButton("Start RF");
	gridBoxParamRF->addWidget(StartButton,3,3,1,2);


	gridBoxParamRF->setColumnStretch(0,1);
	for(int i = 1 ; i < 5 ; ++i )		
		gridBoxParamRF->setColumnStretch(i,3);

	for(int i = 0 ; i < 4 ; ++i )		
		gridBoxParamRF->setRowStretch(i,1);

			// ---- ORK Param ---- 	
	vBoxParamVision->addWidget(labels[1]);	
			// ---- Fusion Param ---- 
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
	curveIntensity->setSamples(x_phi,y_phi);
	rfPlotIntensity->replot();

	curveIntensityTheta->setSamples(y_theta,x_theta);
	rfPlotIntensityTheta->replot();
}

