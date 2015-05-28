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
	
	vBoxParam = new QVBoxLayout();
	
		
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
	labels[0]->setText("===========================\n=====  WORK IN PROGRESS =====\n===========================");
	labels[0]->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	labels[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	labels[0]->setStyleSheet("QLabel { background-color : white; color : black; }");

		//Setup Vlayout
	vBoxParam->addWidget(labels[0]);

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
	mainGridBox->addLayout(vBoxParam,1,1,1,1);

	//rfPlotIntensity->updateCanvasMargins();

	this->setLayout(mainGridBox);

	GUI_OK = 1;
}

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

