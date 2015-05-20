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
		//Setup the interface ROS/Qt
	InterfaceROSGUI = new InterfaceROS(argc,argv);
	connect(InterfaceROSGUI, 	SIGNAL(newInputData(const int&, const QVector<double>&, const QVector<double>&)),
				 	this, 						SLOT(updateRFData(const int&, const QVector<double>&, const QVector<double>&))
					);

	

		//Create Rviz panel
	rvizPanel = new rviz::VisualizationFrame;

		//Create Plotting panel
	rfPlotIntensity = new QwtPlot(QwtText("Intensity map from RF sensor"), parent);
	rfPlotIntensity->setAxisTitle(QwtPlot::xBottom, "Angle en Degree");
	rfPlotIntensity->setAxisScale(QwtPlot::xBottom, 180, -180);
	rfPlotIntensity->setAxisTitle(QwtPlot::yLeft, "<FONT color=#0000ff  face=Arial size=4><B>  Detected Intensity  </FONT>");
	rfPlotIntensity->setAxisScale(QwtPlot::yLeft, 0, 1);
  rfPlotIntensity->setAutoReplot(true);

	curveIntensity = new QwtPlotCurve("Intensity Curve");
	curveIntensity->setPen(QPen(Qt::red));
	curveIntensity->attach(rfPlotIntensity);
	

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
	
	vBoxParam = new QVBoxLayout;
	mainGridBox = new QGridLayout;
		
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

		//Setup Main Layout
	mainGridBox->addWidget(rvizPanel,0,0,1,1);
	mainGridBox->addLayout(vBoxParam,0,1,2,1);
	mainGridBox->addWidget(rfPlotIntensity,1,0,1,1);

	mainGridBox->setColumnStretch(0,5);
	mainGridBox->setColumnStretch(1,1);
	mainGridBox->setRowStretch(0,5);
	mainGridBox->setRowStretch(1,1);

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

void GuiObjectDetection::updateRFData(int index, const QVector<double> &x, const QVector<double> &y)
{
	curveIntensity->setSamples(x,y);
	rfPlotIntensity->replot();
}

