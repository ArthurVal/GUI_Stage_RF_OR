#ifndef MYGUIOD_H
#define MYGUIOD_H

#include <cstdlib>

#include <QApplication>
#include <QtGui>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>

#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "InterfaceROS.h"

#define N_LABEL_MAX 6
#define N_RBUTTON_MAX 5

	
class GuiObjectDetection: public QWidget
{
	Q_OBJECT
	QThread GUIThread;
	
	public:
		//Methods
			//Constructors
		GuiObjectDetection(int argc, char* argv[], unsigned int n_label = N_LABEL_MAX,unsigned int n_rbutton = N_RBUTTON_MAX, QWidget* parent = 0);
			//Desctructors
		inline ~GuiObjectDetection() { 	stopInterfaceROSThread();};

			//Get Attributs
		unsigned int getSizeLabels(){ return sizeLabels; }
		unsigned int getSizeRButtons(){ return sizeRButtons; }
		unsigned int getGUISetup(){return GUI_OK; }

			//Set Labels
		int setTextLabel(unsigned int num_label, char* text);
		int setTextRButtons(unsigned int num_rbutton, char* text);

			//Setup of the GUI
		void setupGUI_1(char* path_rviz_config_file = (char*)".rviz/default.rviz");
		void startInterfaceROSThread();
		void stopInterfaceROSThread();

	signals:
		void startRFAcquisition(	const double &minP,
															const double &maxP,
															const double &minT,
															const double &maxT,
															const double &AcTime,
															const unsigned int &Npts,
															const unsigned int &Nech,
															const unsigned int &freqTSCLK,
															const unsigned int &freqech);
		

	public slots: 
			//Input (RF data)
		void updateRFData(int index,  
											const QVector<double> &x_phi, 
											const QVector<double> &y_phi, 
											const QVector<double> &x_theta, 
											const QVector<double> &y_theta);


			//Internal (Setup RF data QlineEdit interface)

				//Slot to set RF progress bar in "running" mod and disable checkboxs etc ...
		void runningRFActivity(const bool &isRunning);
		void timerRFTimeout();

		void updateMinTheta();
		void updateMaxTheta();

		void updateMinPhi();
		void updateMaxPhi();

		void updateAcTime();
		void updateNPoints();
		void updateNEchantillons();  
		void updateFreqEchantillons();  
		void updateFreqTSCLK();  

		void updateIsRemote(const int &stateRemote); 
		void updateThetaDisable(const int &stateThetaDis); 

			//Output (QPushButton activation => send signal to ROS Service)
		void startAcquisition(); 


	private:
			//Attributs
		rviz::VisualizationFrame *rvizPanel;
		
		QwtPlot *rfPlotIntensity;
		QwtPlotCurve *curveIntensity;
		
		QwtPlot *rfPlotIntensityTheta;
		QwtPlotCurve *curveIntensityTheta;

		QRadioButton *rButtons[N_RBUTTON_MAX];
		QLabel *labels[N_LABEL_MAX];

		QGridLayout *gridBoxParamRF;
		QVBoxLayout *vBoxParamVision;
		QVBoxLayout *vBoxParamFusion;
		
			//Members of RF Parameters window
		QTabWidget *tabParam;
		QWidget *tabPageRF;
		QLineEdit *line[8];
		QPushButton *StartRFButton;
		QProgressBar* progressBarRF;
		QCheckBox* checkBoxRF[2];
		QTimer* timerRF;
		
			//Members of Vision Parameters window
		QWidget *tabPageVision;
		
			//Members of Fusion Parameters window
		QWidget *tabPageFusion;
	
		QGridLayout *mainGridBox;

		InterfaceROS *InterfaceROSGUI;

		unsigned int sizeRButtons;			
		unsigned int sizeLabels;
		unsigned int GUI_OK;
	
		double minTheta, maxTheta, minPhi, maxPhi, acquisitionTime;
		unsigned int nPoint, freqTSCLK, freqEch, nEch;
		bool isRemote;
		bool thetaDisable;
		bool isRunningRFAcquisition;
		bool timerRFtimeout;

};

#endif // MYGUIOD_H
