#ifndef MYGUIOD_H
#define MYGUIOD_H

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
		~GuiObjectDetection() { stopInterfaceROSThread(); };

			//Get Attributs
		unsigned int getSizeLabels(){ return sizeLabels; }
		unsigned int getSizeRButtons(){ return sizeRButtons; }
		unsigned int getGUISetup(){return GUI_OK; }

			//Set Labels
		int setTextLabel(unsigned int num_label, char* text);
		int setTextRButtons(unsigned int num_rbutton, char* text);

			//Setup of the GUI
		void setupGUI_1(char* path_rviz_config_file = "~/.rviz/default.rviz");
		void startInterfaceROSThread();
		void stopInterfaceROSThread();

	public slots: 
		void updateRFData(int index, const QVector<double> &x, const QVector<double> &y);

	private:
			//Attributs
		rviz::VisualizationManager *rvizManager;
		rviz::VisualizationFrame *rvizPanel;		
		QwtPlot *rfPlotIntensity;
		QwtPlotCurve *curveIntensity;
		QRadioButton *rButtons[N_RBUTTON_MAX];
		QLabel *labels[N_LABEL_MAX];
		QVBoxLayout *vBoxParam;
		QGridLayout *mainGridBox;
		InterfaceROS *InterfaceROSGUI;
		unsigned int sizeRButtons;			
		unsigned int sizeLabels;
		unsigned int GUI_OK;	


};

#endif // MYGUIOD_H
