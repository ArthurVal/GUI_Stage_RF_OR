#ifndef INTERFACEROS_H
#define INTERFACEROS_H

#include <QtGui>
#include <QObject>

#include "nodeROSGUI.h"

#define ROS_GUI_FREQ 10

	
class InterfaceROS: public QThread
{
	Q_OBJECT
	
	public:
		//Methods
			//Constructors
		InterfaceROS(int argc, char* argv[], QWidget* parent = 0);
			//Desctructors
		~InterfaceROS() {};

			//Get Attributs
		inline bool getEndThreadRos() {return endThreadRos;}
		QObject* getNodeROSPtr() {return nodeROS;}


			//Function to connect the checkBox from Main GUI to nodeROS slots
		void connectCheckBox(QWidget* checkRemote, QWidget* checkTheta);

			//Main function (ros spin)
		void run();

	signals:
		void transfertInputDataToGUI(int index, 
																const QVector<double> &x_phi, 
																const QVector<double> &y_phi, 
																const QVector<double> &x_theta, 
																const QVector<double> &y_theta);
		void transfertStartRFToNode(	const double &minPhi,
																	const double &maxPhi,
																	const double &minTheta,
																	const double &maxTheta,
																	const double &AcTime,
																	const unsigned int &Npts,
																	const unsigned int &Nech,
																	const unsigned int &freqCLK,
																	const unsigned int &freqech);

	public slots:
		void disableThread();
		void newDataFromNodeROS(int index, 
																const QVector<double> &x_phi, 
																const QVector<double> &y_phi, 
																const QVector<double> &x_theta, 
																const QVector<double> &y_theta);

		void transfertStartRF(	const double &minPhi,
														const double &maxPhi,
														const double &minTheta,
														const double &maxTheta,
														const double &AcTime,
														const unsigned int &Npts,
														const unsigned int &Nech,
														const unsigned int &freqCLK,
														const unsigned int &freqech);

	private:
			//Attributs
		nodeROSGUI *nodeROS;
		int argc_;
		char** argv_;
		bool endThreadRos, checkBoxConnected;

};

#endif // INTERFACEROS_H
