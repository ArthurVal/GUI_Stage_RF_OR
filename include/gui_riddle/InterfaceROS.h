#ifndef INTERFACEROS_H
#define INTERFACEROS_H

#include <QtGui>
#include <QObject>

#include "nodeROSGUI.h"

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "ros/callback_queue.h"


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
		inline bool getEndThreadRos() {return endThreadRos;};


			//Main function (ros spin)
		void run();

	signals:
		void transfertInputDataToGUI(int index, 
																const QVector<double> &x_phi, 
																const QVector<double> &y_phi, 
																const QVector<double> &x_theta, 
																const QVector<double> &y_theta);

	public slots:
		void disableThread();
		void newDataFromNodeROS(int index, 
																const QVector<double> &x_phi, 
																const QVector<double> &y_phi, 
																const QVector<double> &x_theta, 
																const QVector<double> &y_theta);

	private:
			//Attributs
		ros::NodeHandle* n;
		ros::Subscriber chatter_pub_gauss;
		nodeROSGUI *nodeROS;
		int argc_;
		char** argv_;
		bool endThreadRos;

};

#endif // INTERFACEROS_H
