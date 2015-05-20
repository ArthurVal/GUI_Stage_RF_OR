#ifndef INTERFACEROS_H
#define INTERFACEROS_H

#include <QtGui>
#include <QObject>

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "rf_riddle/RF.h"

#define ROS_GUI_FREQ 1

	
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

	public slots:
		void disableThread();

	signals:
		void newInputData(int index, const QVector<double> &x, const QVector<double> &y);

	private:
			//Attributs
		ros::NodeHandle* n;
		ros::Subscriber chatter_pub_gauss;
		ros::Rate *loop_rate;
		bool endThreadRos;

			//callback ROS		
		void callback_getRFData(const rf_riddle::RF &rf_data);
};

#endif // INTERFACEROS_H
