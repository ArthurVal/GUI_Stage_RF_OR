#ifndef NODEROSGUI_H
#define NODEROSGUI_H

#include <QtGui>
#include <QObject>

#include "rf_riddle/RF.h"
#include "rf_riddle/RFBase.h"
#include "rf_riddle/RFSetup.h"
#include "rf_riddle/getRFData.h"
#include "rf_riddle/setRFParam.h"


#include "ros/ros.h"
#include "ros/node_handle.h"
#include "ros/callback_queue.h"

class nodeROSGUI: public QObject
{
	Q_OBJECT
	
	public:
		//Attributs
		ros::NodeHandle* n;
		ros::Subscriber chatter_pub_gauss;
		ros::ServiceClient chatter_client_gauss;
		ros::ServiceClient chatter_client_param;
		bool isRemote;
		bool thetaDisable;
		double minTheta, maxTheta, minPhi, maxPhi, acquisitionTime;
		unsigned int nPoint;

		bool requestSend;

		//Methods
			//Constructors
		nodeROSGUI(QWidget* parent = 0);
			//Desctructors
		~nodeROSGUI() {};

			//init & end ROS
		void init_ROS(int argc, char** argv);
		void end_ROS();

			//callback ROS		
		void callback_getRFData(const rf_riddle::RF &rf_data);	

	signals:
		void newInputDataFromNodeROS(int index, 
																const QVector<double> &x_phi, 
																const QVector<double> &y_phi, 
																const QVector<double> &x_theta, 
																const QVector<double> &y_theta);

	public slots:
		void getDataRF(	const double &minP,
										const double &maxP,
										const double &minT,
										const double &maxT,
										const double &AcTime,
										const unsigned int &Npts);
		void setParamRF();
		void stateChangedIsRemote(const int &newStateRemote);
		void stateChangedThetaDisable(const int &newStateThetaDis);

	protected:

	private:

};

#endif // NODEROSGUI_H
