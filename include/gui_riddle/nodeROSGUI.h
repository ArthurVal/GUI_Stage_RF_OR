#ifndef NODEROSGUI_H
#define NODEROSGUI_H

#include <QtGui>
#include <QObject>
#include "rf_riddle/RF.h"
#include "rf_riddle/RFBase.h"
	
class nodeROSGUI: public QObject
{
	Q_OBJECT
	
	public:
		//Methods
			//Constructors
		nodeROSGUI(QWidget* parent = 0);
			//Desctructors
		~nodeROSGUI() {};

			//callback ROS		
		void callback_getRFData(const rf_riddle::RF &rf_data);

	signals:
		void newInputDataFromNodeROS(int index, 
																const QVector<double> &x_phi, 
																const QVector<double> &y_phi, 
																const QVector<double> &x_theta, 
																const QVector<double> &y_theta);

	protected:

	private:

};

#endif // NODEROSGUI_H
