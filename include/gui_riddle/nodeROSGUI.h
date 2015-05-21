#ifndef NODEROSGUI_H
#define NODEROSGUI_H

#include <QtGui>
#include <QObject>
#include "rf_riddle/RF.h"
	
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
		void newInputDataFromNodeROS(int index, const QVector<double> &x, const QVector<double> &y);

	protected:

	private:

};

#endif // NODEROSGUI_H
