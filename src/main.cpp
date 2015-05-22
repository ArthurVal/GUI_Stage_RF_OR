#include <QApplication>
#include <QtGui>

#include <unistd.h>
#include <iostream>

#include "GuiObjectDetection.h"


int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

	GuiObjectDetection IHM(argc, argv);

	std::cout << "Setup 1 : ..." << std::endl;
	IHM.setupGUI_1((char*)"/home/avalient/.rviz/Stage_RF_OR.rviz");
	if(IHM.getGUISetup() > 0){
		std::cout << "Setup 1 : DONE" << std::endl;
	}else{		
		std::cout << "Setup 1 : FAIL => Shutdown GUI"<< std::endl;
		return -1;
	}

	IHM.startInterfaceROSThread();
	IHM.show();

	return app.exec();
}
