#include <QApplication>
#include <QtGui>

#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <pwd.h>

#include "GuiObjectDetection.h"


int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

	GuiObjectDetection IHM(argc, argv);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;

  char homediRVIZ[256];
	strcpy(homediRVIZ, homedir);
	strcat(homediRVIZ,"/.rviz/Stage_RF_OR.rviz");

	std::cout << "Setup 1 : ..." << std::endl;
	IHM.setupGUI_1(homediRVIZ);
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
