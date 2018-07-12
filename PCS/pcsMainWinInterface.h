#ifndef PCSMAINWININTERFACE_H
#define PCSMAINWININTERFACE_H

#include <QString>

class PCS;
class pcsWindow;

class pcsMainWinInterface
{
public:
	/* pcsMainWinInterface();
	~pcsMainWinInterface();*/
	
	//virtual PCS* getMainWindow() = 0;
	//
	//virtual pcsWindow* GetActiveVTKWindow() = 0;

	enum ConsoleMessageLevel {
		STD_CONSOLE_MESSAGE = 0,
		WRN_CONSOLE_MESSAGE = 1,
		ERR_CONSOLE_MESSAGE = 2,
	};

	//virtual void DispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) = 0;
	virtual void SwitchConsoleDisplay() = 0;
	virtual void RedrawAll() = 0;
	virtual void RefreshAll() = 0;
	virtual void EnableAll() = 0;
	virtual void DisableAll() = 0;
	virtual void DisableAllAlone(pcsWindow* view) = 0;
	//virtual void UpdateWinUI() = 0;
	virtual void BanUI(bool state) = 0;

	virtual void SetFrontView() = 0;
	virtual void SetBottomView() = 0;
	virtual void SetTopView() = 0;
	virtual void SetBackView() = 0;
	virtual void SetLeftView() = 0;
	virtual void SetRightView() = 0;
	virtual void ToggleActiveWindowCenterPerspective() = 0;
	virtual void ToggleActiveWindowCustomLight() = 0;
	virtual void ToggleActiveWindowSunLight() = 0;
	virtual void ToggleActiveWindowViewerBasePerspective() = 0;
	virtual void ZoomOnSelectedEntities() = 0;

	
};

#endif // PCSMAINWININTERFACE_H
