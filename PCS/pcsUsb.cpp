#include "pcsUsb.h"
#include <math.h>
#include <QMessageBox>
#include <QDebug>  

#if defined(Q_OS_WIN)
#include <qt_windows.h>
#include <QtCore/qglobal.h>
#include <dbt.h>
#include <qlist.h>
#endif

#include "UsbDev/UsbDev.h"
#pragma comment(lib, "UsbDev\\Debug\\UsbDev.lib") //找默认库函数

pcsUsb::pcsUsb()
{

}

pcsUsb::~pcsUsb()
{

}

void pcsUsb::UsbCheck()
{

}

void pcsUsb::GetUsbType()
{

}

//QByteArray *MainWindow::receivedData = new QByteArray(); //接收到数据
//ThreadComPort *MainWindow::threadInitComPort = 0;
//#if defined(Q_OS_WIN)
//static const GUID GUID_DEVINTERFACE_USBSTOR = { 0xA5DCBF10L, 0x6530, 0x11D2, { 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED } };
//static const GUID InterfaceClassGuid = GUID_DEVINTERFACE_USBSTOR;
//static bool isDoingSearch = false;
//
//static void SerachComPort(){
//	if (isDoingSearch == false){
//		isDoingSearch = true;
//		bool hasDevice = false;
//		QList<QSerialPortInfo> list = QSerialPortInfo::availablePorts();
//		for (int i = 0; i<list.count(); i++){
//			if (list[i].description().contains("USB CDC", Qt::CaseInsensitive)){
//				qDebug() << "设备已经插入,端口：" << list[i].portName();
//				Globals::PortName = list[i].portName();
//				hasDevice = true;
//			}
//		}
//		if (!hasDevice){
//			qDebug() << "请插入设备";
//			Globals::PortName = "NoDevice";
//			AddControl(new NoDevice());
//			Globals::parent->InitComPort();
//		}
//		isDoingSearch = false;
//	}
//}
//LRESULT CALLBACK dw_internal_proc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
//{
//	if (message == WM_DEVICECHANGE) {
//		switch (wParam) {
//		case DBT_DEVNODES_CHANGED:
//			qDebug() << "设备插拔啦.";
//			Globals::parent->CloseComPort();
//			SerachComPort();
//			break;
//		}
//	}
//	// qDebug()<<"HWND："<<hwnd;
//	return DefWindowProc(hwnd, message, wParam, lParam);
//}
//
//
//static inline HWND WndProc(const void* userData)
//{
//	QString className = "UsbMonitor";
//	HINSTANCE hi = qWinAppInst();
//
//	WNDCLASS wc;
//	wc.style = 0;
//	wc.lpfnWndProc = dw_internal_proc;
//	wc.cbClsExtra = 0;
//	wc.cbWndExtra = 0;
//	wc.hInstance = hi;
//	wc.hIcon = 0;
//	wc.hCursor = 0;
//	wc.hbrBackground = 0;
//	wc.lpszMenuName = NULL;
//	wc.lpszClassName = reinterpret_cast<const wchar_t *>(className.utf16());
//	RegisterClass(&wc);
//
//	HWND hwnd = CreateWindow(wc.lpszClassName,    // classname
//		wc.lpszClassName,    // window name
//		0,                      // style
//		0, 0, 0, 0,            // geometry
//		0,                      // parent
//		0,                      // menu handle
//		hi,                    // application
//		0);                    // windows creation data.
//	if (hwnd) {
//		DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;
//		ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
//		NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
//		NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
//		NotificationFilter.dbcc_classguid = InterfaceClassGuid;
//		RegisterDeviceNotification(hwnd, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);
//	}
//	return hSwnd;
//}
//#endif


//char FirstDriveFromMask(ULONG unitmask)
//{
//	char i;
//
//	for (i = 0; i < 26; ++i)
//	{
//		if (unitmask & 0x1)
//			break;
//		unitmask = unitmask >> 1;
//	}
//
//	return(i + 'A');
//}
//
//LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
//{
//	PDEV_BROADCAST_HDR lpdb = (PDEV_BROADCAST_HDR)lParam;
//	PDEV_BROADCAST_VOLUME lpdbv = (PDEV_BROADCAST_VOLUME)lpdb;
//	TCHAR szMsg[80];
//	char driveName;
//
//	switch (uMsg)
//	{
//	case WM_DEVICECHANGE:
//		switch (wParam)
//		{
//		case DBT_DEVICEARRIVAL:
//			driveName = FirstDriveFromMask(lpdbv->dbcv_unitmask);
//			sprintf(szMsg, "USB Drive %c: has inserted.\n", driveName);
//			printf("%s\r\n", szMsg);
//			MessageBox(hWnd, szMsg, TEXT("WM_DEVICECHANGE"), MB_OK);
//			break;
//		case DBT_DEVICEREMOVECOMPLETE:
//			driveName = FirstDriveFromMask(lpdbv->dbcv_unitmask);
//			sprintf(szMsg, "USB Drive %c: has removed.\n", driveName);
//			printf("%s\r\n", szMsg);
//			MessageBox(hWnd, szMsg, TEXT("WM_DEVICECHANGE"), MB_OK);
//			break;
//		default:
//			;
//		}
//		break;
//	default:
//		;
//	}
//
//	return DefWindowProc(hWnd, uMsg, wParam, lParam);
//}
//
//int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
//{
//	TCHAR szClassName[] = _T("MyApp");
//	WNDCLASS wndcls = { 0 };
//	wndcls.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
//	wndcls.hCursor = (HCURSOR)LoadCursor(NULL, IDC_ARROW);
//	wndcls.hIcon = (HICON)LoadIcon(NULL, IDI_APPLICATION);
//	wndcls.lpfnWndProc = WndProc;
//	wndcls.lpszClassName = szClassName;
//	if (!RegisterClass(&wndcls))
//	{
//		printf("RegisterClass Failed!\r\n");
//		return 0;
//	}
//
//	HWND hWnd = CreateWindow(szClassName, szClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
//		CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, NULL, NULL);
//	if (NULL == hWnd)
//	{
//		printf("CreateWindow Failed!\r\n");
//		return 0;
//	}
//	ShowWindow(hWnd, SW_HIDE);
//	UpdateWindow(hWnd);
//
//	MSG msg;
//	while (GetMessage(&msg, NULL, NULL, NULL))
//	{
//		TranslateMessage(&msg);
//		DispatchMessage(&msg);
//	}
//	return 0;
//}
//

