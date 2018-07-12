#include "pcs.h"
#include <QtWidgets/QApplication>

#include "OnPCS.h"
#include "pcsTimer.h"
#include "pcsCommandLineHandle.h"
//QT
#include <QApplication>
#include <QSplashScreen>
#include <QPixmap>
#include <QMessageBox>
#include <QLocale>
#include <QTime>
#include <QMouseEvent>
#include <QApplication>  
#include <QSplashScreen>  
#include <QPixmap>  
#include <QDebug>  
#include <QDateTime>
	
//class pcsApplication : public QApplication
//{
//public:
//	pcsApplication(int &argc, char **argv)
//		: QApplication(argc, argv)
//	{
//		setOrganizationName("PCS");
//		setApplicationName("PiontCloudSoft");
//	};
//
//}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PCS w;

	OnPCS pcs;
	bool pcsView;
	pcs.setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);

	//QSplashScreen* splash = 0;
	//QTime SplashStartTime;
	//bool CommandLine = (argc>1  && argv[1][0] == '-');
	//if (!CommandLine)
	//{
	//	if (!pcsView)
	//	{
	//		QMessageBox::critical(0, "Error", "This application needs VTK to run!");
	//		return EXIT_FAILURE;
	//	}
	//	SplashStartTime.start();
	//	QPixmap pixmap(QString::fromUtf8("‪:PCS\Resources\png\soft.jpg"));
	//	splash = new QSplashScreen(pixmap, Qt::WindowStaysOnTopHint);
	//	splash->show();
	//	QApplication::processEvents();
	//}

	//PCS::ResetUniqueIDCount();
	//pcsTimer::Init();

	//int result = 0;
	///*命令行处理*/
	//if (CommandLine)
	//{
	//	result = pcsCommandLineHandle::Parse(argc, argv);
	//}
	//else
	//{
	//	PCS* pcs = PCS::TheInstance();
	//	if (!pcs)
	//	{
	//		QMessageBox::critical(0, "Error", "Failed to initialize the main application window?!");
	//		return EXIT_FAILURE;
	//	}
	//	pcs->show();
	//	QApplication::processEvents();

	//	if (argc > 1)
	//	{
	//		if (splash)
	//			splash->close();

	//		QStringList filenames;
	//		for (int i = 1; i<argc; ++i)
	//			filenames << QString(argv[i]);

	//		pcs->addPoint(filenames, UNKNOWN_FILE);
	//	}

	//	if (splash)
	//	{
	//		while (SplashStartTime.elapsed() < 1000)
	//		{
	//			splash->raise();
	//		}

	//		delete splash;
	//		splash = 0;
	//	}

	//	try
	//	{
	//		result = a.exec();
	//	}
	//	catch (...)
	//	{
	//		QMessageBox::warning(0, "PCS crashed!", "Hum, it seems that PCS has crashed... Sorry about that :)");
	//	}
	//}

	//PCS::DestroyInstance();

	//return result;
	#pragma region 延迟启动时间
	QDateTime n = QDateTime::currentDateTime();
	QDateTime now;
	do{
		now = QDateTime::currentDateTime();
		pcs.show();
		a.processEvents();
	} while (n.secsTo(now) <= 1);//数字为需要延时的秒数   
#pragma endregion

	pcs.close();
	w.setMouseTracking(true);

	w.show();
	return a.exec();
}