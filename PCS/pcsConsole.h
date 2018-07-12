#ifndef PCSCONSOLE_H
#define PCSCONSOLE_H

//Qt
#include <QObject>
#include <QMutex>
#include <QStringList>
#include <QTimer>
#include <QVector>
#include <QPair>
//system
#include <stdio.h>
#include <string.h>

#include <pcsLogs.h>

class QListWidget;
class QWidget;
class PCS;

template<class T> struct pcsSingleton
{
	pcsSingleton() : instance(0) {}
	~pcsSingleton() 
	{ 
		release();
	}
	void release() 
	{ 
		if (instance) 
			delete instance;
		instance = 0; 
	}

	T* instance;
};


class pcsConsole : public QObject , public pcsLogs
{
	Q_OBJECT

public:
	//≥ı ºªØ ‰≥ˆ¿∏
	static void InitConsole(QListWidget* textDisplay = 0, 
							QWidget* parentWidget = 0,
							PCS* parentWindow = 0);

	static pcsConsole* TheInstance();
	static void ReleaseInstance();
	void setAutoRefresh(bool state);

public slots:
		void Refresh();

protected:
	pcsConsole();

	virtual void DisplayMessage(const QString& message, MessageLevel level);
	
	QListWidget* textDisplay;
	QWidget* parentWidget;
	PCS* parentWindow;

	typedef QPair<QString, MessageLevel> ConsoleItemType; //
	QVector<ConsoleItemType> Queue; //
	QTimer Timer;
	QMutex Mutex;
};

#endif // PCSCONSOLE_H
