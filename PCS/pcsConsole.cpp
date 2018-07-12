#include "pcsConsole.h"

#include "pcs.h"

//Qt
#include <QListWidget>
#include <QMessageBox>
#include <QApplication>
#include <QColor>
#include <QTime>
//system
#include <assert.h>

static pcsSingleton<pcsConsole> s_console;

pcsConsole::pcsConsole()
	: textDisplay(0)
	, parentWidget(0)
	, parentWindow(0)
{
	
}

pcsConsole* pcsConsole::TheInstance()
{
	if (!s_console.instance)
	{
		s_console.instance = new pcsConsole();
		pcsLogs::RegisterInstance(s_console.instance);
	}

	return s_console.instance;
}

void pcsConsole::ReleaseInstance()
{
	s_console.release();
	pcsLogs::RegisterInstance(0);
}

void pcsConsole::InitConsole(QListWidget* textDisplay,
								QWidget* parentWidget,
								PCS* parentWindow)
{
	pcsConsole* console = TheInstance();
	assert(console);
	console->textDisplay = textDisplay;
	console->parentWidget = parentWidget;
	console->parentWindow = parentWindow;
	if (textDisplay)
		console->setAutoRefresh(true);
}

//自行更新输出框
void pcsConsole::setAutoRefresh(bool state)
{
	if (state)
	{
		QObject::connect(&Timer, SIGNAL(timeout()), this, SLOT(refresh()));
		Timer.start(1000);
	}
	else
	{
		Timer.stop();
		QObject::disconnect(&Timer, SIGNAL(timeout()), this, SLOT(refresh()));
	}
}

void pcsConsole::Refresh()
{
	Mutex.lock();
	assert(textDisplay);
	if (textDisplay && Queue.isEmpty())
	{
		for (QVector<ConsoleItemType>::const_iterator it = Queue.begin(); it != Queue.end(); ++it)
		{
			QListWidgetItem* item = new QListWidgetItem(it->first); //first = message text

			//输出消息的颜色 
			Qt::GlobalColor color = Qt::black;
			switch (it->second) //第二消息程度
			{
				case LOG_Standard:
					color = Qt::black;
					break;
	#ifdef _DEBUG
				case LOG_Standard_Debug:
	#endif
					color = Qt::blue;
					break;
				case LOG_Warning:
	#ifdef _DEBUG
				case LOG_Warning_Debug:
	#endif
					color = Qt::darkRed;
					//关联主界面警告框
					if (parentWindow)
						//parentWindow->SwitchConsoleDisplay();
					break;
				case LOG_Error:
	#ifdef _DEBUG
				case LOG_Error_Debug:
	#endif
					color = Qt::red;
					break;
				default:
	#ifndef _DEBUG
					//skip debug message in debug mode
					continue;
	#else
					//we shoudn't fall here in debug mode!
					assert(false);
					break;
	#endif
			}
			item->setForeground(color);

			textDisplay->addItem(item);
		}

		textDisplay->scrollToBottom();
	}

	Queue.clear();

	Mutex.unlock();
}

void pcsConsole::DisplayMessage(const QString& message, MessageLevel level)
{
	QString formatedMessage = QString("[") + QTime::currentTime().toString() + QString("] ") + message;

	if (textDisplay)
	{
		Mutex.lock();
		Queue.push_back(ConsoleItemType(formatedMessage, level));
		Mutex.unlock();
	}
#ifdef _DEBUG
	else
	{
		switch (level)
		{
		case LOG_Standard:
			printf("MSG: ");
			break;
		case LOG_Standard_Debug:
			printf("MSG-DBG: ");
			break;
		case LOG_Warning:
			printf("WARNING: ");
			break;
		case LOG_Warning_Debug:
			printf("WARNING-DBG: ");
			break;
		case LOG_Error:
			printf("ERROR: ");
			break;
		case LOG_Error_Debug:
			printf("ERROR-DBG: ");
			break;
		}
		printf(" %s\n", qPrintable(formatedMessage));
	}
#endif

	if (parentWidget && (level == LOG_Error || level == LOG_Error_Debug))
	{
		//we display error message in a popup dialog
		QMessageBox::warning(parentWidget, "Error", message);
	}
}
