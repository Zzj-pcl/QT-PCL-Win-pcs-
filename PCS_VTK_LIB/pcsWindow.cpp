#include "pcsWindow.h"

#include "pcsLogs.h"
//QT
#include <QtGui>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QSettings>
#include <QApplication>
//System
#include <string.h>
#include <math.h>
#include <algorithm>
#include <QSystemDetection.h>

#ifdef USE_Vld
//VLD
#include <vld.h>
#endif

/***************VTK组件 (智能指针)***************/
vtkSmartPointer<vtkOrientationMarkerWidget> Coor3D = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkPropAssembly> assembly = vtkSmartPointer<vtkPropAssembly>::New();
vtkSmartPointer<vtkInteractorStyleTrackballCamera> CameraStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

vtkRenderWindowInteractor *iren;

/***********VTK zoom大小模式**********/
const float PCS_MAX_ZOOM_RATIO = 1.0e6f;
const float PCS_MIN_ZOOM_RATIO = 1.0e-6f;

/***********可视化窗体设置***********/
static const char pcs_groupName[] = "pcsWindow";
static const char pcs_perspectiveView[] = "perspectiveView";
static const char pcs_objectMode[] = "objectCenteredView";
static const char pcs_sunLight[] = "sunLightEnabled";
static const char pcs_customLight[] = "customLightEnabled";
static const char pcs_pivotVisibility[] = "pivotVisibility";

const GLuint VTK_INVALID_LIST_ID = (~0); 
static int WindowNumber = 1; //窗体索引

pcsWindow::pcsWindow(QWidget *parent,
	bool silentInitialization)
	:QVTKWidget(parent)
{
	setWindowTitle(QString("3D View %1").arg(WindowNumber));

	setStyleSheet("QWidget{ backgroundcolor:black }");

	
	//自动加载以前的透视设置
	{
		QSettings settings;
		settings.beginGroup(pcs_groupName);

		//加载组件
		bool perspectiveView = settings.value(pcs_perspectiveView, false).toBool();
		bool objectCenteredView = settings.value(pcs_objectMode, true).toBool();
		/*	m_sunLightEnabled = settings.value(pcs_sunLight, true).toBool();
		m_customLightEnabled = settings.value(pcs_customLight, false).toBool();
		int pivotVisibility = settings.value(pcs_pivotVisibility, PIVOT_SHOW_ON_MOVE).toInt();*/
		settings.endGroup();

		//输出框显示当前视角
		if (!pcs_silentInitialization)
		{
			if (!perspectiveView)
				pcsLogs::Print("[pcsWindow] report current perspective");
			else
				pcsLogs::Print(QString("[pcsWindow] report current perspective").arg(objectCenteredView ? "object-centered" : "viewer-centered"));
		}
		
		//pivot visibility
		/*switch (pivotVisibility)
		{
			case PIVOT_HIDE:
				setPivotVisibility(PIVOT_HIDE);
				break;
			case PIVOT_SHOW_ON_MOVE:
				setPivotVisibility(PIVOT_SHOW_ON_MOVE);
				break;
			case PIVOT_ALWAYS_SHOW:
				setPivotVisibility(PIVOT_ALWAYS_SHOW);
				break;
		}*/
		//apply saved parameters
		/*setPerspectiveState(perspectiveView, objectCenteredView);
		if (pcs_customLightEnabled)
			displayNewMessage("Warning: custom light is ON", pcsWindow::LOWER_LEFT_MESSAGE, false, 2, CUSTOM_LIGHT_STATE_MESSAGE);
		if (!pcs_sunLightEnabled)
			displayNewMessage("Warning: sun light is OFF", pcsWindow::LOWER_LEFT_MESSAGE, false, 2, SUN_LIGHT_STATE_MESSAGE);*/
	}
	/*connect(this, &pcsWindow::itemPickedFast, this, &pcsWindow::onItemPickedFast, Qt::DirectConnection);
	connect(&pcs_scheduleTimer, &QTimer::timeout, this, &ccGLWindow::checkScheduledRedraw);
	connect(&pcs_autoRefreshTimer, &QTimer::timeout, this, [=]() {
		update();
	});

	setAcceptDrops(true);
	setAttribute(Qt::WA_AcceptTouchEvents, true);
	setAttribute(Qt::WA_OpaquePaintEvent, true);*/

}

pcsWindow::~pcsWindow()
{
	
}

void pcsWindow::DisplayNewMessage(const QString& message,
	MessagePosition pos,
	bool append,
	int displayMaxDelay_sec,
	MessageType type)
{
	if (message.isEmpty())
	{
		if (!append)
		{
			std::list<MessageToDisplay>::iterator it = pcs_messagesToDisplay.begin();
			while (it != pcs_messagesToDisplay.end())
			{
				if (it->position == pos)
					it = pcs_messagesToDisplay.erase(it);
				else
					++it;
			}
		}
	}
	else
	{
		pcsLogs::Warning("[pcsWindow::displayNewMessage] Appending an empty message has no effect!");
	}
	return;

	//如已存在 则更换信息
	if (!append)
	{
		if (type != Custom_Message)
		{
			for (std::list<MessageToDisplay>::iterator it = pcs_messagesToDisplay.begin(); it != pcs_messagesToDisplay.end();)
			{
				if (it->type == type)
					it = pcs_messagesToDisplay.erase(it);
				else
					++it;
			}
		}
		else
		{
			pcsLogs::WarningDebug("[pcsWindow::displayNewMessage] Append is forced for custom messages!");
		}
	}
	else
	{
		if (pos == Screen_Center_Message)
		{
			pcsLogs::Warning("[pcsWindow::displayNewMessage] Append is not supported for center screen messages!");
			append = false;
		}
	}

	MessageToDisplay mess;
	mess.message = message;
	//mess.messageValidity_sec = pcsTimer::Sec() + displayMaxDelay_sec;
	mess.position = pos;
	mess.type = type;
	pcs_messagesToDisplay.push_back(mess);

	pcsLogs::Print(QString("[displayNewMessage] New message valid until %1 s.").arg(mess.messageValidity_sec));
}

void pcsWindow::InitialVTK()
{
	if (pcs_silentInitialization)
		return;
	
}

bool pcsWindow::InitVTKMessage()
{
	/*if (!)
	{
		pcsLogs::Warning("[Glew] An error occurred while initializing VTK extensions!");
		return false;
	}
	else
	{
		pcsLogs::Print("[Glew] Initialized!");
		return true;
	}*/
	return false;
}

void pcsWindow::toBeRefreshed()
{
	pcs_shouldBeRefreshed = true;

	invalidateViewport();
}

void pcsWindow::refresh()
{
	if (pcs_shouldBeRefreshed && isVisible())
	{
		Redraw();
	}
		
}

void pcsWindow::invalidateViewport()
{
	pcs_validProjectionMatrix = false;
	pcs_updatefbo = true;
}



bool pcsWindow::supportVTKVersion(unsigned VTKVersionFlag)
{
	return ( VTKVersionFlag );
}

QFont pcsWindow::getTextDisplayFont() const
{
	if (!pcs_captureMode.enabled || pcs_captureMode.zoomFactor == 1.0f)
		return pcs_font;

	QFont font = pcs_font;
	font.setPointSize(static_cast<int>(pcs_font.pointSize() * pcs_captureMode.zoomFactor));
	return font;
}

void pcsWindow::WinCoordinate3D()
{
	//axesActor->SetShaftTypeToCylinder();//可以设置每个坐标轴成圆柱或者用户自定义，这里定义成直线
	//axesActor->SetXAxisLabelText("x");
	//axesActor->SetYAxisLabelText("y");
	//axesActor->SetZAxisLabelText("z");///设置每个坐标轴上的文字
	//axesActor->SetTotalLength(3, 3, 3);//设置坐标轴的长度

	//axesActor->GetXAxisCaptionActor2D()->GetProperty()->SetColor(1, 0, 0); //修改X字体颜色为红色  
	//axesActor->GetYAxisCaptionActor2D()->GetProperty()->SetColor(0, 2, 0); //修改Y字体颜色为绿色  
	//axesActor->GetZAxisCaptionActor2D()->GetProperty()->SetColor(0, 0, 3); //修改Z字体颜色为蓝色  

	//renderWindow->AddRenderer(renderer);
	//renderWindowInteractor->SetRenderWindow(renderWindow);
	//renderer->SetBackground(.2 , .3, .4);
	//transform->Translate(1.0, 0.0, 0.0);
	//renderer->AddActor(axesActor);
	//axesActor->SetUserTransform(transform);
	//Coor3D->SetOutlineColor(0.93, 0.57, 0.13);
	//Coor3D->SetOrientationMarker(axesActor);
	//Coor3D->SetInteractor(renderWindowInteractor);
	//Coor3D->SetViewport(0, 0, 0.3, 0.3);
	//Coor3D->SetEnabled(1);
	//Coor3D->InteractiveOn();
	//renderer->ResetCamera();
	//renderWindow->Render();
	//renderWindowInteractor->Start();
}

void pcsWindow::SetBackgroudColor()
{
	pcs_winColor.setBrush(this->backgroundRole(), QBrush(QColor(0, 0, 0)));
	this->setPalette(pcs_winColor);
}

void pcsWindow::Redraw()
{
	pcs_updatefbo = true;
	update();
	updateGeometry();
	mRenWin->StereoUpdate();
	renderWindow->StereoUpdate();
}

void pcsWindow::ZoomGlobal()
{
	SetZoom(1.0f);
	Redraw();
}

void pcsWindow::SetSunLight(bool state)
{
	pcs_sunLightEnabled = state;
	DisplayNewMessage(state ? "Sun light ON" : "Sun light OFF",
						pcsWindow::Lower_Left_Message,
						false,
						2,
						Sun_Light_State_Message);
	Redraw();
	
	{
		QSettings settings;
		settings.beginGroup(pcs_groupName);
		settings.setValue(pcs_sunLight, pcs_sunLightEnabled);
	}
}

void pcsWindow::ToggleSunLight()
{
	SetSunLight(!pcs_sunLightEnabled);
}

void pcsWindow::SetCustomLight(bool state)
{
	pcs_customLightEnabled = state;
	DisplayNewMessage(state ? "Custom light ON" : "Custom light OFF",
		pcsWindow::Lower_Left_Message,
		false,
		2,
		Sun_Light_State_Message);

	invalidateViewport();
	Redraw();

	//save parameter
	{
		QSettings settings;
		settings.beginGroup(pcs_groupName);
		settings.setValue(pcs_customLight, pcs_customLightEnabled);
	}
}

void pcsWindow::ToggleCustomLight()
{
	SetCustomLight(!pcs_customLightEnabled);
}

//Framerate 
static const qint64 Framerate_Duration_Msec = 10000;
static const unsigned Framerate_Min_Frames = 50;
static bool pcs_frameRateTestInProgress = false;
static QTimer pcs_frameRateTimer;
static QElapsedTimer pcs_frameRateElapsedTimer;
static qint64 pcs_frameRateElapsedTime_ms = 0; //i.e. not initialized
static unsigned pcs_frameRateCurrentFrame = 0;

void pcsWindow::FrameRate()
{
	if (pcs_frameRateTestInProgress)
	{
		pcsLogs::Error("Framerate test already in progress!");
		return;
	}
	pcs_frameRateTestInProgress = true;

	connect(&pcs_frameRateTimer, SIGNAL(timeout()), this, SLOT(redraw()), Qt::QueuedConnection);

	DisplayNewMessage("[Framerate test in progress]",
		pcsWindow::Upper_Center_Message,
		true,
		3600);

	pcs_frameRateCurrentFrame = 0;
	pcs_frameRateElapsedTime_ms = 0;
	pcs_frameRateElapsedTimer.start();
	pcs_frameRateTimer.start(0);
};

void pcsWindow::SetZoom(float value)
{
	if (value < PCS_MIN_ZOOM_RATIO)
		value = PCS_MIN_ZOOM_RATIO;
	else if (value > PCS_MAX_ZOOM_RATIO)
		value = PCS_MAX_ZOOM_RATIO;
}

QString pcsWindow::getShadersPath()
{
#if defined(Q_OS_WIN)
	// shaders are in the bundle
	QString  path = QCoreApplication::applicationDirPath();
	path.remove("WINDOWS");
	return path + "Shaders";
#else
	return QApplication::applicationDirPath() + "/shaders";
#endif
}