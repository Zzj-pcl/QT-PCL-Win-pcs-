#include "pcs.h"

#include "pcsPersistentSet.h"
#include "pcsPointCloud.h"
#include "pcsVersion.h"
#include "pcsConsole.h"
#include "pcsUsb.h"
#include "ui_aboutDlg.h"


//QT
#include <QGLFormat>
#include <QSettings>
#include <QtGui>
#include <QMdiArea>
#include <QtGui>
#include <QMdiArea>
#include <QSignalMapper>
#include <QMdiSubWindow>
#include <QLCDNumber>
#include <QFileDialog>
#include <QActionGroup>
#include <QProcess>
#include <QMessageBox>
#include <QElapsedTimer>
#include <QInputDialog>
#include <QTextStream>
#include <QColorDialog>

//System
#include <assert.h>
#include <iostream>
#include <random>

static PCS* pcs_instance = nullptr;

static const QString pcs_uniqueIDKey("UniqueID");
static const QString pcs_allFilesFilter("All (*.*)");
static const QString pcs_fileFilterSeparator(";;");

enum PickingOperation {
	NO_PICKING_OPERATION,
	PICKING_ROTATION_CENTER,
	PICKING_LEVEL_POINTS,
};
static pcsWindow* pcs_pickingWindow = nullptr;
static PickingOperation pcs_currentPickingOperation = NO_PICKING_OPERATION;
static pcsPointCloud* pcs_levelMarkersCloud = nullptr;


PCS::PCS(QWidget *parent)
	: QMainWindow(parent)
{
	setupUi(this);
	QSettings set;
	restoreGeometry(set.value(pcsPS::pcsMainWindow()).toByteArray());
	setWindowTitle(QString("PointCloudSoft v") + pcsVersion::GetPCSVersion(true));
	 
	pcsConsole::InitConsole(Console, this, this);
	
	pcs_WindowMapper = new QSignalMapper(this);
	connect(pcs_WindowMapper, SIGNAL(mapped(QWidget*)), this, SLOT(setActiveSubWindow(QWidget*)));

	ConnectActions();
	showMaximized();

	QMainWindow::statusBar()->showMessage(QString("Ready"));
	pcsConsole::Print("PointCloudSoft started!");

	restoreState(set.value(pcsPS::pcsMainWindow()).toByteArray());

	//pc.cloud.reset(new PointCloudT);
	//pc.cloud->resize(1);
	//viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	//viewer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
	//qvtkWidget->update();
	//viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
	
}

PCS::~PCS()
{
	assert( pcs_MdiArea && pcs_WindowMapper);
	/*pcs_MdiArea->disconnect();
	pcs_WindowMapper->disconnect();*/

	//if (pcs_MdiArea)
	//{
	//	QList<QMdiSubWindow*> subWindowList = pcs_MdiArea->subWindowList();
	//	for (int i = 0; i<subWindowList.size(); ++i)
	//		static_cast<QVTKWidget*>(subWindowList[i]->widget())->setSceneDB(0);
	//}

	/*while (!m_mdiDialogs.empty())
	{
		m_mdiDialogs.back().dialog->disconnect();
		m_mdiDialogs.back().dialog->stop(false);
		m_mdiDialogs.back().dialog->setParent(0);
		delete m_mdiDialogs.back().dialog;
		m_mdiDialogs.pop_back();
	}*/

	pcs_MdiArea->closeAllSubWindows();

	pcsConsole::ReleaseInstance();
}

void PCS::ConnectActions()
{
	/*** MAIN MENU  主界面***/
	//文件
	connect(actionOpen, SIGNAL(triggered()), this, SLOT(Load()));
	//3D
	//connect(ui.menu3D, SIGNAL(aboutToShow()), this, SLOT(UpdateMenu3D()));
	connect(actionNew3DView, SIGNAL(triggered()), this, SLOT(New3DView()));
	//帮助
	//connect(actionHelp, SIGNAL(triggered()), this, SLOT(Help()));
	connect(actionAbout, SIGNAL(triggered()), this, SLOT(About()));
	//connect(actionAboutPlugins, SIGNAL(triggered()), this, SLOT(aboutPlugins()));

	//数据读取
	connect(actionUSBread, SIGNAL(triggered()), this, SLOT(OpenUsbRead()));
}

//void PCS::ShowPointCloudAdd()
//{
//	for (int i = 0; i != clouds_vec.size(); i++)
//	{
//		viewer->addPointCloud(clouds_vec[i].cloud, "cloud" + QString::number(i).toStdString());
//		viewer->updatePointCloud(clouds_vec[i].cloud, "cloud" + QString::number(i).toStdString());
//	}
//	viewer->resetCamera();
//	qvtkWidget->update();
//}

//PCS* PCS::TheInstance()
//{
//	if (!pcs_instance)
//		pcs_instance = new PCS();
//	return pcs_instance;
//}

//pcsWindow* PCS::GetActiveVTKWindow()
//{
//	return TheInstance()->getActiveVTKWindow();
//}

//pcsWindow* PCS::GetVTKWindow(const QString& title)
//{
//	QList<QMdiSubWindow *> windows = TheInstance()->pcs_MdiArea->subWindowList();
//	int winNum = windows.size();
//
//	if (winNum == 0)
//		return 0;
//
//	for (int i = 0; i<winNum; ++i)
//	{
//		pcsWindow* win = static_cast<pcsWindow*>(windows.at(i)->widget());
//		if (win->windowTitle() == title)
//			return win;
//	}
//
//	return 0;
//}

//void PCS::GetVTKWindows(std::vector<pcsWindow*>& vtkWindows)
//{
//	QList<QMdiSubWindow*> windows = TheInstance()->pcs_MdiArea->subWindowList();
//	int winNum = windows.size();
//
//	if (winNum == 0)
//		return;
//
//	vtkWindows.clear();
//	vtkWindows.reserve(winNum);
//
//	for (int i = 0; i < winNum; ++i)
//	{
//		vtkWindows.push_back(static_cast<pcsWindow*>(windows.at(i)->widget()));
//	}
//	
//}

//void PCS::RefreshAllVTKWindow()
//{
//	TheInstance()->RefreshAll();
//}

//void PCS::UpdateWinUI()
//{
//	TheInstance()->UpdateUI();
//}

//void PCS::CloseInstance()
//{
//	if (pcs_instance)
//		delete pcs_instance;
//	else
//		return ;
//
//	pcs_instance = 0;
//}

//pcsWindow *PCS::getActiveVTKWindow()
//{
//	if (!pcs_MdiArea)
//		return 0;
//
//	QMdiSubWindow *activeSubWindow = pcs_MdiArea->activeSubWindow();
//	if (activeSubWindow)
//		return static_cast<pcsWindow*>(activeSubWindow->widget());
//	else
//	{
//		QList<QMdiSubWindow*> subWindowList = pcs_MdiArea->subWindowList();
//		if (!subWindowList.isEmpty())
//			return static_cast<pcsWindow*>(subWindowList[0]->widget());
//	}
//
//	return 0;
//}

//void PCS::DispToConsole(QString message, ConsoleMessageLevel level)
//{
//	switch (level)
//	{
//		case STD_CONSOLE_MESSAGE:
//			pcsConsole::Print(message);
//			break;
//		case WRN_CONSOLE_MESSAGE:
//			pcsConsole::Warning(message);
//			break;
//		case ERR_CONSOLE_MESSAGE:
//			pcsConsole::Error(message);
//			break;
//	}
//}

//void PCS::ForceConsoleDisplay()
//{
//	//if the console is hidden, we autoamtically display it!
//	if (dockConsole && dockConsole->isHidden())
//	{
//		dockConsole->show();
//		QApplication::processEvents();
//	}
//}

//打开点云文件
void PCS::Load()
{
	QSettings set;
	set.beginGroup(pcsPS::Load());
	QString currentPath = set.value(pcsPS::CurrentPath(), QApplication::applicationDirPath()).toString();
	int currentOpenDlgFilter = set.value(pcsPS::SelectedInputFilter(), LAS).toInt();
			
	QString filter;
	filter = filter.append(QString(PCS_File_Type_Filters[UNKNOWN_FILE]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[LAS]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[ASCII]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[DXF]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[DGN]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[PDS]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[PDMS]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[PCD]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[PLY]) + ";;");
	filter = filter.append(QString(PCS_File_Type_Filters[OBJ]));
	
	QString selectedFilter = PCS_File_Type_Filters[currentOpenDlgFilter];
			
	QStringList selectFile = QFileDialog::getOpenFileNames(
		this,
		QString::fromLocal8Bit("选择文件"),
		currentPath,
		filter,
		&selectedFilter
#ifdef _DEBUG
		, QFileDialog::DontUseNativeDialog
#endif
		);
	
		if (selectFile.isEmpty())
		{
			QMessageBox message(QMessageBox::Warning, "Title", QString::fromLocal8Bit("选择的文件不存在！"));
			return;
		}
			
		PCS_File_Types fType;
		if ( fType = UNKNOWN_FILE)
		{
			for (unsigned i = 0; i < static_cast<unsigned>(FILE_TYPES_COUNT); ++i)
			{
				if (selectedFilter == QString(PCS_File_Type_Filters[i]))
				{
					fType = PCS_File_Types_Enums[i];							
					break;
				}
			}
		}		

		for (int i = 0; i < selectFile.size(); ++i)
		{
			QString filename = selectFile[i];
			if (selectedFilter.endsWith("PCD Point Cloud Library cloud (*.pcd)", Qt::CaseInsensitive))
			{
				pcl::io::loadPCDFile(filename.toStdString(), *(pc.cloud));
				clouds_vec.push_back(pc);
			}
			
		}

		currentPath = QFileInfo(selectFile[0]).absolutePath();
		currentOpenDlgFilter = fType;
		set.setValue(pcsPS::CurrentPath(), currentPath);
		set.setValue(pcsPS::SelectedInputFilter(), currentOpenDlgFilter);
		set.endGroup();


	//QSettings set;
	//set.beginGroup(pcsPS::Load());
	//QString currentPath = set.value(pcsPS::CurrentPath(), QApplication::applicationDirPath()).toString();
	//QStringList selectFiles = QFileDialog::getOpenFileNames(this,
	//	QString::fromLocal8Bit("选择文件"),
	//	currentPath,
	//	tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));
	//if (selectFiles.isEmpty())
	//	return;
	//clouds_vec.clear();
	//pc.cloud.reset(new PointCloudT);
	//for (int i = 0; i != selectFiles.size(); i++)
	//{
	//	pc.cloud.reset(new PointCloudT);
	//	QString filename = selectFiles[i];
	//	std::string file_name = filename.toStdString();
	//	int status = -1;
	//	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
	//	{
	//		status=pcl::io::loadPCDFile(file_name, *(pc.cloud));
	//		if (pc.cloud->points[0].r == 0 && pc.cloud->points[0].g == 0 && pc.cloud->points[0].b == 0)
	//		{
	//			SetCloudColor(255, 255, 255);
	//		}
	//	}	
	//	//setA(255); 
	//	clouds_vec.push_back(pc);  //将点云导入点云容器
	//}
	//ShowPointCloudAdd();
}

//void PCS::SetCloudColor(unsigned int r, unsigned int g, unsigned int b)
//{
//	for (size_t i = 0; i < pc.cloud->size(); i++)
//	{
//		pc.cloud->points[i].r = r;
//		pc.cloud->points[i].g = g;
//		pc.cloud->points[i].b = b;
//		pc.cloud->points[i].a = 255;
//	}
//}

//void PCS::AddPoint(const QStringList& filenames, PCS_File_Types fType , pcsWindow* View)
//{
//	Eigen::Vector3d LoadCoordinateShift(0, 0, 0);
//	bool LoadCoordinateTransEnable = false;
//
//	Eigen::Vector3d AddCoordinateShift(0, 0, 0);
//
//	for (int i = 0; i < filenames.size(); ++i)
//	{
//
//	}
//}


QMdiSubWindow* PCS::getMdiSubWindow(QVTKWidget* win)
{
	QList<QMdiSubWindow*> subWindowList = pcs_MdiArea->subWindowList();
	for (int i = 0; i<subWindowList.size(); ++i)
		if (static_cast<QVTKWidget*>(subWindowList[i]->widget()) == win)
			return subWindowList[i];

	return 0;
}


pcsWindow* PCS::New3DView()
{
	assert(pcs_MdiArea);
	pcs_MdiArea = new QMdiArea(this);
	setCentralWidget(pcs_MdiArea);
	QList<QMdiSubWindow*> subWindowList = pcs_MdiArea->subWindowList();
	pcsWindow* otherView = 0;

	if (!subWindowList.isEmpty())
	{
		otherView = static_cast<pcsWindow*>(subWindowList[0]->widget());

	}
	
	if (subWindowList.isEmpty())
	{	
		//QGLFormat format = QGLFormat::defaultFormat();
		//format.setStencil(false);
		//format.setSwapInterval(0);		
		pcsWindow *vtkView = new pcsWindow(this, otherView);
		vtkView->setMinimumSize(400, 400);
		vtkView->resize(400, 400);
		QPalette p;
		p.setBrush(vtkView->backgroundRole(), QBrush(QColor(0, 0, 0)));
		vtkView->setPalette(p);
		vtkView->setBackgroundRole(QPalette::ColorRole(0));
		vtkView->setAutoFillBackground(true);
		vtkView->SetRenderWindow(viewer->getRenderWindow());
		viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
		viewer->setupInteractor(vtkView->GetInteractor(), vtkView->GetRenderWindow());
		vtkView->update();
		pcs_MdiArea->addSubWindow(vtkView);

		connect(vtkView, SIGNAL(EntitySelectionChanged(int)), dockDBtree, SLOT(selectEntity(int)));
		connect(vtkView, SIGNAL(EntitiesSelectionChanged(std::set<int>)), dockDBtree, SLOT(selectEntities(std::set<int>)));

		/*connect(vtkView, SIGNAL(MouseWheelRotated(float)), this, SLOT(echoMouseWheelRotate(float)));
		connect(vtkView, SIGNAL(CameraDisplaced(float, float)), this, SLOT(echoCameraDisplaced(float, float)));
		connect(vtkView, SIGNAL(CameraPosChanged(const pcsVector3D&)), this, SLOT(echoCameraPosChanged(const pcsVector3D&)));
		connect(vtkView, SIGNAL(PivotPointChanged(const pcsVector3D&)), this, SLOT(echoPivotPointChanged(const pcsVector3D&)));
		connect(vtkView, SIGNAL(PixelSizeChanged(float)), this, SLOT(echoPixelSizeChanged(float)));*/
		//connect(vtkView, SIGNAL(destroyed(QObject*)));
		connect(vtkView, SIGNAL(filesDropped(const QStringList&)), this, SLOT(addToDBAuto(const QStringList&)));

		vtkView->setAttribute(Qt::WA_DeleteOnClose);
		dockDBtree->update();
		vtkView->showMaximized();
		QMainWindow::statusBar()->showMessage(QString("New 3D View"), 3000);
		
		return vtkView;
	}

	else
	{
		QMessageBox::warning(NULL, "Warning", QString::fromLocal8Bit("已存在3D窗体"), QMessageBox::Ok);
		return ERROR;
	}
}

QMdiSubWindow* PCS::getMdiSubWindow(pcsWindow* View)
{
	QList<QMdiSubWindow*> subWindowList = pcs_MdiArea->subWindowList();
	for (int i = 0; i < subWindowList.size(); ++i)
	{
		if (static_cast<pcsWindow*>(subWindowList[i]->widget()) == View)
			return subWindowList[i];
	}
	return 0;
}

void PCS::UpdateMenu3D()
{
	menu3D->clear();
	menu3D->addAction(actionNew3DView);
	menu3D->addSeparator();
	menu3D->addAction(actionClose);
	menu3D->addAction(actionCloseAll);
	menu3D->addSeparator();
	menu3D->addAction(actionTitle);

	QList<QMdiSubWindow *> windows = pcs_MdiArea->subWindowList();
	if (!windows.isEmpty())
	{
		//动态添加分隔符
		QAction* separator = new QAction(this);
		separator->setSeparator(true);
		menu3D->addAction(separator);
		
		QList<QMdiSubWindow *> windows = pcs_MdiArea->subWindowList();
		for (int i = 0; i<windows.size(); ++i)
		{
			QWidget *child = windows.at(i)->widget();
			QString text = QString("&%1 %2").arg(i + 1).arg(child->windowTitle());
			/*QAction *action = menu3D->addAction(text);
			action->setCheckable(true);
			action->setChecked(child == GetActiveVTKWindow());
			connect(action, SIGNAL(triggered()), pcs_windowMapper, SLOT(map()));
			pcs_windowMapper->setMapping(action, windows.at(i));*/
		}
	}
}

void PCS::SwitchConsoleDisplay()
{
	if (dockConsole && dockConsole->isHidden())
	{
		dockConsole->show();
		QApplication::processEvents();
	}
}

//void PCS::showPointCloud()
//{
//	for (int i = 0; i != pc_vec.size(); i++)
//	{
//		viewer->addPointCloud(pc_vec[i].cloud, "cloud" + QString::number(i).toStdString());
//		viewer->updatePointCloud(pc_vec[i].cloud, "cloud" + QString::number(i).toStdString());
//	}
//	viewer->resetCamera();
//	
//}

void PCS::BanUI(bool state)
{
	//toolBarMainTool->setDisabled(state);
	//toolBarViewTool->setDisabled(state);
	//toolBarDataTool->setDisabled(state);
	//toolBarCameraTool->setDisabled(state);
	////toolBarPluginTools->setDisabled(state);
	//
	//dockDBtree->setDisabled(state);
	//menubar->setDisabled(state);

	//if (state)
	//{
	//	menuPointData->setDisabled(true);
	//	menuDataTool->setDisabled(true);
	//}
	//else
	//{
	//	UpdateMenus();
	//}

	//pcs_UiFrozen = state;
}

//void PCS::UpdateMenus()
//{
//	ccGLWindow* win = getActiveVTKWindow();
//	bool hasMdiChild = (win != 0);
//	bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities()>0);
//
//	//General Menu
//	menuEdit->setEnabled(true/*hasSelectedEntities*/);
//	menuTools->setEnabled(true/*hasSelectedEntities*/);
//
//	//3D Views Menu
//	actionClose3DView->setEnabled(hasMdiChild);
//	actionCloseAll3DViews->setEnabled(hasMdiChild);
//	actionTile3DViews->setEnabled(hasMdiChild);
//	actionCascade3DViews->setEnabled(hasMdiChild);
//	actionNext3DView->setEnabled(hasMdiChild);
//	actionPrevious3DView->setEnabled(hasMdiChild);
//
//	//Shaders & Filters display Menu
//	bool shadersEnabled = (win ? win->areShadersEnabled() : false);
//	actionLoadShader->setEnabled(shadersEnabled);
//	actionDeleteShader->setEnabled(shadersEnabled);
//
//	bool filtersEnabled = (win ? win->areGLFiltersEnabled() : false);
//	actionNoFilter->setEnabled(filtersEnabled);
//
//	//View Menu
//	toolBarView->setEnabled(hasMdiChild);
//
//	//oher actions
//	actionSegment->setEnabled(hasMdiChild && hasSelectedEntities);
//	actionTranslateRotate->setEnabled(hasMdiChild && hasSelectedEntities);
//	actionPointPicking->setEnabled(hasMdiChild);
//	//actionPointListPicking->setEnabled(hasMdiChild);
//	actionTestFrameRate->setEnabled(hasMdiChild);
//	actionRenderToFile->setEnabled(hasMdiChild);
//	actionToggleSunLight->setEnabled(hasMdiChild);
//	actionToggleCustomLight->setEnabled(hasMdiChild);
//	actionToggleCenteredPerspective->setEnabled(hasMdiChild);
//	actionToggleViewerBasedPerspective->setEnabled(hasMdiChild);
//
//	//plugins
//	foreach(QAction* act, m_glFilterActions.actions())
//		act->setEnabled(hasMdiChild);
//}

void PCS::RedrawAll()
{
	QList<QMdiSubWindow*> windows = pcs_MdiArea->subWindowList();
	for (int i = 0; i < windows.size(); ++i)
	{
		static_cast<pcsWindow*>(windows.at(i)->widget())->Redraw();
	}
		
}

void PCS::EnableAll()
{
	QList<QMdiSubWindow *> windows = pcs_MdiArea->subWindowList();
	for (int i = 0; i < windows.size(); ++i)
	{
		windows.at(i)->setEnabled(true);
	}
		
}

void PCS::DisableAll()
{
	QList<QMdiSubWindow *> windows = pcs_MdiArea->subWindowList();
	for (int i = 0; i < windows.size(); ++i)
	{
		windows.at(i)->setEnabled(false);
	}
		
}

void PCS::DisableAllAlone(pcsWindow* view)
{
	QList<QMdiSubWindow*> windows = pcs_MdiArea->subWindowList();
	for (int i = 0; i < windows.size(); ++i)
	{
		if (static_cast<pcsWindow*>(windows.at(i)->widget()) != view)
			windows.at(i)->setEnabled(false);
	}
		
}

void PCS::RefreshAll()
{
	QList<QMdiSubWindow*> windows = pcs_MdiArea->subWindowList();
	for (int i = 0; i < windows.size(); ++i)
	{
		static_cast<pcsWindow*>(windows.at(i)->widget())->refresh();
	}
		
}

void PCS::UpdateUI()
{
	/*UpdateUIWithSelection();
	UpdateMenus();
	if (dockDBtree)
		dockDBtree->UpdatePropertiesView();*/
}

void PCS::SetFrontView()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->setView(PCS_FRONT_VIEW);*/
}

void PCS::SetBottomView()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//	win->setView(PCS_BOTTOM_VIEW);
}
void PCS::SetTopView()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//	win->setView(PCS_TOP_VIEW);
}


void PCS::SetBackView()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->setView(PCS_BACK_VIEW);*/
}

void PCS::SetLeftView()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->setView(PCS_LEFT_VIEW);*/
}

void PCS::SetRightView()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->setView(PCS_RIGHT_VIEW);*/
}

void PCS::SetIsoView1()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->setView(PCS_ISO_VIEW_1);*/
}

void PCS::SetIsoView2()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->setView(PCS_ISO_VIEW_2);*/
}

void PCS::ToggleActiveWindowCenterPerspective()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//{
	//	win->togglePerspective(true);
	//	win->redraw();
	//	updateViewModePopUpMenu(win);
	//	updatePivotVisibilityPopUpMenu(win);
	//}
}


void PCS::ToggleActiveWindowViewerBasePerspective()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//{
	//	win->togglePerspective(false);
	//	win->redraw();
	//	updateViewModePopUpMenu(win);
	//	updatePivotVisibilityPopUpMenu(win);
	//}
}

void PCS::ToggleActiveWindowCustomLight()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
	{
		win->toggleCustomLight();
		win->redraw();
	}*/
}

void PCS::ToggleActiveWindowSunLight()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//{
	//	win->toggleSunLight();
	//	win->redraw();
	//}
}

void PCS::SetGlobalZoom()
{
	/*pcsWindow* win = getActiveVTKWindow();
	if (win)
		win->zoomGlobal();*/
}

void PCS::ResetUniqueIDCount()
{
	QSettings set;
	set.setValue(pcs_uniqueIDKey, static_cast<unsigned>(0));
}

void PCS::ZoomOnSelectedEntities()
{
	//pcsWindow* win = 0;

	//ccHObject tempGroup("TempGroup");
	//size_t selNum = m_selectedEntities.size();
	//for (size_t i = 0; i<selNum; ++i)
	//{
	//	if (i == 0 || !win)
	//	{
	//		//take the first valid window as reference
	//		win = static_cast<ccGLWindow*>(m_selectedEntities[i]->getDisplay());
	//	}

	//	if (win)
	//	{
	//		if (m_selectedEntities[i]->getDisplay() == win)
	//		{
	//			tempGroup.addChild(m_selectedEntities[i], ccHObject::DP_NONE);
	//		}
	//		else if (m_selectedEntities[i]->getDisplay() != 0)
	//		{
	//			ccLog::Error("All selected entities must be displayed in the same 3D view!");
	//			return;
	//		}
	//	}
	//}

	//if (tempGroup.getChildrenNumber() != 0)
	//{
	//	ccBBox box = tempGroup.getBB(false, false, win);
	//	win->updateConstellationCenterAndZoom(&box);
	//}

	//refreshAll();
}

void PCS::SetPivotAlwaysOn()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//{
	//	win->setPivotVisibility(pcsWindow::PIVOT_ALWAYS_SHOW);
	//	win->redraw();

	//	if (pcs_pivotVisibilityPopupButton)
	//		pcs_pivotVisibilityPopupButton->setIcon(actionSetPivotAlwaysOn->icon());
	//}
}

void PCS::SetPivotRotationOnly()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//{
	//	win->setPivotVisibility(pcsWindow::PIVOT_SHOW_ON_MOVE);
	//	win->redraw();

	//	if (pcs_pivotVisibilityPopupButton)
	//		pcs_pivotVisibilityPopupButton->setIcon(actionSetPivotRotationOnly->icon());
	//}
}

void PCS::SetPivotOff()
{
	//pcsWindow* win = getActiveVTKWindow();
	//if (win)
	//{
	//	win->setPivotVisibility(pcsWindow::PIVOT_HIDE);
	//	win->redraw();

	//	if (pcs_pivotVisibilityPopupButton)
	//		pcs_pivotVisibilityPopupButton->setIcon(actionSetPivotOff->icon());
	//}
}


void PCS::SetOtherView()
{
	//setOtherView(getActiveVTKWindow());
}

void PCS::SetCenterPerspectiveView()
{
	//setCenterPerspectiveView(getActiveVTKWindow());
}

void PCS::SetViewPerspectiveView()
{
	//setViewPerspectiveView(getActiveVTKWindow());
}

void PCS::About()
{
	QDialog* aboutDialog = new QDialog(this);

	Ui::aboutDlg ui;
	ui.setupUi(aboutDialog);

	QString Version = pcsVersion::GetPCSVersion();
	QString htmlText = ui.textEdit->toHtml();
	QString enrichedHtmlText = htmlText.arg(Version);
	//pcsLog::PrintDebug(htmlText);
	//pcsLog::PrintDebug(Version);
	//pcsLog::PrintDebug(enrichedHtmlText);

	ui.textEdit->setHtml(enrichedHtmlText);

	aboutDialog->exec();

	//delete aboutDialog; 
}

void PCS::OpenUsbRead()
{

}