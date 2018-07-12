#ifndef PCS_H
#define PCS_H

#include <QtWidgets/QMainWindow>
#include "ui_pcs.h"

//Qt
#include <QMainWindow>
#include <QString>
#include <QDialog>
#include <QDir>
#include <QActionGroup>

#include <vector>

//VTK
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType)
//PCL
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h> //io 数据读写
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
//outofcore
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/outofcore/boost.h>
#include <pcl/outofcore/octree_base.h>
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/grid.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/outofcore_cloud.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/viewport.h>


#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>

#include "PCS_File.h"
#include "pcsWindow.h"
#include "pcsPointCloud.h"
#include "pcsMainWinInterface.h"


class QMdiArea;
class QSignalMapper;
class QToolButton;
class QAction;
class QMdiSubWindow;

using namespace pcl;
using namespace pcl::outofcore;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef OutofcoreOctreeBase<OutofcoreOctreeDiskContainer<pcl::PointXYZ>, pcl::PointXYZ> OctreeDisk;
typedef OutofcoreOctreeBaseNode<OutofcoreOctreeDiskContainer<pcl::PointXY>, pcl::PointXYZ> OctreeDiskNode;

class PCS : public QMainWindow, public pcsMainWinInterface, public Ui::PCSClass
{
	Q_OBJECT
//protected:	
//	PCS();
//	virtual ~PCS();

public:	
	PCS(QWidget *parent = 0);
	~PCS();
	
	//静态类VTK窗体方法ss
	static void ResetUniqueIDCount();
	//static PCS* TheInstance();
	//static pcsWindow* GetActiveVTKWindow();
	//static pcsWindow* GetVTKWindow(const QString& title);
	//static void GetVTKWindows(std::vector<pcsWindow*>& vtkWindows);
	//static void RefreshAllVTKWindow();
	//static void UpdateWinUI();
	//static void CloseInstance();
	//virtual pcsWindow* getActiveVTKWindow();
	//virtual void DispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE);
	//virtual void ForceConsoleDisplay();
	//inline virtual QMainWindow* GetMainWin() { return this; }

	/*void AddPoint(const QStringList& filenames,
		PCS_File_Types fType = UNKNOWN_FILE,
		pcsWindow* View = 0);*/

public:
	QMdiSubWindow* getMdiSubWindow(QVTKWidget* win);

	pcsPointCloud pc;
	std::vector<pcsPointCloud> clouds_vec;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

protected slots:
	virtual void BanUI(bool state);
	virtual void RedrawAll();
	virtual void EnableAll();
	virtual void DisableAll();
	virtual void DisableAllAlone(pcsWindow* view);
	virtual void RefreshAll();
	virtual void UpdateUI();
	virtual void SetFrontView();
	virtual void SetBottomView();
	virtual void SetTopView();
	virtual void SetBackView();
	virtual void SetLeftView();
	virtual void SetRightView();
	virtual void SetIsoView1();
	virtual void SetIsoView2();
	virtual void ToggleActiveWindowCenterPerspective();
	virtual void ToggleActiveWindowCustomLight();
	virtual void ToggleActiveWindowSunLight();
	virtual void ToggleActiveWindowViewerBasePerspective();
	virtual void SetGlobalZoom();
	virtual void ZoomOnSelectedEntities();
	virtual void SetPivotAlwaysOn();
	virtual void SetPivotRotationOnly();
	virtual void SetPivotOff();
	virtual void SetOtherView();
	virtual void SetCenterPerspectiveView();
	virtual void SetViewPerspectiveView();
	virtual void SwitchConsoleDisplay();

public slots:
		pcsWindow* New3DView();
		void Load();
		void About();

		void OpenUsbRead();
		void UpdateMenu3D();
		

protected:
	void ConnectActions();
	/*void showpointcloudadd();
	void setcloudcolor(unsigned int r, unsigned int g, unsigned int b);*/

	bool pcs_UiFrozen;

	QToolButton* pcs_pivotVisibilityPopupButton;
	QMdiArea* pcs_MdiArea;
	QSignalMapper* pcs_WindowMapper;
	QMdiSubWindow* getMdiSubWindow(pcsWindow* View);

	QString pcs_pluginsPath;
	QStringList pcs_pluginFileNames;
	//QActionGroup pcs_glFilterActions;

};

#endif // PCS_H
