#ifndef PCSWINDOW_H
#define PCSWINDOW_H

#include "pcsGeometry.h"
#include "pcsGenericDisplay.h"

//VTK
#include <vtkActorCollection.h>
#include <vtkActor2DCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkAppendFilter.h>
#include <vtkCameraActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkHull.h>
#include <vtkInformation.h>
#include <vtkInformationStringKey.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLODActor.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkMutexLock.h>
#include <vtkObjectFactory.h>
#include <vtkTextActor.h>
#include <vtkRectilinearGrid.h>
#include <vtkUnsignedCharArray.h>

#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkParallelCoordinatesInteractorStyle.h>
#include <vtkLight.h>
#include <vtkImageData.h>  
#include <vtkInteractorStyleImage.h>  
#include <vtkTIFFWriter.h>  
#include <vtkSmartPointer.h>  
#include <vtkImageCanvasSource2D.h>  
#include <vtkRenderWindowInteractor.h>   
#include <vtkImageActor.h>  
#include <vtkImageMapper3D.h>  
#include <vtkOrientationMarkerWidget.h>
#include <vtkTIFFReader.h>  
#include <vtkPolyDataMapper.h> 
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCoordinate.h>
#include <vtkViewport.h>
#include <vtkSmartPointer.h>
#include <vtkPropAssembly.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkProperty2D.h> 
#include <vtkDelaunay2D.h>
#include <vtkCaptionActor2D.h> 
#include <vtkRenderer.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
//Qt
#include <QGLWidget>
#include <QFont>
#include <QGLFormat>
#include <QFont>
//system
#include <set>
#include <list>

#define PCS_Picking_Buffer_Size 65536

class pcsWindow : public QVTKWidget
{
	Q_OBJECT
public:
	//! Picking mode
	enum Picking_Mode {
		No_Picking,
		Entity_Picking,
		Entity_Rect_Picking,
		Fast_Picking,
		Point_Picking,
		Triangle_Picking,
		Auto_Point_Picking,
		Default_Picking,
	};

	//! Interaction mode (with the mouse!)
	enum Interaction_Mode {
		TRANSFORM_CAMERA,
		TRANSFORM_ENTITY,
		SEGMENT_ENTITY,
		PAN_ONLY,
	};

	//! Default message positions on screen
	enum MessagePosition {
		Lower_Left_Message,
		Upper_Center_Message,
		Screen_Center_Message,
	};

	//! Message type
	enum MessageType {
		Custom_Message,
		Screen_Size_Message,
		Perspective_State_Message,
		Sun_Light_State_Message,
		Custom_Light_State_Message,
		Manual_Transformation_Message,
		Manual_Segmentation_Message,
	};

	//! Pivot symbol visibility
	enum PivotVisibility {
		Pivot_Hide,
		Pivot_Show_On_Move,
		Pivot_Always_Show,
	};

	pcsWindow(QWidget *parent = 0,
				bool silentInitialization = false);
	virtual ~pcsWindow();

	//继承pcsGenericDisplay显示
	virtual void toBeRefreshed();
	virtual void refresh();
	virtual void invalidateViewport();
	virtual bool supportVTKVersion(unsigned VTKVersionFlag);
	virtual QFont getTextDisplayFont() const; //takes rendering zoom into account!


	virtual void DisplayNewMessage(const QString& message,
									MessagePosition pos,
									bool append = false,
									int displayMaxDelay_sec = 2,
									MessageType type = Custom_Message);

	virtual void SetZoom(float value);  //设置当前大小

public slots:
	void  ZoomGlobal();
	void  FrameRate();
	virtual void Redraw();

signals:
	void EntitySelectionChanged(int uniqueID);
	void EntitiesSelectionChanged(std::set<int> entID);
	void PointPicked(int cloudUniqueID, unsigned pointIndex, int x, int y);
	void CameraDisplaced(float x, float y);
	void MouseWheelRotated(float wheelDelta_deg);
	void PerspectiveStateChanged();
	void PixelSizeChanged(float);
	void PivotPointChanged(const  pcsVector3D&);
	void CameraPosChanged(const  pcsVector3D&);
	void Translation(const pcsVector3D& t);
	void LeftButtonClicked(int, int);
	void RightButtonClicked(int, int);
	void MouseMoved(int, int, Qt::MouseButtons);
	void ButtonReleased();
	void Drawing3D();
	void FilesDropped(const QStringList& filenames);

protected:
	void InitialVTK();
	static bool InitVTKMessage();
	
	void SetBackgroudColor(); 

	/*********相机机制**********/
	//void SetCamera();
	/*********光照机制**********/
	//void SetLight();
	void SetSunLight(bool state);
	void ToggleSunLight();
	bool SunLightEnabled() const { return pcs_sunLightEnabled; }
	void SetCustomLight(bool state);
	void ToggleCustomLight();
	bool CustomLightEnabled() const { return pcs_customLightEnabled; }

	/**********坐标机制*********/
	void WinCoordinate3D();
	
	struct MessageToDisplay
	{
		QString message;
		int messageValidity_sec;
		MessagePosition position;
		MessageType type;
	};
	std::list<MessageToDisplay> pcs_messagesToDisplay;

	struct CaptureModeOptions
	{
		bool enabled;
		float zoomFactor;
		bool renderOverlayItems;

		CaptureModeOptions()
			: enabled(false)
			, zoomFactor(1.0f)
			, renderOverlayItems(false)
		{}
	};

	CaptureModeOptions pcs_captureMode;
	
	QPoint pcs_lastMousePosition;
	QPalette pcs_winColor;
	QFont pcs_font;

	int pcs_Width;
	int pcs_Height;

	bool pcs_validProjectionMatrix;
	bool pcs_customLightEnabled;
	bool pcs_sunLightEnabled;
	bool pcs_lodActivated;
	bool pcs_shouldBeRefreshed;
	bool pcs_cursorMoved;
	bool pxs_unclosable;
	bool pcs_updatefbo; //(frame buffer object)
	bool pcs_silentInitialization;
	bool pcs_messageToDisplay;

private:
	static QString getShadersPath();
};

#endif // PCSWINDOW_H
