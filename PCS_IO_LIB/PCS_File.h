#ifndef PCS_FILE_H
#define PCS_FILE_H


#include <QString>
#include "pcsGeometry.h"


const int MAX_ASCII_FILE_LINE_LENGTH = 4096;

//点云格式文件类型，用于加载和保存标志
enum PCS_File_Types{
	UNKNOWN_FILE = 0,		/**< unknown type */
	LAS,
	//LAZ,
	ASCII,
	DXF,
	DGN,
	PDS,
	PDMS,
	PCD,
	PLY,
	OBJ,
	FILE_TYPES_COUNT
//	SOI,		/**< SOI (Mensi Soisic) */
//	ASCII,		/**< ASC,NEU, XYZ, TXT, PTS, etc. */
//	BIN,		/**< CloudCompare binary */
//	PN,		/**< Point-Normal (binary) */
//	PV,		/**< Point-Value (binary) */
//	PLY,		/**< Stanford mesh file */
//	OBJ,		/**< Wavefront mesh file */
//	POV,		/**< Multiple Point-Of-View cloud meta-file (ascii) */
//	MA,		/**< Maya mesh (ascii) */
//	ICM,		/**< Calibrated Images meta-file */
//	DM_ASCII,		/**< Depth Map (ascii) */
//	BUNDLER,		/**< Bundler output (ascii) */
//	VTK,		/**< VTK mesh/cloud file */
//	STL,		/**< STL mesh file (ascii) */
//	PCD,		/**< Point Cloud Library file */
//	OFF,		/**< OFF mesh file (ascii) */
//	PTX,		/**< PTX cloud file (ascii) */
//#ifdef PCS_X3D_SUPPORT
//	X3D,		/**< X3D mesh file */
//#endif
//#ifdef PCS_LAS_SUPPORT
//	LAS,		/**< LAS lidar point cloud (binary) */
//#endif
//#ifdef PCS_E57_SUPPORT
//	E57,		/**< ASTM E2807-11 E57 file */
//#endif
//#ifdef PCS_PDMS_SUPPORT
//	PDMS,		/**< PDMS (.pdmsmac) */
//#endif
//#ifdef PCS_DXF_SUPPORT
//	DXF,		/**< DXF (Autocad) */
//#endif
//#ifdef PCS_GDAL_SUPPORT
//	RASTER,		/**< GIS 2D1/2 raster (supported by GDAL) */
//#endif
//#ifdef PCS_FBX_SUPPORT
//	FBX,		/**< Autodesk FBX format */
//#endif
//	FILE_TYPES_COUNT,		/**< Fake file type (for automatic counting) */
};

const PCS_File_Types PCS_File_Types_Enums[] = { UNKNOWN_FILE, 
												LAS,
												//LAZ,
												ASCII,
												DXF,
												DGN,
												PDS,
												PDMS,
												PCD,
												PLY,
												OBJ
//SOI, ASCII, BIN,
//PN, PV, PLY, OBJ, POV,
//MA, ICM, DM_ASCII, BUNDLER,
//VTK, STL, PCD, OFF, PTX
//#ifdef PCS_X3D_SUPPORT
//, X3D
//#endif
//#ifdef PCS_LAS_SUPPORT
//, LAS
//#endif
//#ifdef PCS_E57_SUPPORT
//, E57
//#endif
//#ifdef PCS_PDMS_SUPPORT
//, PDMS
//#endif
//#ifdef PCS_DXF_SUPPORT
//, DXF
//#endif
//#ifdef PCS_GDAL_SUPPORT
//, RASTER
//#endif
//#ifdef PCS_FBX_SUPPORT
//, FBX
//#endif
};

const char PCS_File_Type_Filters[][64] = {
	"LAS lidar point cloud (*.Las *.Laz)",
	"ASCII files (*.txt *.asc *.neu *.xyz *.pts *.csv)",
	"DXF file (*.dxf)",
	"DGN file (*.dgn)",
	"PDS file (*.pds)",
	"PDMS file (*.pdms *.pdmsmac *.mac)",
	"PCD Point Cloud Library cloud (*.pcd)",
	"PLY file (*.ply)",
	"OBJ file (*.obj)",
	"All (*.*)",
	/*"SOI Soisic Mensi (*.soi)",
	"ASCII files (*.txt *.asc *.neu *.xyz *.pts *.csv)",
	"BIN CloudCompare binaries (*.bin)",
	"PN Point-Normal [binary] (*.pn)",
	"PV Point-Value [binary] (*.pv)",
	"PLY Stanford mesh file (*.ply)",
	"OBJ Wavefront mesh file (*.obj)",
	"POV Multiple Point-Of-View cloud meta-file [ascii] (*.pov)",
	"MA Maya ASCII file (*.ma)",
	"ICM Calibrated Images meta-file (*.icm)",
	"ASCII Depth Map (*.txt *.asc)",
	"Snavely's Bundler output (*.out)",
	"VTK cloud or mesh (*.vtk)",
	"STL mesh (*.stl)",
	"PCD Point Cloud Library cloud (*.pcd)",
	"OFF mesh (*.off)",
	"PTX cloud (*.ptx)"
#ifdef PCS_X3D_SUPPORT
	, "X3D mesh file (*.x3d)"
#endif
#ifdef PCS_LAS_SUPPORT
	, "LAS lidar point cloud (*.las *.laz)"
#endif
#ifdef PCS_E57_SUPPORT
	, "E57 ASTM E2807-11 files (*.e57)"
#endif
#ifdef PCS_PDMS_SUPPORT
	, "PDMS (*.pdms *.pdmsmac *.mac)"
#endif
#ifdef PCS_DXF_SUPPORT
	, "DXF (*.dxf)"
#endif
#ifdef PCS_GDAL_SUPPORT
	, "RASTER grid (*.*)"
#endif
#ifdef PCS_FBX_SUPPORT
	, "FBX Autodesk mesh (*.fbx)"
#endif*/

};



//打开点云文件遇到的错误
enum PCS_File_Error {
	PCS_FERR_NO_ERROR,
	PCS_FERR_BAD_ARGUMENT,
	PCS_FERR_UNKNOWN_FILE,
	PCS_FERR_WRONG_FILE_TYPE,
	PCS_FERR_WRITING,
	PCS_FERR_READING,
	PCS_FERR_NO_SAVE,
	PCS_FERR_NO_LOAD,
	PCS_FERR_BAD_ENTITY_TYPE,
	PCS_FERR_CANCELED_BY_USER,
	PCS_FERR_NOT_ENOUGH_MEMORY,
	PCS_FERR_MALFORMED_FILE,
	PCS_FERR_CONSOLE_ERROR,
	PCS_FERR_BROKEN_DEPENDENCY_ERROR,
	PCS_FERR_FILE_WAS_WRITTEN_BY_PLUGIN
};



class PCS_File
{
public:
	virtual ~PCS_File(){}

	static void LoadFromFile(const QString& filenames, PCS_File_Types ftype = UNKNOWN_FILE,
		bool coordinatesShiftEnable = 0,
		pcsVector3D coordinatesShift = 0
		);
	
	static PCS_File_Types ChangeFileFormatFromExtension(QString e);
	static PCS_File *CreateFile(PCS_File_Types fType);
};

#endif // PCS_FILE_H
    