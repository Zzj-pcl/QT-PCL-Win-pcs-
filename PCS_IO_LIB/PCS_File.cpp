#include "PCS_File.h"
#include "pcsLogs.h"

//Qt
#include <QFileInfo>
#include <assert.h>

void PCS_File::LoadFromFile(const QString& filenames, PCS_File_Types ftype,
	bool coordinatesShiftEnable,
	pcsVector3D coordinatesShift)
{
	QFileInfo f(filenames);
	if (!f.exists())
	{
		pcsLogs::Warning(QString::fromLocal8Bit("[load] File '%1' doesn't exist!"));
		return;
	}
	if (ftype=UNKNOWN_FILE)
	{
		QString extention = QFileInfo(filenames).suffix();
		if (extention.isEmpty())
		{
			pcsLogs::Warning(QString::fromLocal8Bit("[load] Can't guess file format: no file extention"));
			return;
		}	
		ftype = ChangeFileFormatFromExtension(extention); //将扩展文件转为文件格式
		if (ftype=UNKNOWN_FILE)
		{
			pcsLogs::Warning(QString::fromLocal8Bit("[Load] Can't guess file format: unknown file extension '%1'").arg(extention));
		}
	}

	PCS_File* fIO = CreateFile(ftype);
	if (!fIO)
		return;

}

PCS_File_Types PCS_File::ChangeFileFormatFromExtension(QString e)
{
	e = e.toUpper();
	if (e == "ASC")
		return ASCII;
	else if (e == "TXT")
		return ASCII;
	else if (e == "XYZ")
		return ASCII;
	else if (e == "NEU")
		return ASCII;
	else if (e == "PTS")
		return ASCII;
	else if (e == "CSV")
		return ASCII;
	else if (e == "PLY")
		return PLY;
	else if (e == "OBJ")
		return OBJ;
	else if (e == "PCD")
		return PCD;
	else if (e == "LAS")
		return LAS;
	else if (e == "LAZ")
		return LAS;
	else if (e == "DXF")
		return DXF;
	return UNKNOWN_FILE;
}

//PCS_File* PCS_File::CreateFile(PCS_File_Types fType)
//{
//
//}