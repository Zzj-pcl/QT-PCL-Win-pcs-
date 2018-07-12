#include "pcsVersion.h"

 
#define PCS_Version_Number "1.0"
#define PCS_Version_Time "2018-2"

bool PCS_ENV_64;

QString pcsVersion::GetPCSVersion(bool full)
{
	QString VersionStr = QString("%1 %2").arg(PCS_Version_Number).arg(PCS_Version_Time);

	if (PCS_ENV_64)
	{
		QString pcs64 = "64 bits";
		if (full)
		{
			QString pcsPlatForm = "Windows";
			VersionStr += QString("[%1 %2]").arg(pcsPlatForm).arg(pcs64);
		}
		else
		{
			VersionStr += QString("[%1]").arg(pcs64);
		}
	}
	return VersionStr;
}