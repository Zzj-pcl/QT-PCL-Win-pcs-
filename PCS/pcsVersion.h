#ifndef PCSVERSION_H
#define PCSVERSION_H

#include <QString.h>

class pcsVersion
{
public:
	pcsVersion();
	~pcsVersion();

	static QString GetPCSVersion(bool full = true);

};

#endif // PCSVERSION_H
