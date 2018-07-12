#ifndef PCDFILE_H
#define PCDFILE_H

#include "PCS_File.h"
#include <vector>

class PCDFile : PCS_File
{
public:
	PCDFile();
	~PCDFile();

protected:
	struct  PCDHeader
	{
		QString version; 
		std::vector<QString> fields; 
		std::vector<size_t> size;
		std::vector<QString> type;
		std::vector<size_t> count;
		size_t width;
		size_t height; 
		std::vector<float> viewpoint;
		size_t points;
		QString data;
	};
	
};

#endif // PCDFILE_H
