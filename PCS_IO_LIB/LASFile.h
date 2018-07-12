#ifndef LASFILE_H
#define LASFILE_H

#include "PCS_File.h"
//lasLib
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/factory.hpp>	// liblas::ReaderFactory
//system
#include <string.h>
#include <iostream>   // std::cout
#include <vector>

//公用文件头块 
enum PublicHeaderBlock{};
//变量长度记录
enum VariableLengthRecords{};
//点数据记录
enum Point_Data_Record{};

enum LAS_Files{ 
				LAS_X = 0,
				LAS_Y = 1,
				LAS_Z = 2,
				LAS_Classificaton = 3,
				LAS_Flight_Line_Edge = 4,
				LAS_Time = 5,
				LAS_Intensity = 6,
				LAS_Ruturn_Number = 7,
				LAS_Number_Of_Returns = 8,
				LAS_Scan_Angle = 9,
				LAS_ScanDirection =10,
				LAS_Red = 11,
				LAS_Green = 12,
				LAS_Blue = 13,
				LAS_User_Data = 14,
				LAS_Point_Source_Id = 15,
				
				LAS_Invalid = 255, //无效标识

};

const char LAS_File_Nanes[][28] = { "X",
									"Y",
									"Z",
									"Classification",
									"FlightLine Edge",
									"Time",
									"Intensity",
									"Return Number",
									"Numbser of Returns",
									"Scan Angle",
									"Scan Direction",
									"Red",
									"Green",
									"Blue",
									"User Data",
									"Point Source Id",
									//"[Classif] Value",
									//"[Classif] Synthetic flag",
									//"[Classif] Key-point flag",
									//"[Classif] Withheld flag",
};

class LASFile : public PCS_File 
{
public:
	virtual PCS_File_Error LoadFile();
	
};

#endif // LASFILE_H
