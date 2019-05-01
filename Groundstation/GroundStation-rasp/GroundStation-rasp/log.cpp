/*
 * log.cpp
 *
 * Created: 16-Apr-19 23:14:55
 *  Author: Kenneth
 */ 
#include "log.h"

//empty constructor.
Log::Log()
{
	
}

Log::Log(std::string filePath, std::string fileName, LogMode_t mode)
{
	if(filePath.back() == '/')
	{
		_filePath = filePath;
	}else
	{
		_filePath = filePath + "/";
	}
		
	_fileName = fileName;
	_mode = mode;	
		
}

//destructor
Log::~Log()
{
	
}

void Log::WriteLog(std::string data)
{
	// 
	if(_mode == LOG_MODE_SAME_FILE)
	{
		logfilewPath = _filePath + _fileName;	
	}
	else
	{
		std::time_t t = std::time(0);   // get time now
		std::tm* now = std::localtime(&t);
		std::string day = "";
		std::string month = "";
		std::string year = std::to_string(now->tm_year+1900);
		
		if(now->tm_mday < 10)
		{
			day = "0" + std::to_string(now->tm_mday);
		}
		else
		{
			day = std::to_string(now->tm_mday);
		}
		
		if(now->tm_mon < 10)
		{
			month = "0" + std::to_string(now->tm_mon);
		}
		else
		{
			month = std::to_string(now->tm_mon);
		}		
			
		
		logfilewPath = _filePath + day + "-" + month + "-" + year + "-" + _fileName;
	}
	
	std::cout << "Opening log file:" << logfilewPath << ":" << std::endl;
	FILE * LogFile = fopen(logfilewPath.c_str(),"a");
	fprintf(LogFile, data.c_str());  // String to file
	fclose (LogFile); // must close after opening*/		
}
