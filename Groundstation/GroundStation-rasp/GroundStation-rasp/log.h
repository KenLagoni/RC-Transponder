/*
 * log.h
 *
 * Created: 16-Apr-19 23:15:05
 *  Author: Kenneth
 */ 

#ifndef LOG_H_
#define LOG_H_

#include <string>
#include <ctime> // for time / date
#include <iostream> // cout debug

typedef enum
{
	LOG_MODE_SAME_FILE=0x01,
	LOG_MODE_NEW_FILE_DAYLY,	
}LogMode_t;

class Log
{
	public:
	Log();
	Log(std::string filePath, std::string fileName, LogMode_t mode);
	~Log();
	void WriteLog(std::string data);
	
	protected:
	
	
	private:
	LogMode_t _mode;
	std::string _filePath = "";
	std::string _fileName = "";
	std::string logfilewPath = "";
};


#endif /* LOG_H_ */