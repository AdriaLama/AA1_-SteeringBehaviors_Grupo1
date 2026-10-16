// FILE: Logger implementation - see logger.hpp for the contract/WATCH OUT.
// Note: setLoggerDirectory() is optional - skip it and writeToLog/deleteLog
// just use the current working directory instead of prefixing one.
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "logger.hpp"

Logger * Logger::s_pInstance = 0;

Logger * Logger::Instance()
{
	if (s_pInstance == 0)
	{
		s_pInstance = new Logger();
	}
	return s_pInstance;
}


void Logger::setLoggerDirectory(string _directory)
{
    directory = _directory;
    loggerDirSet = 1;
}

void Logger::deleteLog(string filename)
{
    int ret=0;
    string dirAndFilename = "";

    if(loggerDirSet){
        dirAndFilename = directory+"/"+filename;
        ret = remove(dirAndFilename.c_str());
    }
    else{
        ret = remove(filename.c_str());
    }

   if(ret == 0) {
      cout << "Log file:" << filename << " deleted successfully!" << endl;
   } else {
      cerr << "Error in deleteLog(...): unable to delete the log file:" << filename << endl;
   }
}

void Logger::writeToLog(string message, string filename)
{
    string dirAndFilename = "";

    if(loggerDirSet){
        dirAndFilename = directory+"/"+filename;
    }
    else{
        dirAndFilename = filename;
    }

    ofstream logfile;
    logfile.open (dirAndFilename, ios::out | ios::app);
    logfile << message << endl;
    logfile.close();

}

