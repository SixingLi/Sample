#ifndef LOGGER_H__
#define LOGGER_H__

#include "easylogging++.h"

 // The Macro "INITIALIZE_EASYLOGGINGPP" Must be executed once, so recommanded  run in the main file
// #define ELPP_THREAD_SAFE //支持多线程 set in easylogging++.h

#define LOGFatal(obj)  if((obj.get_level())<=(Logging::EFatal)) CLOG(FATAL, (obj.get_id().c_str())) // Fatal 级别，默认情况下会使程序中断，可设置标记 LoggingFlag::DisableApplicationAbortOnFatalLog 来阻止中断
#define LOGError(obj) if((obj.get_level())<=(Logging::EError)) CLOG(ERROR, (obj.get_id().c_str()))
#define LOGWarning(obj) if((obj.get_level())<=(Logging::EWarning)) CLOG(WARNING, (obj.get_id().c_str()))
#define LOGDebug(obj) if((obj.get_level())<=(Logging::EDebug)) CLOG(DEBUG, (obj.get_id().c_str()))
#define LOGInfo(obj) if((obj.get_level())<=(Logging::EInfo)) CLOG(INFO, (obj.get_id().c_str()))
#define LOGTrace(obj) if((obj.get_level())<=(Logging::ETrace)) CLOG(TRACE, (obj.get_id().c_str()))
#define LOGVerbose(obj) if((obj.get_level())<=(Logging::EVerbose)) VLOG(0)

namespace Logging
{
	enum Log_Level
	{
		EGlobal,
		EVerbose,
		ETrace,
		EInfo,
		EDebug,
		EWarning,
		EError,
		EFatal
	}; // items must be in order

	class Logger
	{
	public:
		Logger(std::string logger_id, const char* toStdOutput = "true", const char* toFile = "true");
		~Logger(){};

		void set_default_format(); // set by code
		void set_default_by_config_file(std::string path = "../conf/log.conf"); // set by config file
		void get_logger(const char* toStdOutput, const char* toFile);
		void set_output(const char* toStdOutput, const char* toFile);

		void set_level(Log_Level level); // height->low: INFO/WARNING/ERROR/FATAL/DEBUG/TRACE
		Log_Level get_level();
		std::string get_id();

	private:
		Log_Level m_level;
		el::Configurations m_defaultConf;
		std::string m_logger_id;
		std::string m_log_name;
	};



} // namespace Logging
#endif
