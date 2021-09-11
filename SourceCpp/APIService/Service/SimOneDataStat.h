#pragma once

#include <string>
#include <chrono>
#include <fstream>
#include "cybertron/DefinesCore.hpp"
#include "cybertron/core/UtilPath.hpp"
#include "cybertron/core/UtilFile.hpp"
#include "cybertron/core/UtilDirectory.hpp"
#include "cybertron/core/UtilTime.hpp"

/*
	这个类专门进行业务的性能统计用的，主要统计
	1：传感器节点从自身发送到BridgeIO的时间消耗，BridgeIO到API层的消耗
	2：热区数据从自身发送到BridgeIO的时间消耗，BridgeIO到API层的消耗
	本类只计算，内部业务节点往外发的数据发送的性能统计
*/
using namespace cybertron;
class SimOneDataStat {
public:
	static SimOneDataStat * GetInstance()
	{
		static SimOneDataStat instance;
		return &instance;
	}

	enum statType { stat_hotarea, stat_sensor};


	struct DataStat
	{
		//当前总共接收了多少包
		std::uint64_t totalCount;
		//当前总共接收多少字节
		std::uint64_t totalPacketSize;
		//总共消耗了多少时间
		std::uint64_t totalTimeMs[3];
		//平均的时间消耗是多少
		std::uint64_t AvgTimeMS[3];
		//最大时间消耗是多少
		std::uint64_t MaxTimeMS[3];
		//最小时间消耗是多少
		std::uint64_t MinTimeMS[3];
		//有多少包超过阈值
		std::uint64_t thresholdCount[3];
	};
	struct DataContent
	{
		std::uint64_t nodeCreateTime;	
		std::uint64_t BridgeIORecvTime;	
		std::uint64_t BridgeIOSendTime;	
		std::uint64_t APIRecvTime;
		std::uint64_t packageSize;
		int frame;
		int sensorType;
	};
	SimOneDataStat();
	~SimOneDataStat();
	void Init(std::string caseName);
	void setLogFilePath(std::string logPath) {
		mLogFilePath = logPath;
	}
	void addPacketStat(statType type, DataContent* pDataContent);
	void updateStat(statType type, DataStat* pStat, std::uint64_t* t, int tCount, std::uint64_t packageSize,int frame, int sensorType);
	void CreatePerformanceFile();
	void setIsOpen(bool isOpen){ mbIsOpen = isOpen; };
	bool getIsOpen() {return mbIsOpen ;};
	void setFilePath(char *filePath) { mLogFilePath = filePath; };
	void setOutputInterval(int intervalPacketCount) { mIntervalPacketCount = intervalPacketCount; }
private:
	//当前总共的运行时间
	std::uint64_t mStartTimeStamp;
	std::uint64_t mTotalTimeSec;
	int mIndex;

	//定时多少秒输出
	int mOutPutTimeSec;

	DataStat		mHotAreaStat;
	DataStat		mAPISensorStat;

	std::uint64_t 	mThresholdMS;

	bool mbIsOpen;
	std::string mLogFilePath;
	std::string mCaseName;
	int mIntervalPacketCount;	
	std::fstream mOutputStream;
	std::vector<std::string> mColList;
};