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
	�����ר�Ž���ҵ�������ͳ���õģ���Ҫͳ��
	1���������ڵ�������͵�BridgeIO��ʱ�����ģ�BridgeIO��API�������
	2���������ݴ������͵�BridgeIO��ʱ�����ģ�BridgeIO��API�������
	����ֻ���㣬�ڲ�ҵ��ڵ����ⷢ�����ݷ��͵�����ͳ��
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
		//��ǰ�ܹ������˶��ٰ�
		std::uint64_t totalCount;
		//��ǰ�ܹ����ն����ֽ�
		std::uint64_t totalPacketSize;
		//�ܹ������˶���ʱ��
		std::uint64_t totalTimeMs[3];
		//ƽ����ʱ�������Ƕ���
		std::uint64_t AvgTimeMS[3];
		//���ʱ�������Ƕ���
		std::uint64_t MaxTimeMS[3];
		//��Сʱ�������Ƕ���
		std::uint64_t MinTimeMS[3];
		//�ж��ٰ�������ֵ
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
	//��ǰ�ܹ�������ʱ��
	std::uint64_t mStartTimeStamp;
	std::uint64_t mTotalTimeSec;
	int mIndex;

	//��ʱ���������
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