#include "SimOneDataStat.h"
#pragma warning(disable:4996)
SimOneDataStat::SimOneDataStat() {

	mIndex = 0;
	mThresholdMS = 20;
	mHotAreaStat = { 0 };
	mAPISensorStat = { 0 };
	mOutPutTimeSec = 10;
	mStartTimeStamp = std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0);
	
	mbIsOpen = false;
	mLogFilePath = UtilPath::getExecutablePath();
	mIntervalPacketCount = 200;
}
SimOneDataStat::~SimOneDataStat() {
	//UtilTime::millisecondsCurrent()
	mOutputStream.flush();
	mOutputStream.close();
}
void SimOneDataStat::Init(std::string caseName) {

	// 时间戳/类型/总包数量/总接收(字节)/avg大小(字节)/总耗时(t0)总耗时(t1/总耗时(t2)/avg耗时(t0)/avg耗时(t1)/avg耗时(t2)
	//max耗时(t0)/max耗时(t1)/max耗时(t2)/min耗时(t0)/min耗时(t1)/min耗时(t2)/over的数量(t0)/over的数量(t1)/over的数量(t2)/over value
	//cur cost(t0)/cur cost(t1)/cur cost(t2)/当前运行时间
	mColList.push_back("TimeStamp");
	mColList.push_back("Type");
	mColList.push_back("Frame");
	mColList.push_back("SensorType");
	mColList.push_back("AcceptedBytes");
	mColList.push_back("TotalPacketNumber");
	mColList.push_back("TotalAcceptedBytes");
	mColList.push_back("AverageAcceptedBytes");
	mColList.push_back("TotalTimeSpent(t0)");
	mColList.push_back("TotalTimeSpent(t1)");
	mColList.push_back("TotalTimeSpent(t2)");
	mColList.push_back("AverageTimeSpent(t0)");
	mColList.push_back("AverageTimeSpent(t1)");
	mColList.push_back("AverageTimeSpent(t2)");
	mColList.push_back("MaxTimeSpent(t0)");
	mColList.push_back("MaxTimeSpent(t1)");
	mColList.push_back("MaxTimeSpent(t2)");
	mColList.push_back("MinTimeSpent(t0)");
	mColList.push_back("MinTimeSpent(t1)");
	mColList.push_back("MinTimeSpent(t2)");
	mColList.push_back("NumberThresholdExceeded(t0)");
	mColList.push_back("NumberThresholdExceeded(t1)");
	mColList.push_back("NumberThresholdExceeded(t2)");
	mColList.push_back("TotalThresholdExceeded");
	mColList.push_back("EachNodeTimeSpent(t0)");
	mColList.push_back("EachNodeTimeSpentt1)");
	mColList.push_back("EachNodeTimeSpent(t2)");
	mColList.push_back("CurrentRunTime");
	mCaseName = caseName;
	CreatePerformanceFile();
	return;
}
void SimOneDataStat::addPacketStat(statType type, DataContent* pDataContent){
	std::uint64_t t[3] = { 0 };
	t[0] = pDataContent->BridgeIORecvTime - pDataContent->nodeCreateTime;
	t[1] = pDataContent->BridgeIOSendTime - pDataContent->BridgeIORecvTime;
	t[2] = pDataContent->APIRecvTime - pDataContent->BridgeIOSendTime;
	std::uint64_t packageSize = pDataContent->packageSize;
	mIndex++;
	int frame = pDataContent->frame;
	int sensorType = pDataContent->sensorType;
	switch (type)
	{
	case SimOneDataStat::stat_hotarea:
		updateStat(type, &mHotAreaStat, t, 3, packageSize, frame, sensorType);
		break;
	case SimOneDataStat::stat_sensor:
		updateStat(type, &mAPISensorStat, t, 3, packageSize, frame, sensorType);
		break;
	default:
		break;
	}
}
void SimOneDataStat::updateStat(statType type, DataStat* pStat, std::uint64_t* t, int tCount, std::uint64_t packageSize, int frame, int sensorType) {
	pStat->totalCount++;
	pStat->totalPacketSize += packageSize;
	for (size_t i = 0; i < tCount; i++)
	{
		pStat->totalTimeMs[i] += t[i];
		pStat->AvgTimeMS[i] = pStat->totalTimeMs[i] / pStat->totalCount;
		if (t[i] > mThresholdMS) {
			pStat->thresholdCount[i] += 1;
		}
		if (t[i] > pStat->MaxTimeMS[i]) {
			pStat->MaxTimeMS[i] = t[i];
		}
		if (t[i] < pStat->MinTimeMS[i]) {
			pStat->MinTimeMS[i] = t[i];
		}
	}
	mTotalTimeSec = std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0);
	std::string lines;
	if (pStat->totalCount % mIntervalPacketCount == 0) {
		std::uint64_t avg = pStat->AvgTimeMS[2];
		std::vector<std::string> ContentList;
		char szLine[1024] = { 0 };
		std::string currentTime = UtilTime::getLocalTimeStringForFile();
		std::uint64_t overValueCount = pStat->thresholdCount[0]+pStat->thresholdCount[1]+pStat->thresholdCount[2];
		if (type == 0)
		{		
			sprintf(szLine, "%s,%s,%d,%d,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id\n", \
				currentTime.c_str(),
				"HotAreaTask",
				frame,
				sensorType,
				packageSize,
				pStat->totalCount, 
				pStat->totalPacketSize,
				avg, 
				pStat->totalTimeMs[0], pStat->totalTimeMs[1], pStat->totalTimeMs[2],
				pStat->AvgTimeMS[0], pStat->AvgTimeMS[1], pStat->AvgTimeMS[2],
				pStat->MaxTimeMS[0], pStat->MaxTimeMS[1], pStat->MaxTimeMS[2], 
				pStat->MinTimeMS[0], pStat->MinTimeMS[1], pStat->MinTimeMS[2],
				pStat->thresholdCount[0], pStat->thresholdCount[1], pStat->thresholdCount[2],
				overValueCount,
				t[0], t[1], t[2],
				mTotalTimeSec- mStartTimeStamp
			);
			
			//printf("hotarea===>t1:%lld, t2:%lld, t3:%lld,size:%lld avg:%d count:%lld\n", t[0], t[1], t[2], packageSize, avg, pStat->totalCount. );
		}
		else
		{
			sprintf(szLine, "%s,%s,%d,%d,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id,%Id\n", \
				currentTime.c_str(),
				"SensorTask",
				frame,
				sensorType,
				packageSize,
				pStat->totalCount,
				pStat->totalPacketSize,
				avg,
				pStat->totalTimeMs[0], pStat->totalTimeMs[1], pStat->totalTimeMs[2],
				pStat->AvgTimeMS[0], pStat->AvgTimeMS[1], pStat->AvgTimeMS[2],
				pStat->MaxTimeMS[0], pStat->MaxTimeMS[1], pStat->MaxTimeMS[2],
				pStat->MinTimeMS[0], pStat->MinTimeMS[1], pStat->MinTimeMS[2],
				pStat->thresholdCount[0], pStat->thresholdCount[1], pStat->thresholdCount[2],
				overValueCount,
				t[0], t[1], t[2],
				mTotalTimeSec - mStartTimeStamp
			);
			//printf("sensor===>t1:%lld, t2:%lld, t3:%lld,size:%lld avg:%d count:%lld\n", t[0], t[1], t[2], packageSize, avg, pStat->totalCount);
		}
		mOutputStream.write(szLine, sizeof(szLine));
		mOutputStream.flush();
	}
}
void SimOneDataStat::CreatePerformanceFile()
{
	std::string logPath = UtilPath::combine(mLogFilePath, "performanceFile");
	if (!UtilDirectory::exist(logPath)) {
		if (!UtilDirectory::createMultipleDirectory(logPath))
		{
			printf("create performance file failed");
		}
	}
	printf("===============>PerformanceFilePath:%s", logPath.c_str());
	std::string file_name =std::string("Performance_") + mCaseName+ std::string("_")+ UtilTime::getLocalTimeStringForFile();
	file_name += ".csv";
	std::string path_file_name = UtilPath::combine(logPath, file_name);
	if (!UtilFile::tryOpen(mOutputStream, path_file_name, std::ios::out | std::ios::trunc | std::ios::binary))
	{
		return ;
	}
	std::string lines ;
	for (size_t i = 0; i < mColList.size(); i++)
	{
		lines += mColList[i];
		if (i == mColList.size() - 1) {
			lines += "\n";
		}
		else {
			lines += ',';
		}	
	}	
	mOutputStream.write(lines.data(), lines.length());
	return;
}