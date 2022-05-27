#ifndef GNSS_H_
#define GNSS_H_

#define GNSS_MESSAGE_SIZE_MAX 100

#include <string>
#include <set>
#include <ctime>
#include <chrono>
#include <memory>
#include <sstream>
#include "Service/SimOneIOStruct.h"
#include "tsf/include/tsf.h"

using namespace std;

struct GNSSMessage
{
	int GPGGA_Size;
	int GPGLL_Size;
	int GPGSA_Size;
	int GPRMC_Size;
	int GPVTG_Size;

	char GNSS_GPGGA[GNSS_MESSAGE_SIZE_MAX];
	char GNSS_GPGLL[GNSS_MESSAGE_SIZE_MAX];
	char GNSS_GPGSA[GNSS_MESSAGE_SIZE_MAX];
	char GNSS_GPRMC[GNSS_MESSAGE_SIZE_MAX];
	char GNSS_GPVTG[GNSS_MESSAGE_SIZE_MAX];
};

class GNSS
{
public:
	GNSS(double initLatm, double initLon, double initAlt);
	virtual ~GNSS();

	GNSSMessage getGNSSMessage(SimOne_Data_Gps *pData);

protected:
	GNSSMessage m_GNSSMessage;

	bool m_initGPSBase = false;
	int m_lastStatellitesUpdateTime = 0;
	int m_satellitesNum = 0;
	string m_strGPGGA;
	string m_strGPGLL;
	string m_strGPGSA;
	string m_strGPRMC;
	string m_strGPVTG;

	string m_satellitesNumStr;
	string m_satellitesUsedStr;
	string m_timeUTC;
	string m_date;
	string m_positionGWS;
	string m_HDOP;
	string m_VDOP;
	string m_PDOP;
	string m_MSLAltitude;
	string m_speedKnot;
	string m_speedKPH;
	string m_courceOfGround;

	void updateUTC();
	void updateDate();
	string deg2GNSS(double deg, bool isLon);
	void updatePositionGWS(float x, float y, float z);
	void updateSatellitesInfo(bool isGPSLost);
	void updateHDOP(float HDOP, bool isGPSLost);
	void updateVDOP(float VDOP, bool isGPSLost);
	void updatePDOP(float HDOP, float VDOP, bool isGPSLost);
	void updateMSLAltitude(float x, float y, float z);
	string getCheckSum(string checkStr);
	void updateSpeedKnot(float velX, float velY, float velZ, bool isGPSLost);
	void updateSpeedKPH(float velX, float velY, float velZ, bool isGPSLost);
	void updateCourceOfGround(float angleRad);
};

#endif

