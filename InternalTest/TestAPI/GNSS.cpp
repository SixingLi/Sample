#include "GNSS.h"


GNSS::GNSS(double initLat, double initLon, double initAlt)
{
	SetGPSBase(initLat, initLon, initAlt);
}

GNSS::~GNSS()
{
}

GNSSMessage GNSS::getGNSSMessage(SimOne_Data_Gps *pData)
{
	updateUTC();
	updatePositionGWS(pData->posX, pData->posY, pData->posZ);
	updateSatellitesInfo(pData->isGPSLost);
	updateHDOP(1.0, pData->isGPSLost);
	updateVDOP(1.5, pData->isGPSLost);
	updatePDOP(1.0, 1.5, pData->isGPSLost);
	updateMSLAltitude(pData->posX, pData->posY, pData->posZ);
	updateSpeedKnot(pData->velX, pData->velY, pData->velZ, pData->isGPSLost);
	updateSpeedKPH(pData->velX, pData->velY, pData->velZ, pData->isGPSLost);
	updateCourceOfGround(pData->oriZ);

	m_strGPGGA = "$GPGGA," + m_timeUTC + m_positionGWS + "1," + m_satellitesNumStr + m_HDOP + m_MSLAltitude;
	m_strGPGGA += getCheckSum(m_strGPGGA);
	memset(m_GNSSMessage.GNSS_GPGGA, 0, 100);
	m_GNSSMessage.GPGGA_Size = m_strGPGGA.length();
	strcpy(m_GNSSMessage.GNSS_GPGGA, m_strGPGGA.c_str());

	m_strGPGLL = "$GPGLL," + m_positionGWS + m_timeUTC + "A,A";
	m_strGPGLL += getCheckSum(m_strGPGLL);
	memset(m_GNSSMessage.GNSS_GPGLL, 0, 100);
	m_GNSSMessage.GPGLL_Size = m_strGPGLL.length();
	strcpy(m_GNSSMessage.GNSS_GPGLL, m_strGPGLL.c_str());

	m_strGPGSA = "$GPGSA,A,3," + m_satellitesUsedStr + m_PDOP + m_HDOP + m_VDOP;
	m_strGPGSA += getCheckSum(m_strGPGSA);
	memset(m_GNSSMessage.GNSS_GPGSA, 0, 100);
	m_GNSSMessage.GPGSA_Size = m_strGPGSA.length();
	strcpy(m_GNSSMessage.GNSS_GPGSA, m_strGPGSA.c_str());

	m_strGPRMC = "$GPRMC," + m_timeUTC + "A," + m_positionGWS + m_speedKnot + m_courceOfGround + m_date + ",,A";
	m_strGPRMC += getCheckSum(m_strGPRMC);
	memset(m_GNSSMessage.GNSS_GPRMC, 0, 100);
	m_GNSSMessage.GPRMC_Size = m_strGPRMC.length();
	strcpy(m_GNSSMessage.GNSS_GPRMC, m_strGPRMC.c_str());

	m_strGPVTG = "$GPVTG," + m_courceOfGround + "T,,M," + m_speedKnot + "N," + m_speedKPH + "K,A";
	m_strGPVTG += getCheckSum(m_strGPVTG);
	memset(m_GNSSMessage.GNSS_GPVTG, 0, 100);
	m_GNSSMessage.GPVTG_Size = m_strGPVTG.length();
	strcpy(m_GNSSMessage.GNSS_GPVTG, m_strGPVTG.c_str());

	return m_GNSSMessage;
}

void GNSS::updateUTC()
{
	chrono::system_clock::time_point tp = chrono::system_clock::now();

	time_t raw_time = chrono::system_clock::to_time_t(tp);;
	tm* timeinfo = gmtime(&raw_time);

	char buf[8] = { 0 };

	strftime(buf, 8, "%H%M%S.", timeinfo);

	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());

	std::string milliseconds_str = std::to_string(ms.count() % 1000);

	if (milliseconds_str.length() < 3) {
		milliseconds_str = std::string(3 - milliseconds_str.length(), '0') + milliseconds_str;
	}

	m_timeUTC = string(buf) + milliseconds_str + ",";
}

void GNSS::updateDate()
{
	chrono::system_clock::time_point tp = chrono::system_clock::now();

	time_t raw_time = chrono::system_clock::to_time_t(tp);;
	tm* timeinfo = gmtime(&raw_time);

	char buf[8] = { 0 };

	strftime(buf, 8, "%d%m%y", timeinfo);

	m_date = string(buf) + ",";
}

string GNSS::deg2GNSS(double deg, bool isLon)
{
	string result;
	int dd = (int)deg;
	result = to_string(dd);

	if (isLon) {
		if (dd > 10 && dd < 100) {
			result = "0" + result;
		}
		else if (dd < 10) {
			result = "00" + result;
		}
	}
	else {
		if (dd < 10) {
			result = "0" + result;
		}
	}

	double min = abs((deg - (double)dd)) * 60;
	if (min < 10) {
		result += "0";
	}
	string temp = to_string(min);
	temp = temp.substr(0, temp.find(".") + 5);
	result += temp;

	return result;
}

void GNSS::updatePositionGWS(float x, float y, float z)
{
	string latStr, lonStr, result;
	Point temp;
	temp.x = x;
	temp.y = y;
	temp.z = z;
	Point p = GetWGS(temp);

	latStr = deg2GNSS(p.x, false);
	lonStr = deg2GNSS(p.y, true);

	m_positionGWS = latStr + ",N," + lonStr + ",E,";
}

void GNSS::updateSatellitesInfo(bool isGPSLost)
{
	if (isGPSLost) {
		m_lastStatellitesUpdateTime = 0;
		m_satellitesNum = 0;
		m_satellitesNumStr = "00,";
		m_satellitesUsedStr = ",,,,,,,,,,,,";
		return;
	}

	chrono::system_clock::time_point tp = chrono::system_clock::now();
	int sec = chrono::system_clock::to_time_t(tp);

	if (sec - m_lastStatellitesUpdateTime > 10) {
		m_satellitesNum = (rand() % (12 - 4 + 1)) + 4;

		m_satellitesNumStr = to_string(m_satellitesNum) + ",";

		if (m_satellitesNum < 10) {
			m_satellitesNumStr = "0" + m_satellitesNumStr;
		}

		set<int> satellitesID;
		int idCount = 0;
		while (satellitesID.size() != m_satellitesNum) {
			satellitesID.insert((rand() % (32 - 1 + 1)) + 1);
		}
		set<int>::iterator it;
		m_satellitesUsedStr.clear();
		for (it = satellitesID.begin(); it != satellitesID.end(); it++) {
			string temp = to_string(*it) + ",";
			if (*it < 10) {
				temp = "0" + temp;
			}
			m_satellitesUsedStr += temp;
		}
		for (int i = 0; i < 12 - m_satellitesNum; i++) {
			m_satellitesUsedStr += ",";
		}

		m_lastStatellitesUpdateTime = sec;
	}
}

void GNSS::updateHDOP(float HDOP, bool isGPSLost)
{
	if (isGPSLost) {
		m_HDOP = "99.99,";
	}
	else {
		string temp = to_string(HDOP);
		temp = temp.substr(0, temp.find(".") + 2);
		m_HDOP = temp + ",";
	}
}

void GNSS::updateVDOP(float VDOP, bool isGPSLost)
{
	if (isGPSLost) {
		m_VDOP = "99.99,";
	}
	else {
		string temp = to_string(VDOP);
		temp = temp.substr(0, temp.find(".") + 2);
		m_VDOP = temp + ",";
	}
}

void GNSS::updatePDOP(float HDOP, float VDOP, bool isGPSLost)
{
	if (isGPSLost) {
		m_PDOP = "99.99,";
	}
	else {
		float PDOP = sqrt(HDOP * HDOP + VDOP * VDOP);
		string temp = to_string(PDOP);
		temp = temp.substr(0, temp.find(".") + 2);
		m_PDOP = temp + ",";
	}
}

void GNSS::updateMSLAltitude(float x, float y, float z)
{
	Point tempPoint;
	tempPoint.x = x;
	tempPoint.y = y;
	tempPoint.z = z;
	Point p = GetWGS(tempPoint);

	string temp = to_string(p.z);
	temp = temp.substr(0, temp.find(".") + 2);
	m_MSLAltitude = temp + ",M,-5.0,M,,0000";
}

string GNSS::getCheckSum(string checkStr)
{
	int check;
	string::iterator it = checkStr.begin();
	it++;
	check = *it;
	it++;
	while (it != checkStr.end()) {
		check ^= *it;
		it++;
	}
	ostringstream oss;
	oss << "*" << hex << check << "\r\n";
	return oss.str();
}

void GNSS::updateSpeedKnot(float velX, float velY, float velZ, bool isGPSLost)
{
	float speedKnot = sqrt(velX * velX + velY * velY + velZ * velZ) * 1.9438;
	if (isGPSLost) {
		m_speedKnot = "0.00,";
	}
	else {
		string temp = to_string(speedKnot);
		temp = temp.substr(0, temp.find(".") + 3);
		m_speedKnot = temp + ",";
	}
}

void GNSS::updateSpeedKPH(float velX, float velY, float velZ, bool isGPSLost)
{
	float speedKPH = sqrt(velX * velX + velY * velY + velZ * velZ) * 3.6;
	if (isGPSLost) {
		m_speedKPH = "0.00,";
	}
	else {
		string temp = to_string(speedKPH);
		temp = temp.substr(0, temp.find(".") + 3);
		m_speedKPH = temp + ",";
	}
}

void GNSS::updateCourceOfGround(float angleRad)
{
	string temp;
	float angle = -angleRad * 57.2957795;
	if (angle < 0) {
		angle = 360 + angle;
	}

	temp = to_string(angle);
	temp = temp.substr(0, temp.find(".") + 3);
	m_courceOfGround = temp + ",";
}


