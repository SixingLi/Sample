#pragma once

#include "assert.h"
#include <vector>
#include <string>

class UtilId
{
public:
	static std::string getRoadId(const std::string& laneId)
	{
		std::vector<std::string> idList = UtilString::split(laneId, "_");
		assert(idList.size() == 3);
		return idList[0];
	}

	static std::string getSectionId(const std::string& laneName)
	{
		const auto& idList = UtilString::split(laneName, "_");
		assert(idList.size() == 3);
		return idList[1];
	}

	static std::string getLaneId(const std::string& laneName)
	{
		const auto& idList = UtilString::split(laneName, "_");
		assert(idList.size() == 3);
		return idList[2];
	}
};