#pragma once

#include "libexport.h"
#include "public/common/MEnum.h"
#include "public/common/MLaneLink.h"
#include "public/common/MModelObject.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MAbstractRoadInfo.h"
#include "public/common/MLaneId.h"
#include "public/common/MObject.h"
#include "public/common/MParkingSpace.h"

namespace HDMapStandalone {

	// In this system, laneName is organized as string(SSD::SimString) with this format roadId_sectionIndex_laneId.
	// In public/common/MLaneId.h, MLaneId class is defined to hold this format laneName as data struct.
	// Its method ToString() can convert MLaneId object into roadId_sectionIndex_laneId format string.

    class LIBEXPORT MHDMap {
    public:

		// 1. Load map data related
		//
        //@brief Load map data from xodr file. Xodr file's geographic info must be localENU.
        //If geographic info is not localENU, need to use Wet.exe or WorldEditor to convert xodr into our localENU format.
        //@param xodrFile represents the xodr file's pathname.
        //outputFile represents the output ASCII file that dump data into.
        //@return Returns true if succeeded, false if failed.
        static bool LoadData(const SSD::SimString& xodrFile, MLoadErrorCode& error, const char* outputFile = nullptr);

        //@brief Load map data from xodr file content. Xodr content's geographic info must be localENU.
        //If geographic info is not localENU, need to use Wet.exe or WorldEditor to convert xodr into our localENU format.
        //@param xodrContent represents the xodr formatted content string.
        //outputFile represents the output ASCII file that dump data into.
        //@return Returns true if succeeded, false if failed.
        static bool LoadDataFromContent(const SSD::SimString& xodrContent, MLoadErrorCode& error, const char* outputFile = nullptr);

		//@brief Load map data from database content.
		//@param dbContent represents the json formatted content string.
		//headerXml is the header info in xml format.
		//outputFile represents the output ASCII file that dump data into.
		//@return Returns true if succeeded, false if failed.
		static bool LoadFromDBContent(const SSD::SimString& dbContent, const SSD::SimString& headerXml, MLoadErrorCode& error, const char* outputFile = nullptr);

		//@brief Export map data as xodr file. Xodr file's geographic info must be localENU.
		//@param xodrFile represents the xodr file's pathname.
		//@return Returns true if succeeded, false if failed.
		static bool ExportOpenDrive(const SSD::SimString& xodrFile);

		//@brief Get MAbstractRoadInfo object list based on the loaded map.
		//@return Returns MAbstractRoadInfo object list.
		static SSD::SimVector<MAbstractRoadInfo> GetAbstractRoadData();


		// 2. Lane related
		//
		//@brief Check whether lane exists in current map.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@return Returns true if exists, else returns false.
		static bool ContainsLane(const SSD::SimString& laneName);

		//@brief Get all lane data in the loaded map.
		//@return Returns MLaneVector for all lane data.
		static SSD::SimVector<MLaneInfo> GetLaneData();

		//@brief Get all lane data indicated by lane list.
		//@param laneList specifies a list of lane name, with each lane name with this format roadId_sectionIndex_laneId.
		//@return Returns MLaneVector for specified lanes.
		static SSD::SimVector<MLaneInfo> GetLaneData(const SSD::SimStringVector& laneList);

		//@brief Get all lanes' lane line info in the loaded map.
		//@return Returns MLaneLineInfo list for all lanes.
		static SSD::SimVector<MLaneLineInfo> GetLaneLineInfo();

		//@brief Get all junction id in the loaded map.
		//@return Returns junction id list.
		static SSD::SimVector<long> GetJunctionList();

		//@brief Get lane id list for specified road.
		//@param roadId.
		//@return Returns lane id list.
        static SSD::SimStringVector GetLaneList(const long& roadId);

		//@brief Get lane id list in the same section for specified lane id.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName roadId_sectionIndex_laneId's laneId should not be set as 0, as it does not make sense to use 0.
		//@return Returns lane id list that belong to the same section of specified lane id.
		static SSD::SimStringVector GetSectionLaneList(const SSD::SimString& laneId);

        //@brief Get lane data of specified lane.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map.
        //@return Returns MLaneInfo object.
        static MLaneInfo GetLaneSample(const SSD::SimString& laneName);

		//@brief Get all lane data by specified distance in forward direction of current position
		//@param inputPt is a 3d point.
		//laneName with this format roadId_sectionIndex_laneId.
		//forward is the distance in forward direction. Its maximum limit is 200 meters, and it minimum limit is greater than 0 meter.
		//@pre-condition laneName must exist in current map.
		//@return Returns MLaneInfo object list.
		static SSD::SimVector<MLaneInfo> GetLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, const double& forward);

		//@brief Get lane sample for specified lane and forward and backward range of current position
		//@param inputPt is a 3d point.
		//laneName with this format roadId_sectionIndex_laneId.
		//forward is the distance in forward direction. Its maximum limit is 200 meters, and it minimum limit is greater than 0 meter.
		//info as an output parameter, is a MLaneInfo object.
		//@pre-condition laneName must exist in current map.
		//@return Returns true if inputPt is valid to define the range of lane sample.
		static bool GetCurrentLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName,
			const double& forwardMaximum, const double& backwardMaximum, MLaneInfo& info);

		//@brief Get lane's MLaneLink based on lane's name.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map. laneName roadId_sectionIndex_laneId's laneId should not be set as 0,
		//as it does not make sense to use 0.
		//@return Returns MLaneLink object.
		static MLaneLink GetLaneLink(const SSD::SimString& laneName);

        //@brief Get lane's speed limit.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map.
        //@return Returns value of speed limit.
        static double GetLaneSpeedLimit(const SSD::SimString& laneName);

        //@brief Get lane's type.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map.
        //@return Returns MLaneType enum value.
        static MLaneType GetLaneType(const SSD::SimString& laneName);

		//@brief Get length of lane's middle line.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns lane's length.
		static double GetLaneLength(const SSD::SimString& laneName);

		//@brief Get lane's width in bitangent direction of specified MLaneId.
		//@param laneId is lane's id in MLaneId format.
		//s is the s-value in  s-t coordinate system.
		//@pre-condition laneId must exist in current map.
		//@return Returns lane width by s.
		static double GetLaneWidth(const HDMapStandalone::MLaneId& laneId, const double& s);

		static SSD::SimPoint3D GetLaneDirection(const SSD::SimString& laneName, const double& s);

		static double GetLaneT(const HDMapStandalone::MLaneId& laneId, const double& roadS);
		static double GetLaneWidthByRoadS(const HDMapStandalone::MLaneId& laneId, const double& roadS);

		//@brief Get length of road.
		//@param roadId is road's id.
		//@pre-condition specified road must exist in current map.
		//@return Returns road's length.
		static double GetRoadLength(const long& roadId);

		//@brief Check whether specified road is two-side road or not.
		//@param roadId is road's id.
		//@pre-condition specified road must exist in current map.
		//@return Returns true if specified road is two-side road, else returns false.
		static bool IsTwoSideRoad(const long& roadId);

        //@brief Check whether current lane is driving.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map.
        //@return Returns true if is driving, else returns false.
        static bool IsDriving(const SSD::SimString& laneName);

        //@brief Check whether current lane is driving and no dirving lane on left side or right side.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map.
        //@return Returns true if the lane is driving with no dirving lane on left side or right, else returns false.
        static bool IsDrivingAtEdge(const SSD::SimString& laneName);

		//@brief Get the list of cross hatch in the map.
		//@return Returns cross hatch MObject list.
		static SSD::SimVector<MObject> GetCrossHatchList();

		//@brief Get the list of cross hatch in the specified lane's road neighborhood.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns cross hatch MObject list.
		static SSD::SimVector<MObject> GetCrossHatchList(const SSD::SimString& laneName);

		//@brief Check whether current lane belongs to a junction.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//juncId is as an output parameter, which returns the owner junction id.
		//@pre-condition laneName must exist in current map.
		//@return Returns true if the lane belongs to a junction, else returns false.
		static bool IsInJunction(const SSD::SimString& laneName, long& juncId);

		//@brief Get center point of the specified junction.
		//@param junctionId is junction's id.
		//@pre-condition junction must exist in current map.
		//@return Returns center point.
		static SSD::SimPoint3D GetJunctionCenter(const long& junctionId);

		//3. Location query related: go to MLocation.h
		//


		//4. Traffic light and sign related: go to MLightAndSign.h
		//


		//5. Parking space related
		//
        //@brief Get id list of parkingSpaces in the map.
        //@return Returns id list.
        static SSD::SimStringVector GetParkingSpaceIds();

		//@brief Get parkingSpace list in the map.
		//@return Returns MParkingSpace object list.
		static SSD::SimVector<MParkingSpace> GetParkingSpaceList();

        //@brief Get id list of parkingSpaces regarding of a position and a distance range any parkingSpace is close to.
        //@param inputPt is the referenced position; distance means the distance range.
        //ids is an output parameter to specify the ids of parkingSpaces.
        //@return Returns true if any parkingSpace is found, else returns false.
        static bool GetParkingSpaceIds(const SSD::SimPoint3D& inputPt, double distance, SSD::SimStringVector& ids);

        //@brief Check whether parkingSpace exists in current map.
        //@param id is the id of parkingSpace.
        //@return Returns true if exists, else returns false.
        static bool ContainsParkingSpace(const SSD::SimString& id);


		//6. Model related
		//
        //@brief Get model object list by specfiying object type list in the map.
        //@param typeList is the type list, string format as "type1,type2".
        //@return Returns model object list.
        static SSD::SimVector<MModelObject> GetModelObject(const SSD::SimString& typeList);
    };
}