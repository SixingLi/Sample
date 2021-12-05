#pragma once

#include "libexport.h"
#ifdef STATIC_SSD
#include "SSD/public/SimPoint3D.h"
#else
#include "SSD/SimPoint3D.h"
#endif

#include "common/MSignal.h"

namespace HDMapStandalone {

	class LIBEXPORT MNavigation {
	public:
		//@brief Get lane's successor lanes' names.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns successor lanes' names.
		static SSD::SimStringVector GetSuccessors(const SSD::SimString& laneName);

		//@brief Get lane's predescessor lanes' names.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns predescessor lanes' names.
		static SSD::SimStringVector GetPredescessors(const SSD::SimString& laneName);

		//@brief Get lane's next comming junction.
		//It works if lane's successor belongs to the same road but with different section.
		//It returns unfound result if lane's owner road does not connect to any junction.
		//@param inputPt is a 3d point.
		//@pre-condition laneName must exist in current map.
		//@return Returns junction's id. Returns -1 if no junction is found.
		static long GetNextJunction(const SSD::SimPoint3D& inputPt);

		//@brief Get lane's next comming junction.
		//It works if lane's successor belongs to the same road but with different section.
		//It returns unfound result if lane's owner road does not connect to any junction.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns junction's id. Returns -1 if no junction is found.
		static long GetNextJunction(const SSD::SimString& laneName);

		//@brief Get lane's next comming junction's traffic light list.
		//Regard either this lane connects to the junction or its ownner road connects to the junction.
		//If this lane is inside the junction, should not be considered, this case, this API returns the next junction faraway or return false.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//juncId as an output parameter, which returns junction's id.
		//lights as an output parameter, which returns junction's traffic light list.
		//validityIndexList as an output parameter, which returns for each traffic light its validity index that map to input lane.
		//@return Returns true if the junction exists.
		static bool GetNextJunctionLights(const SSD::SimString& laneName, long& juncId, SSD::SimVector<MSignal>& lights,
			SSD::SimVector<SSD::SimVector<int>>& validityIndexList);

		//@brief Get lane's POI junction's traffic light list.
		//If lane belongs to the POI junction's incoming road, this api covers.
		//If lane is inside the POI junction, this api covers.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//juncId as an output parameter, which returns junction's id.
		//lights as an output parameter, which returns junction's traffic light list.
		//validityIndexList as an output parameter, which returns for each traffic light its validity index that map to input lane.
		//stopLineIds as an output parameter, which returns for each traffic light its validity stopLine id list.
		//@return Returns true if the junction exists.
		static bool GePOIJunctionLights(const SSD::SimString& laneName, long& juncId, SSD::SimVector<MSignal>& lights,
			SSD::SimVector<SSD::SimVector<int>>& validityIndexList, SSD::SimVector<SSD::SimVector<long>>& stopLineIds);

		//@brief Check whether ego goes through a junction controlled by a traffic light.
		//@param fromLocation is the location of start point, toLocation is the end point.
		//controlledByLight as an output parameter, which returns the found traffic light.
		//foundLight as an output parameter, which returns the found traffic light.
		//@return Returns true if is through a junction.
		static bool IsThroughJunction(const SSD::SimPoint3D& fromLocation, const SSD::SimPoint3D& toLocation,
			bool& controlledByLight, MSignal& foundLight);
	};
}