#pragma once

#include "horizon_map_env_ndm_data.h"
#include "horizon_map_env_ndm_physical.h"

namespace HorizonMapEnv {

	//LaneDirection
	typedef enum LaneDirection {
		LaneDirection_Unknown = 0,      // 0 << 0
		LaneDirection_Forward = 2,      // 1 << 1
		LaneDirection_Backward = 4,     // 1 << 2
		LaneDirection_Bidirection = 8,  // 1 << 3
		LaneDirection_Tidal = 16        // 1 << 4
	}NDM_LaneDirection;

	// LaneTransition
	typedef enum LaneTransition {
		LaneTransition_Unknown = 0,
		LaneTransition_Continue = 2,    // 1 << 1  lane continuous
		LaneTransition_Merging = 4,     // 1 << 2  lane merging
		LaneTransition_Splitting = 8    // 1 << 3, lane spliting
	}NDM_LaneTransition;

	// Lane Attr
	typedef struct LaneAttr {
		float curvature;     // curvature, (1/m), 1024--invalid
		float slope;         // road elevation, (��), 128--invalid
		float banking;       // super elevation, (��), 128--invalid
		float headingAngle;  // heading angle, (��), 361--invalid
		float offset;        // offset relative to the start of lane, (m)
		float width;         // width, (m), -1--invalid
	}NDM_LaneAttr;

	typedef struct VehicleType {
		typedef enum Type {
			VEHICLE_TYPE_UNKNOWN = 0,
			CAR = 1,
			MOTORCYCLE = 2,
			BICYCLE = 3,
			TRUCK = 4,
			TAXI = 5,
			BUS = 6,
			PEDESTRAIN = 7,
			CABLE_CAR = 8
		}NDM_Type;
		NDM_Type type;
		float conf;
	}NDM_VehicleType;

	typedef enum SpeedLimitType {
		SpeedLimitType_Unknown = 0,
		SpeedLimitType_Min = 2,  // minimum speed limit
		SpeedLimitType_Max = 4   // maximum speed limit
	}NDM_SpeedLimitType;

	// Source of Speed Limit
	typedef enum SpeedLimitSource {
		SpeedLimitSource_Unknown = 0,
		SpeedLimitSource_Implicit = 2,  // derived results
		SpeedLimitSource_Explicit = 4   // clearly marked
	}NDM_SpeedLimitSource;


	// Defines speed limit for each type of vehicle
	typedef struct SpeedLimit {
		NDM_SpeedLimitType limit_type;			// see SpeedLimitType
		float speed_value;					// value of speed limit (km/h)
		NDM_SpeedLimitSource source;		// source of speed limit
		float offset;						// starting range of speed limit (offset from the start of lane, unit: m)
		float end_offset;					// end range of speed limit (offset from the start of lane, unit: m)
	}NDM_SpeedLimit;

	// Defines speed limit for each type of vehicle
	typedef struct TimeLimit {
		float time_begin;
		float time_end;
	}NDM_TimeLimit;


	// Lane Restriction
	// Defines all restrictions for the LaneElement.
	typedef struct LaneRestriction {
		SSD::SimVector<NDM_VehicleType> vehicle_types;
		SSD::SimVector<NDM_SpeedLimit> speed_limits;
		SSD::SimVector<NDM_TimeLimit> time_limits;
		SSD::SimVector<int> lanemarking_types;			//spread and inference, see LaneMarkingType
		SSD::SimVector<int> trafficsign_types;			//spread and inference, see TrafficSignType
		// Only set if restriction exists.
		float weight_limit;				//upper bound, need lower than this
		float height_limit;				//upper bound, need lower than this
	}NDM_LaneRestriction;

	typedef struct Link {
		SSD::SimString id;
		float offset;
		float end_offset;
	}NDM_Link;

	typedef struct Lane {
		SSD::SimString str_id;
		SSD::SimStringVector l_laneline_ids;
		SSD::SimStringVector r_laneline_ids;
		SSD::SimString driveline_id;
		NDM_LaneDirection direction;
		NDM_LaneTransition transition;
		SSD::SimVector<NDM_LaneAttr>  attrs;
		float lane_length;
		int type;
		SSD::SimVector<NDM_Link> objs;
		//SSD::SimVector<NDM_Link> obstacles;
		SSD::SimStringVector pred_ids;
		SSD::SimStringVector succ_ids;
		SSD::SimStringVector left_ids; //只给临近的一个车道
		SSD::SimStringVector right_ids; //只给临近的一个车道
		SSD::SimVector<NDM_LaneRestriction> restrictions;
	}NDM_Lane;

	typedef struct ParkingSpaceRestriction {
		int number_limit;
		SSD::SimVector<NDM_TimeLimit> time_limits;
		SSD::SimVector<NDM_VehicleType> vehicle_types;
	}NDM_ParkingSpaceRestriction;

	typedef struct ParkingSpace {
		SSD::SimString id;
		SSD::SimStringVector parkingslot_ids;
		SSD::SimStringVector border_ids; 
		NDM_Polygon bounding_polygon;
		SSD::SimStringVector link_ids;  ///ParkingSpace, Section, Junction
		NDM_ParkingSpaceRestriction restriction;// restriction of the parking space,base.proto 不用
	}NDM_ParkingSpace;

	typedef enum SpecialSituationType {
		SpecialSituationType_DeadEnd = 248,
		SpecialSituationType_FerryTerminal = 249,
		SpecialSituationType_TollBooth = 250,
		SpecialSituationType_RailroadCrossing = 251,
		SpecialSituationType_PedestrianCrossing = 252,
		SpecialSituationType_SpeedBump = 253,
		SpecialSituationType_CertifiedRoad = 254,
		SpecialSituationType_TollBooth_CertifiedRoad = 255
	}NDM_SpecialSituationType;

	typedef struct Section { //双向上两个section
		SSD::SimString id;
		SSD::SimStringVector lane_ids;
		SSD::SimStringVector l_border_ids; //不用
		SSD::SimStringVector r_border_ids; //不用
		SSD::SimVector<NDM_Link> objs;
		double length;

		SSD::SimStringVector pred_ids;  ///Section, Junction
		SSD::SimStringVector succ_ids;  ///Section, Junction
		SSD::SimStringVector left_ids;  ///Section, ParkingSpace 不用
		SSD::SimStringVector right_ids; ///Section, ParkingSpace 不用

		NDM_Polygon bounding_polygon;  //section内左右车道的采样点
		NDM_SpecialSituationType special_situation_type; //收费站检查站
	}NDM_Section;

	typedef struct Junction {
		SSD::SimString id;
		SSD::SimVector<NDM_Link> objs;
		/// virtual lanes in Junction
		SSD::SimStringVector lane_ids;
		NDM_Polygon bounding_polygon;

		SSD::SimStringVector in_link_ids;		///Section, ParkingSpace
		SSD::SimStringVector out_link_ids;		///Section, ParkingSpace
	}NDM_Junction;

	typedef struct Elevation {
		SSD::SimString id;
		SSD::SimStringVector elevationplane_ids;
		SSD::SimStringVector link_ids;  ///Elevation
	}NDM_Elevation;

	typedef struct LogicalLayer {
		SSD::SimVector<NDM_Line> virtuallines;
		SSD::SimVector<NDM_Lane> lanes;
		//SSD::SimVector<NDM_Lane> virtual_lanes;
		SSD::SimVector<NDM_ParkingSpace> parkingspaces;
		SSD::SimVector<NDM_Section> sections;
		SSD::SimVector<NDM_Junction> junctions;
	}NDM_LogicalLayer;

	typedef enum CalculateSide
	{
		eLeft,
		eRight,
		eBoth
	}CalculateSide_;

	typedef struct RoadSection
	{
		long roadId;
		int sectionIndex;
		CalculateSide_ side = CalculateSide_::eRight;
		SSD::SimString roadSectionSideName;
		SSD::SimString roadSectionName;
		SSD::SimStringVector laneNameList;
	}RoadSection_;

	class NDM_LogicalLayer_Creator {
	public:
		void GetBoundarySampleList_(SSD::SimVector<NDM_Line> &virtuallines, const SSD::SimString& laneName, const SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo, long junctionId)
		{
			for (HDMapStandalone::MLaneLineInfo mLaneinfo : laneLineInfo)
			{
				std::vector<std::string> splitItem = UtilString::split(mLaneinfo.laneName.GetString(), "_");
				if (splitItem[2] == "0") {
					continue;
				}
				bool isInJunction = false;
				long junctionId_c;
#ifdef NDM_MAP_LOCAL
				isInJunction = HDMapStandalone::MHDMap::IsInJunction(mLaneinfo.laneName, junctionId_c);
#else
				isInJunction = SimOneAPI::IsInJunction(mLaneinfo.laneName, junctionId_c);
#endif
				if (laneName == mLaneinfo.laneName || junctionId_c == junctionId)
				{
					//std::cout << "mLaneinfo.laneName:" << mLaneinfo.laneName.GetString() << std::endl;
					auto iter = mVirtualLines.find(mLaneinfo.laneName);
					if (iter == mVirtualLines.end())
					{
						NDM_CurveLine lineleft, lineright, linecenter;
						NDM_Line left;
						left.str_id.SetString((std::string(mLaneinfo.laneName.GetString()) + "_l").c_str());
						left.type = NDM_LineType::LineType_Virtual;

						NDM_Line right;
						right.str_id.SetString((std::string(mLaneinfo.laneName.GetString()) + "_r").c_str());
						right.type = NDM_LineType::LineType_Virtual;

						NDM_Line center;
						center.str_id.SetString((std::string(mLaneinfo.laneName.GetString()) + "_c").c_str());
						center.type = NDM_LineType::LineType_Center;

#ifdef NDM_MAP_LOCAL
						HDMapStandalone::MLaneInfo cLaneinfo = HDMapStandalone::MHDMap::GetLaneSample(mLaneinfo.laneName);
#else
						HDMapStandalone::MLaneInfo cLaneinfo;
						SimOneAPI::GetLaneSample(mLaneinfo.laneName, cLaneinfo);
#endif

						lineleft.marking = LineMarking_IntersectionVirualMarking;  //TODO:
						lineright.marking = LineMarking_IntersectionVirualMarking;  //TODO:
						lineleft.color = UNKNOWN_LINE_COLOR;
						lineright.color = UNKNOWN_LINE_COLOR;
						linecenter.marking = LineMarking_IntersectionVirualMarking;
						linecenter.color = UNKNOWN_LINE_COLOR;

						for (SSD::SimPoint3DVector points : mLaneinfo.leftBoundary.segmentList)
						{
							for (auto point : points)
							{
								NDM_Point point_pt;
								point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
								lineleft.points.push_back(point_pt);
							}
						}
						for (SSD::SimPoint3DVector points : mLaneinfo.rightBoundary.segmentList)
						{
							for (auto point : points)
							{
								NDM_Point point_pt;
								point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
								lineright.points.push_back(point_pt);
							}
						}

						for (auto point : cLaneinfo.centerLine)
						{
							NDM_Point point_pt;
							point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
							linecenter.points.push_back(point_pt);
						}

						//break;
						left.lines_3d.push_back(std::move(lineleft));
						right.lines_3d.push_back(std::move(lineright));
						center.lines_3d.push_back(std::move(linecenter));

						virtuallines.push_back(std::move(left));
						virtuallines.push_back(std::move(right));
						virtuallines.push_back(std::move(center));
						mVirtualLines[mLaneinfo.laneName] = true;

					}
				}
			}
		}

		void GetLaneLink_Lanes(SSD::SimVector<NDM_Lane> &lanes,const SSD::SimString &idStr) {

#ifdef NDM_MAP_LOCAL
			auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(idStr);
#else
			HDMapStandalone::MLaneLink laneLink;
			SimOneAPI::GetLaneLink(idStr, laneLink);
#endif
			//predecessorLaneNameList
			if (idStr == mCurrentLaneName) {
				for (SSD::SimString predName : laneLink.predecessorLaneNameList)
				{
#ifdef NDM_MAP_LOCAL
					auto& laneInfo = HDMapStandalone::MHDMap::GetLaneSample(predName);
#else
					HDMapStandalone::MLaneInfo laneInfo;
					SimOneAPI::GetLaneSample(predName, laneInfo);
#endif
					LaneSample_ sample;
					sample.laneCode = 0;
					long juncId = -1;
#ifdef NDM_MAP_LOCAL
					sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(laneInfo.laneName, juncId);
#else
					sample.inJunction = SimOneAPI::IsInJunction(laneInfo.laneName, juncId);
#endif
					sample.leftBoundary = std::move(laneInfo.leftBoundary);
					sample.rightBoundary = std::move(laneInfo.rightBoundary);
					sample.centerLine = std::move(laneInfo.centerLine);
					GetLane_(lanes, predName, sample);
				}
			}


			std::vector<std::string> splitItem = UtilString::split(idStr.GetString(), "_");
#ifdef NDM_MAP_LOCAL
			const auto& sectionLanes = HDMapStandalone::MHDMap::GetLaneList(atol(splitItem[0].c_str()));
#else
			SSD::SimStringVector sectionLanes;
			SimOneAPI::GetLaneList(atol(splitItem[0].c_str()), sectionLanes);
#endif
			for (auto laneName_s : sectionLanes) {

				std::vector<std::string> splitItem_s = UtilString::split(laneName_s.GetString(), "_");
				long road_id = atoi(splitItem_s[0].c_str());
				auto iter = mRoadIds.find(road_id);
				if (iter != mRoadIds.end()) {
					mRoadIds[road_id] = false;
				}
				else {
					mRoadIds[road_id] = false;
				}

#ifdef NDM_MAP_LOCAL
				auto& laneInfo = HDMapStandalone::MHDMap::GetLaneSample(laneName_s);
#else
				HDMapStandalone::MLaneInfo laneInfo;
				SimOneAPI::GetLaneSample(laneName_s, laneInfo);
#endif
				LaneSample_ sample;
				sample.laneCode = 0;
				long juncId = -1;
#ifdef NDM_MAP_LOCAL
				sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(laneInfo.laneName, juncId);
#else
				sample.inJunction = SimOneAPI::IsInJunction(laneInfo.laneName, juncId);
#endif
				sample.leftBoundary = std::move(laneInfo.leftBoundary);
				sample.rightBoundary = std::move(laneInfo.rightBoundary);
				sample.centerLine = std::move(laneInfo.centerLine);

				GetLane_(lanes, laneName_s, sample);
			}
		}

		void GetLane_(SSD::SimVector<NDM_Lane> &lanes, const SSD::SimString &laneName, const LaneSample_ &laneSample)
		{
			NDM_Lane lane;
			long juncId;
			lane.str_id = laneName;
			//std::cout << "------lane name = " << laneName.GetString() << std::endl;

			GetJunction_(mLogicalLayer.junctions, laneName);
#ifdef NDM_MAP_LOCAL
			if (HDMapStandalone::MHDMap::IsInJunction(laneName, juncId))
#else
			if (SimOneAPI::IsInJunction(laneName, juncId))
#endif
			{
				return;
			}

			GetSection_(mLogicalLayer.sections, laneName);

			SSD::SimString llid;
			llid.SetString((std::string(laneName.GetString()) + "_l").c_str());
			lane.l_laneline_ids.push_back(llid);
			SSD::SimString lrid;
			lrid.SetString((std::string(laneName.GetString()) + "_r").c_str());
			lane.r_laneline_ids.push_back(lrid);
			SSD::SimString lcid;
			lcid.SetString((std::string(laneName.GetString()) + "_c").c_str());
			lane.driveline_id = lcid;
			lane.direction = NDM_LaneDirection::LaneDirection_Forward;
			lane.transition = NDM_LaneTransition::LaneTransition_Unknown;

			GetLaneAttr_(lane.attrs, laneSample);
#ifdef NDM_MAP_LOCAL
			lane.type = (int)HDMapStandalone::MHDMap::GetLaneType(laneName);//----------------------------
			auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(laneName);
#else
			HDMapStandalone::MLaneType type;
			SimOneAPI::GetLaneType(laneName, type);
			lane.type = (int)type;
			HDMapStandalone::MLaneLink laneLink;
			SimOneAPI::GetLaneLink(laneName, laneLink);
#endif
			for (auto& pre : laneLink.predecessorLaneNameList)
			{
				lane.pred_ids.push_back(pre);
			}
			for (auto& suc : laneLink.successorLaneNameList)
			{
				lane.succ_ids.push_back(suc);
			}
			
			//只写相邻的左侧一个车道名
			SSD::SimString leftLane = laneLink.leftNeighborLaneName;
			if (!leftLane.Empty())
			{
				lane.left_ids.push_back(leftLane);
			}

			//只写相邻的右侧一个车道名
			SSD::SimString rightLane = laneLink.rightNeighborLaneName;

			if (!rightLane.Empty())
			{
				lane.right_ids.push_back(rightLane);
			}

#ifdef NDM_MAP_LOCAL
			lane.lane_length = HDMapStandalone::MHDMap::GetLaneLength(laneName);
			lane.type = int(HDMapStandalone::MHDMap::GetLaneType(laneName));
#else
			lane.lane_length = SimOneAPI::GetLaneLength(laneName);
			HDMapStandalone::MLaneType laneType;
			SimOneAPI::GetLaneType(laneName, laneType);
			lane.type = (int)laneType;
#endif
			lanes.push_back(lane);
		}

		void GetLaneAttr_(SSD::SimVector<NDM_LaneAttr> &attrs,const LaneSample_ &laneSample) {

			double offset_= 0;
			int index = 0;
			for (index = 0; index < laneSample.leftBoundary.size(); index+= BOUNDARY_SAMPLE_DISTANCE) {
				NDM_LaneAttr laneAttr;
				laneAttr.width = sqrt((laneSample.leftBoundary[index].x - laneSample.rightBoundary[index].x)*(laneSample.leftBoundary[index].x - laneSample.rightBoundary[index].x) + (laneSample.leftBoundary[index].y - laneSample.rightBoundary[index].y)*(laneSample.leftBoundary[index].y - laneSample.rightBoundary[index].y));
				laneAttr.banking = NDM_Util::Get_Slop_Banking_(laneSample.leftBoundary[index], laneSample.rightBoundary[index]);

				if (index == 0 && laneSample.centerLine.size() > BOUNDARY_SAMPLE_DISTANCE) {
					laneAttr.curvature = NDM_Util::GetCurvature_(laneSample.centerLine[index], laneSample.centerLine[index + 1], laneSample.centerLine[index + 2]);
					laneAttr.slope = NDM_Util::Get_Slop_Banking_(laneSample.centerLine[index], laneSample.centerLine[index + 1]);

					SSD::SimPoint2D dir(laneSample.centerLine[index + 1].x - laneSample.centerLine[index].x, laneSample.centerLine[index + 1].y - laneSample.centerLine[index].y);
					dir.Normalize();
					laneAttr.headingAngle = NDM_Util::ConvertHeading(NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir));
					laneAttr.offset = offset_;
				}
				else if (index == laneSample.centerLine.size() - 1 && index - 2 > 0) {
					laneAttr.curvature = NDM_Util::GetCurvature_(laneSample.centerLine[index - 2], laneSample.centerLine[index - 1], laneSample.centerLine[index]);
					laneAttr.slope = NDM_Util::Get_Slop_Banking_(laneSample.centerLine[index - 2], laneSample.centerLine[index - 1]);

					SSD::SimPoint2D dir(laneSample.centerLine[index - 1].x - laneSample.centerLine[index - 2].x, laneSample.centerLine[index - 1].y - laneSample.centerLine[index - 2].y);
					dir.Normalize();
					laneAttr.headingAngle = NDM_Util::ConvertHeading(NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir));
					offset_+=sqrt((laneSample.centerLine[index].x - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].x)*(laneSample.centerLine[index].x - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].x) + (laneSample.centerLine[index].y - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].y)*(laneSample.centerLine[index].y - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].y));
					laneAttr.offset = offset_;
				}
				else {
					laneAttr.curvature = NDM_Util::GetCurvature_(laneSample.centerLine[index - 1], laneSample.centerLine[index], laneSample.centerLine[index + 1]);
					laneAttr.slope = NDM_Util::Get_Slop_Banking_(laneSample.centerLine[index - 1], laneSample.centerLine[index]);

					SSD::SimPoint2D dir(laneSample.centerLine[index].x - laneSample.centerLine[index - 1].x, laneSample.centerLine[index].y - laneSample.centerLine[index - 1].y);
					dir.Normalize();
					laneAttr.headingAngle = NDM_Util::ConvertHeading(NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir));

					offset_+=sqrt((laneSample.centerLine[index].x - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].x)*(laneSample.centerLine[index].x - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].x) + (laneSample.centerLine[index].y - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].y)*(laneSample.centerLine[index].y - laneSample.centerLine[index - BOUNDARY_SAMPLE_DISTANCE].y));
					laneAttr.offset = offset_;
				}
				attrs.push_back(laneAttr);
			}
		}

		void Create_VirtualLines(SSD::SimVector<NDM_Line> &virtuallines, const LaneInfo_t &laneInfo, const SSD::SimVector<HDMapStandalone::MLaneLineInfo> &laneLineInfo) {

			SSD::SimVector<long> JunctionIDs;
			bool isInJunction;
			long junctionId;

#ifdef NDM_MAP_LOCAL
			isInJunction = HDMapStandalone::MHDMap::IsInJunction(laneInfo.currentLane, junctionId);
#else
			isInJunction = SimOneAPI::IsInJunction(laneInfo.currentLane, junctionId);
#endif
			if (isInJunction) {
				auto iter = std::find_if(JunctionIDs.begin(), JunctionIDs.end(), [&](const long & item)
				{
					return item == junctionId;
				});
				if (iter == JunctionIDs.end())
				{
					JunctionIDs.push_back(junctionId);
					GetBoundarySampleList_(virtuallines, laneInfo.currentLane, laneLineInfo, junctionId);
				}
			}
			else {
#ifdef NDM_MAP_LOCAL
				auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(laneInfo.currentLane);
#else
				HDMapStandalone::MLaneLink laneLink;
				SimOneAPI::GetLaneLink(laneInfo.currentLane, laneLink);
#endif
				for (auto lanePre : laneLink.predecessorLaneNameList) {
#ifdef NDM_MAP_LOCAL
					isInJunction = HDMapStandalone::MHDMap::IsInJunction(lanePre, junctionId);
#else
					isInJunction = SimOneAPI::IsInJunction(lanePre, junctionId);
#endif
					if (isInJunction) {
						auto iter = std::find_if(JunctionIDs.begin(), JunctionIDs.end(), [&](const long & item)
						{
							return item == junctionId;
						});
						if (iter == JunctionIDs.end())
						{
							JunctionIDs.push_back(junctionId);
							GetBoundarySampleList_(virtuallines, lanePre, laneLineInfo, junctionId);
						}
					}
				}	
			}

			for (auto &laneInfoVitural : laneInfo.dataList) {
				for (auto &lanename : laneInfoVitural.laneNameList) {
#ifdef NDM_MAP_LOCAL
					isInJunction = HDMapStandalone::MHDMap::IsInJunction(lanename, junctionId);
#else
					isInJunction = SimOneAPI::IsInJunction(lanename, junctionId);
#endif
					if (isInJunction) {
						auto iter = std::find_if(JunctionIDs.begin(), JunctionIDs.end(), [&](const long & item)
						{
							return item == junctionId;
						});
						if (iter == JunctionIDs.end())
						{
							JunctionIDs.push_back(junctionId);
							GetBoundarySampleList_(virtuallines, lanename, laneLineInfo, junctionId);
						}
					}
				}
			}
		}

		void Create_Lanes(SSD::SimVector<NDM_Lane> &lanes, const LaneInfo_t &laneInfo,const SSD::SimVector<HDMapStandalone::MLaneLineInfo> &laneLineInfo) {

			//std::cout << "++++++++++++++++++++++++++++++" << laneInfo.currentLane.GetString() << std::endl;
			for (auto& laneData : laneInfo.dataList) {
				for (int laneIndex = 0; laneIndex < laneData.laneSampleList.size();laneIndex++) {
					GetLaneLink_Lanes(lanes, laneData.laneNameList[laneIndex]);
				}
			}

			for (auto &kv : mRoadIds) {
				if (kv.second == true) {
					kv.second = false;
					SSD::SimStringVector laneidlist = HDMapStandalone::MHDMap::GetLaneList(kv.first);
					if(!laneidlist.empty())
						GetLaneLink_Lanes(lanes, laneidlist[0]);
				}
			}
		}

		void GetJunctionConectRoad(long junctionId) {
#ifdef NDM_MAP_LOCAL
			HDMapStandalone::MJunction mjunction = HDMapStandalone::MHDMap::GetJunction(junctionId);
#else
			HDMapStandalone::MJunction mjunction;
			SimOneAPI::GetJunction(junctionId, mjunction);
#endif
			for (auto& incomingRoadid : mjunction.incomingRoadIds) {
				auto iter = mRoadIds.find(incomingRoadid);
				if (iter == mRoadIds.end()) {
					mRoadIds[incomingRoadid] = true;
				}
			}
		}

		void Create_ParkingSpaces(SSD::SimVector<NDM_ParkingSpace> &parkingspaces, const SSD::SimVector<HDMapStandalone::MParkingSpace> &parkingSpaces_) {
			
			for (auto& parkingspace_ : parkingSpaces_)
			{
				NDM_ParkingSpace parkingSpace;
				parkingSpace.id.SetString(std::to_string(parkingspace_.id).c_str());
				parkingSpace.parkingslot_ids.push_back((std::to_string(parkingspace_.id)+"_0").c_str());
				parkingSpace.link_ids.push_back(std::to_string(parkingspace_.roadId).c_str());
				NDM_Point point_pt;
				point_pt.x = parkingspace_.heading.x; point_pt.y = parkingspace_.heading.y; point_pt.z = parkingspace_.heading.z;
				parkingSpace.bounding_polygon.orientation = point_pt;
				for (auto& point : parkingspace_.boundaryKnots)
				{
					NDM_Point point_pt;
					point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
					parkingSpace.bounding_polygon.points.push_back(point_pt);
				}
				parkingSpace.bounding_polygon.edgeline_width = parkingspace_.front.width;
				auto iter = std::find_if(parkingspaces.begin(), parkingspaces.end(), [&](const ParkingSpace & item)
				{
					return item.id == parkingSpace.id;
				});
				if (iter == parkingspaces.end())
				{
					parkingspaces.push_back(std::move(parkingSpace));
				}
			}	
		}

		void GetJunction_(SSD::SimVector<NDM_Junction>&junctions, const SSD::SimString& laneName)
		{
			NDM_Junction junction;
			long juncId = -1;
#ifdef NDM_MAP_LOCAL
			if (HDMapStandalone::MHDMap::IsInJunction(laneName, juncId))
#else
			if (SimOneAPI::IsInJunction(laneName, juncId))
#endif
			{
				junction.id.SetString(("junction_"+std::to_string(juncId)).c_str());
#ifdef NDM_MAP_LOCAL
				HDMapStandalone::MJunction mjunction = HDMapStandalone::MHDMap::GetJunction(juncId);
#else
				HDMapStandalone::MJunction mjunction;
				SimOneAPI::GetJunction(juncId, mjunction);
#endif
				
				double center_x = 0;
				double center_y = 0;
				SSD::SimVector<NDM_Point> points_junction;
				for (auto& connect : mjunction.connectingRoadIds)
				{
					SSD::SimStringVector laneidlist = HDMapStandalone::MHDMap::GetLaneList(connect);
					for (auto& laneid : laneidlist)
					{
#ifdef NDM_MAP_LOCAL
						auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(laneid);
#else
						HDMapStandalone::MLaneLink laneLink;
						SimOneAPI::GetLaneLink(laneid, laneLink);
#endif

#ifdef NDM_MAP_LOCAL
						HDMapStandalone::MLaneInfo cLaneinfo = HDMapStandalone::MHDMap::GetLaneSample(laneid);
#else
						HDMapStandalone::MLaneInfo cLaneinfo;
						SimOneAPI::GetLaneSample(laneid, cLaneinfo);
#endif
						junction.lane_ids.push_back(laneid);
						NDM_Point point_pt;
						point_pt.x = cLaneinfo.centerLine[0].x; point_pt.y = cLaneinfo.centerLine[0].y; point_pt.z = cLaneinfo.centerLine[0].z;
						//junction.bounding_polygon.points.push_back(point_pt);
						points_junction.push_back(point_pt);
						//计算十字路口几何中心
						center_x += point_pt.x;
						center_y += point_pt.y;

						point_pt.x = cLaneinfo.centerLine[cLaneinfo.centerLine.size()-1].x; point_pt.y = cLaneinfo.centerLine[cLaneinfo.centerLine.size() - 1].y; point_pt.z = cLaneinfo.centerLine[cLaneinfo.centerLine.size() - 1].z;
						//junction.bounding_polygon.points.push_back(point_pt);
						points_junction.push_back(point_pt);
						//计算十字路口几何中心
						center_x += point_pt.x;
						center_y += point_pt.y;
						for (auto &lane_pred : laneLink.predecessorLaneNameList) {
							RoadSection_ roadSection_c;
							if (!GetRoadSection_(lane_pred, roadSection_c)) {
								continue;
							}
							std::string strID = std::string(roadSection_c.roadSectionSideName.GetString());
							auto iter = std::find_if(junction.in_link_ids.begin(), junction.in_link_ids.end(), [&](const SSD::SimString & item)
							{
								return item == strID.c_str();
							});
							if (iter == junction.in_link_ids.end())
							{
								//std::cout << "inlink:" << strID << std::endl;
								junction.in_link_ids.push_back(std::move(strID.c_str()));
							}
						}

						for (auto &lane_succ : laneLink.successorLaneNameList) {
							RoadSection_ roadSection_c;
							if (!GetRoadSection_(lane_succ, roadSection_c)) {
								continue;
							}
							std::string strID = std::string(roadSection_c.roadSectionSideName.GetString());
							auto iter = std::find_if(junction.out_link_ids.begin(), junction.out_link_ids.end(), [&](const SSD::SimString & item)
							{
								return item == strID.c_str();
							});
							if (iter == junction.out_link_ids.end())
							{
								//std::cout << "outlink:" << strID << std::endl;
								junction.out_link_ids.push_back(std::move(strID.c_str()));
							}
							
						}
					}
				}

				center_x = center_x / points_junction.size();
				center_y = center_y / points_junction.size();
				for (int index = 0; index < points_junction.size()-1;index++)
				{
					for (int j = 0; j < points_junction.size()-1 - index; j++)
					{
						SSD::SimPoint2D dir_j,dir_jaddone;
						dir_j.x = points_junction[j].x - center_x;
						dir_j.y = points_junction[j].y - center_y;
						dir_j.Normalize();
						dir_jaddone.x = points_junction[j+1].x - center_x;
						dir_jaddone.y = points_junction[j+1].y - center_y;
						dir_jaddone.Normalize();
						double anglej = NDM_Util::ConvertHeading(NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir_j));
						double anglejaddone = NDM_Util::ConvertHeading(NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir_jaddone));
						if(j==6)
						std::cout << "pointj1:	" << points_junction[j + 1].x << "	" << points_junction[j + 1].y << "	" << points_junction[j + 1].z << std::endl;

						if (anglej > anglejaddone)
						{
							NDM_Point temp = std::move(points_junction[j]);
							points_junction[j] = std::move(points_junction[j + 1]);
							points_junction[j + 1] = std::move(temp);
							if (j == 6) {
								std::cout << "			temp:" << temp.x << "	" << temp.y << "	" << temp.z << std::endl;
								std::cout << "			pointj:" << points_junction[j].x << "	" << points_junction[j].y << "	" << points_junction[j].z << std::endl;
								std::cout << "			pointj1:" << points_junction[j+1].x << "	" << points_junction[j+1].y << "	" << points_junction[j+1].z << std::endl;
							}
						}
					}
				}

				//根据十字路口采样点和几何中心建立方向向量，计算与正北方向的夹角，根据夹角排序顺时针方向
				junction.bounding_polygon.points = std::move(points_junction);


				auto iter = std::find_if(junctions.begin(), junctions.end(), [&](const Junction & item)
				{
					return item.id == junction.id;
				});
				if (iter == junctions.end())
				{
					junctions.push_back(std::move(junction));
				}
			}
		}

		void GetSectionSampleList_(NDM_Section &section, const SSD::SimString& laneName, CalculateSide_ side)
		{
			if (side == eLeft)
			{
				SSD::SimString l_border_id = (std::string(laneName.GetString()) + "_l").c_str();
				section.l_border_ids.push_back(l_border_id);
				int pos = std::string(laneName.GetString()).find_last_of('_');
				SSD::SimString cutLsection = (std::string(laneName.GetString()).substr(0, pos).c_str());

				HDMapStandalone::MLaneId id(laneName.GetString());
				SSD::SimString sectionId;
				sectionId.SetString((UtilString::ToString(id.roadId) + "_" + UtilString::ToString(id.sectionIndex)).c_str());

				auto iter = std::find_if(section.left_ids.begin(), section.left_ids.end(), [&](const SSD::SimString& item)
				{
					return item == sectionId;
				});
				if (iter == section.left_ids.end())
				{
					section.left_ids.push_back(sectionId);
				}
			}
			else if (side == eRight)
			{
				SSD::SimString strR = (std::string(laneName.GetString()) + "_r").c_str();
				section.r_border_ids.push_back(strR);
				int pos = std::string(laneName.GetString()).find_last_of('_');

				HDMapStandalone::MLaneId id(laneName.GetString());
				SSD::SimString sectionId;
				sectionId.SetString((UtilString::ToString(id.roadId) + "_" + UtilString::ToString(id.sectionIndex)).c_str());

				auto iter = std::find_if(section.right_ids.begin(), section.right_ids.end(), [&](const SSD::SimString& item)
				{
					return item == sectionId;
				});
				if (iter == section.left_ids.end())
				{
					section.right_ids.push_back(sectionId);
				}
			}
			else
			{
				SSD::SimString strL = (std::string(laneName.GetString()) + "_l").c_str();
				section.l_border_ids.push_back(strL);
				SSD::SimString strR = (std::string(laneName.GetString()) + "_r").c_str();
				section.r_border_ids.push_back(strR);
			}
		}

		void GetSection_(SSD::SimVector<NDM_Section> &sections, const SSD::SimString& laneName)
		{
			NDM_Section section;
			
			std::vector<std::string> splitItem = UtilString::split(laneName.GetString(), "_");
			SSD::SimStringVector laneList = HDMapStandalone::MHDMap::GetSectionLaneList(laneName);
			RoadSection_ roadSection_c;
			if (!GetRoadSection_(laneName,roadSection_c)) {
				return;
			}
			section.id.SetString(roadSection_c.roadSectionSideName.GetString());
			//std::cout << "section:	" << roadSection_c.roadSectionSideName.GetString() << std::endl;
			for (auto laneName_t : laneList)
			{
				RoadSection_ roadSection_t;
				if (!GetRoadSection_(laneName_t, roadSection_t)) {
					return;
				}
				if (roadSection_t.roadSectionSideName == roadSection_c.roadSectionSideName) {
					section.lane_ids.push_back(laneName_t);
				}
			}

			if (!section.lane_ids.empty()) {
#ifdef NDM_MAP_LOCAL
				section.length = HDMapStandalone::MHDMap::GetLaneLength(section.lane_ids[0]);
#else
				section.length = SimOneAPI::GetLaneLength(section.lane_ids[0]);
#endif
			}

			auto iter = std::find_if(sections.begin(), sections.end(), [&](const Section & item)
			{
				return item.id == section.id;
			});
			if (iter == sections.end())
			{
				SSD::SimStringVector laneList = HDMapStandalone::MHDMap::GetSectionLaneList(laneName);

				bool have_border = false;
				for (auto laneName_ : laneList) {
#ifdef NDM_MAP_LOCAL
					auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(laneName_);
#else
					HDMapStandalone::MLaneLink laneLink;
					SimOneAPI::GetLaneLink(laneName_, laneLink);
#endif
					//GetSectionSampleList_(section, laneName, CalculateSide_::eBoth);
					for (auto& pre : laneLink.predecessorLaneNameList)
					{
						bool isInJunction;
						long junctionId;

#ifdef NDM_MAP_LOCAL
						isInJunction = HDMapStandalone::MHDMap::IsInJunction(pre, junctionId);
#else
						isInJunction = SimOneAPI::IsInJunction(pre, junctionId);
#endif
						if (isInJunction) {
#ifdef NDM_MAP_LOCAL
							auto& laneLink_pre = HDMapStandalone::MHDMap::GetLaneLink(pre);
#else
							HDMapStandalone::MLaneLink laneLink_pre;
							SimOneAPI::GetLaneLink(pre, laneLink_pre);
#endif
							for (auto& pre_pre : laneLink_pre.predecessorLaneNameList) {
								RoadSection_ roadSection_c;
								if (!GetRoadSection_(pre_pre, roadSection_c)) {
									continue;
								}

								SSD::SimString pred_PredSectionName = roadSection_c.roadSectionSideName;

								auto iter = std::find_if(section.pred_ids.begin(), section.pred_ids.end(), [&](const SSD::SimString& item)
								{
									return item == pred_PredSectionName;
								});
								if (iter == section.pred_ids.end())
								{
									//std::cout << "predID:" << std::string(pred_PredSectionName.GetString()) << std::endl;
									section.pred_ids.push_back(pred_PredSectionName);
								}
							}
						}
						else {
							RoadSection_ roadSection_c;
							if (!GetRoadSection_(pre, roadSection_c)) {
								continue;
							}

							SSD::SimString predSectionName = roadSection_c.roadSectionSideName;

							auto iter = std::find_if(section.pred_ids.begin(), section.pred_ids.end(), [&](const SSD::SimString& item)
							{
								return item == predSectionName;
							});
							if (iter == section.pred_ids.end())
							{
								section.pred_ids.push_back(predSectionName);
							}
						}
					}
					for (auto& suc : laneLink.successorLaneNameList)
					{
						bool isInJunction;
						long junctionId;

#ifdef NDM_MAP_LOCAL
						isInJunction = HDMapStandalone::MHDMap::IsInJunction(suc, junctionId);
#else
						isInJunction = SimOneAPI::IsInJunction(suc, junctionId);
#endif
						if (isInJunction) {
#ifdef NDM_MAP_LOCAL
							auto& laneLink_suc = HDMapStandalone::MHDMap::GetLaneLink(suc);
#else
							HDMapStandalone::MLaneLink laneLink_suc;
							SimOneAPI::GetLaneLink(suc, laneLink_suc);
#endif
							for (auto& suc_suc : laneLink_suc.successorLaneNameList) {

								RoadSection_ roadSection_c;
								if (!GetRoadSection_(suc_suc, roadSection_c)) {
									continue;
								}

								SSD::SimString succ_SuccSectionName = roadSection_c.roadSectionSideName;

								auto iter = std::find_if(section.succ_ids.begin(), section.succ_ids.end(), [&](const SSD::SimString& item)
								{
									return item == succ_SuccSectionName;
								});
								if (iter == section.succ_ids.end())
								{
									section.succ_ids.push_back(succ_SuccSectionName);
								};
							}
						}
						else {
							RoadSection_ roadSection_c;
							if (!GetRoadSection_(suc, roadSection_c)) {
								continue;
							}

							SSD::SimString succSectionName = roadSection_c.roadSectionSideName;

							auto iter = std::find_if(section.succ_ids.begin(), section.succ_ids.end(), [&](const SSD::SimString& item)
							{
								return item == succSectionName;
							});
							if (iter == section.succ_ids.end())
							{
								//std::cout << "succID:" << std::string(succSectionName.GetString()) << std::endl;
								section.succ_ids.push_back(succSectionName);
							};
						}
					}
					HDMapStandalone::MLaneType lane_type;
#ifdef NDM_MAP_LOCAL
					lane_type = HDMapStandalone::MHDMap::GetLaneType(laneName);
#else
					SimOneAPI::GetLaneType(laneName, lane_type);
#endif
					std::vector<std::string> idList = UtilString::split(laneName_.GetString(), "_");
#ifdef NDM_MAP_LOCAL
					HDMapStandalone::MLaneInfo cLaneinfo = HDMapStandalone::MHDMap::GetLaneSample(laneName_);
#else
					HDMapStandalone::MLaneInfo cLaneinfo;
					SimOneAPI::GetLaneSample(laneName_, cLaneinfo);
#endif
					if (abs(atoi(idList[2].c_str()))==1) {
						for (int i = 0; i < cLaneinfo.leftBoundary.size();i+= BOUNDARY_SAMPLE_DISTANCE) {
							auto point = cLaneinfo.leftBoundary[i];
							NDM_Point point_pt;
							point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
							section.bounding_polygon.points.push_back(point_pt);
						}
					}

					if (lane_type == HDMapStandalone::MLaneType::border && abs(atoi(idList[2].c_str())) < laneList.size()) {
						for (int i = cLaneinfo.leftBoundary.size()-1; i >0; i -= BOUNDARY_SAMPLE_DISTANCE) {
							have_border = true;
							auto point = cLaneinfo.leftBoundary[i];
							NDM_Point point_pt;
							point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
							section.bounding_polygon.points.push_back(point_pt);
						}
					}
				}
				if (!have_border) {
#ifdef NDM_MAP_LOCAL
					HDMapStandalone::MLaneInfo cLaneinfo = HDMapStandalone::MHDMap::GetLaneSample(laneList[laneList.size() - 1]);
#else
					HDMapStandalone::MLaneInfo cLaneinfo;
					SimOneAPI::GetLaneSample(laneList[laneList.size() - 1], cLaneinfo);
#endif
					for (int i = cLaneinfo.rightBoundary.size()-1; i >0; i -= BOUNDARY_SAMPLE_DISTANCE) {
						auto point = cLaneinfo.rightBoundary[i];
						NDM_Point point_pt;
						point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
						section.bounding_polygon.points.push_back(point_pt);
					}
				}
				sections.push_back(std::move(section));
			}
		}

		bool GetRoadSection_(const SSD::SimString& laneName, RoadSection_& roadSection)
		{
			std::vector<std::string> idList = UtilString::split(laneName.GetString(), "_");
			roadSection.roadId = UtilString::FromString<long>(idList[0]);
			roadSection.sectionIndex = UtilString::FromString<int>(idList[1]);
			roadSection.laneNameList.push_back(laneName);

			const auto& laneId = UtilString::FromString<int>(idList[2]);
			std::string side_name = "right";
			if (laneId == 0)
			{
				return  false;
			}
			if (laneId > 0)
			{
				side_name = "left";
				roadSection.side = CalculateSide_::eLeft;
			}

			roadSection.roadSectionSideName.SetString((idList[0] + "_" + idList[1] + "_" + side_name).c_str());
			roadSection.roadSectionName.SetString((idList[0] + "_" + idList[1]).c_str());
			return true;
		}

		void ResetRoadIdTag(std::map<long, bool> &roadIds) {
			for (auto &kv : roadIds) {
				if (kv.second == false) {
					kv.second = true;
				}
			}
		}

		void Create_LogicalLayer(const SSD::SimPoint3D& pos, const double forward, const HorizonMapEnv::NDM_PhysicalLayer_Creator &physicalLayerCreator)
		{
			mRoadIds = physicalLayerCreator.mRoadIds;
			ResetRoadIdTag(mRoadIds);
			const auto& idStr = physicalLayerCreator.mLaneInfo.currentLane;
			mCurrentLaneName = idStr;
			if (idStr.Empty())
			{
				return;
			}
			Create_VirtualLines(mLogicalLayer.virtuallines,
				physicalLayerCreator.mLaneInfo,
				physicalLayerCreator.mLaneLineInfo);
			Create_ParkingSpaces(mLogicalLayer.parkingspaces,
				physicalLayerCreator.mParkingSpaces);
			Create_Lanes(mLogicalLayer.lanes, 
				physicalLayerCreator.mLaneInfo,
				physicalLayerCreator.mLaneLineInfo);
			std::cout << "=============================finish to create ndm map======================" << std::endl;
		}
	public:
		NDM_LogicalLayer mLogicalLayer;
		SSD::SimString mCurrentLaneName;
		map<SSD::SimString,bool> mVirtualLines;
		std::map<long,bool> mRoadIds;
	};
}