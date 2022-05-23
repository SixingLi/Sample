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
		int lanemarking_types;			//spread and inference, see LaneMarkingType
		int trafficsign_types;			//spread and inference, see TrafficSignType
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
		SSD::SimStringVector left_ids;
		SSD::SimStringVector right_ids;
		SSD::SimVector<NDM_LaneRestriction> restrictions;
	}NDM_Lane;

	typedef struct ParkingSpaceRestriction {
		int number_limit;// ��λ����
		SSD::SimVector<NDM_TimeLimit> time_limits;// ʱ������
		SSD::SimVector<NDM_VehicleType> vehicle_types;// ͣ�ų���
	}NDM_ParkingSpaceRestriction;


	// ͣ����
	typedef struct ParkingSpace {
		SSD::SimString id;
		// ��������, �洢�������˲�������parking(ͣ����) id
		// optional string global_id = 2;  // ��ʱ����Ҫ
		// һ��ͣ��������һ������ͣ��λ
		SSD::SimStringVector parkingslot_ids;
		SSD::SimStringVector border_ids;  // �߽���id, GeneralLine
		NDM_Polygon bounding_polygon;
		SSD::SimStringVector link_ids;  ///ParkingSpace, Section, Junction
		// restriction of the parking space, ��base.proto
		NDM_ParkingSpaceRestriction restriction;
	}NDM_ParkingSpace;


	typedef enum SpecialSituationType {
		SpecialSituationType_DeadEnd = 248,
		SpecialSituationType_FerryTerminal = 249,
		SpecialSituationType_TollBooth = 250,         // �շ�վ
		SpecialSituationType_RailroadCrossing = 251,
		SpecialSituationType_PedestrianCrossing = 252,
		SpecialSituationType_SpeedBump = 253,
		SpecialSituationType_CertifiedRoad = 254,     // ���վ
		SpecialSituationType_TollBooth_CertifiedRoad = 255,  // �շ�վ�ͼ��վ
	}NDM_SpecialSituationType;

	typedef struct Section {
		SSD::SimString id;
		SSD::SimStringVector lane_ids;
		SSD::SimStringVector l_border_ids;
		SSD::SimStringVector r_border_ids;
		SSD::SimVector<NDM_Link> objs;
		double length;  // section����, ��λ[m]

		SSD::SimStringVector pred_ids;  ///Section, Junction
		SSD::SimStringVector succ_ids;  ///Section, Junction
		SSD::SimStringVector left_ids;  ///Section, ParkingSpace
		SSD::SimStringVector right_ids; ///Section, ParkingSpace

		NDM_Polygon bounding_polygon;

		NDM_SpecialSituationType special_situation_type;// section����������ͣ����շ�վ�����վ��
	}NDM_Section;

	typedef struct Junction {
		SSD::SimString id;
		SSD::SimVector<NDM_Link> objs;
		/// virtual lanes in Junction
		/// ÿһ��Lane��������·���ڵĿ�ͨ�е���������(��Pred_Lane(id�洢��virtual lane��pred_ids��)��Succ_Lane(id�洢��succ_ids��))
		/// ����ʵ�ʴ��ڵ�lane(����·������ת��ת��)�����ɵ�virtual lane
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



	class NDM_LogicalLayer_Creator {

	public:
		void GetBoundarySampleList_(SSD::SimVector<NDM_Line> &virtuallines, const SSD::SimString& laneName, const SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo, long junctionId)
		{
			NDM_CurveLine lineleft, lineright, linecenter;

			for (HDMapStandalone::MLaneLineInfo mLaneinfo : laneLineInfo)
			{
				 std::vector<std::string> splitItem = UtilString::split(mLaneinfo.laneName.GetString(), "_");
				if (splitItem[2] == "0") {
					continue;
				}
				bool isInJunction = false;
#ifdef NDM_MAP_LOCAL
				isInJunction = HDMapStandalone::MHDMap::IsInJunction(mLaneinfo.laneName, junctionId);
#else
				isInJunction = SimOneAPI::IsInJunction(mLaneinfo.laneName, junctionId);
#endif
				if (laneName == mLaneinfo.laneName || isInJunction)
				{
					//std::cout << "mLaneinfo.laneName:" << mLaneinfo.laneName.GetString() << std::endl;
					auto iter = mVirtualLines.find(mLaneinfo.laneName);
					//std::find_if(virtuallines.begin(), virtuallines.end(), [&](const NDM_Line & item)
					//{
					//	//std::cout << "item.str_id.GetString():" << item.str_id.GetString() << std::endl;
					//	auto& splitItem = UtilString::split(item.str_id.GetString(), "_");
					//	auto& splitLaneName = UtilString::split(mLaneinfo.laneName.GetString(), "_");
					//	return splitItem[0] == splitLaneName[0] && splitItem[1] == splitLaneName[1] && splitItem[2] == splitLaneName[2];
					//});
					if (iter == mVirtualLines.end())
					{
						NDM_Line left;
						left.str_id.SetString((std::string(mLaneinfo.laneName.GetString()) + "_l").c_str());
						left.type = NDM_LineType::LineType_LaneLine;

						NDM_Line right;
						right.str_id.SetString((std::string(mLaneinfo.laneName.GetString()) + "_r").c_str());
						right.type = NDM_LineType::LineType_LaneLine;

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
								lineleft.points.push_back({ point.x,point.y,point.z,0 });
							}
						}
						for (SSD::SimPoint3DVector points : mLaneinfo.rightBoundary.segmentList)
						{
							for (auto point : points)
							{
								lineright.points.push_back({ point.x,point.y,point.z,0 });
							}
						}

						for (auto point : cLaneinfo.centerLine)
						{
							linecenter.points.push_back({ point.x,point.y,point.z,0 });
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
					sample.laneCode = NDM_Util::codeLane++;
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
			//leftNeighborLanes
			while (1)
			{
				SSD::SimString leftLane = laneLink.leftNeighborLaneName;

				if (leftLane.Empty())
				{
					break;
				}
#ifdef NDM_MAP_LOCAL
				auto& laneInfo = HDMapStandalone::MHDMap::GetLaneSample(leftLane);
#else
				HDMapStandalone::MLaneInfo laneInfo;
				SimOneAPI::GetLaneSample(leftLane, laneInfo);
#endif
				LaneSample_ sample;
				sample.laneCode = NDM_Util::codeLane++;
				long juncId = -1;
#ifdef NDM_MAP_LOCAL
				sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(laneInfo.laneName, juncId);
#else
				sample.inJunction = SimOneAPI::IsInJunction(laneInfo.laneName, juncId);
#endif
				sample.leftBoundary = std::move(laneInfo.leftBoundary);
				sample.rightBoundary = std::move(laneInfo.rightBoundary);
				sample.centerLine = std::move(laneInfo.centerLine);

				GetLane_(lanes, leftLane, sample);

#ifdef NDM_MAP_LOCAL
				laneLink = HDMapStandalone::MHDMap::GetLaneLink(leftLane);
#else
				SimOneAPI::GetLaneLink(leftLane, laneLink);
#endif
			}

			//rightNeighborLanes
			while (1)
			{
				auto& rightLane = laneLink.rightNeighborLaneName;
				if (rightLane.Empty())
				{
					break;
				}
#ifdef NDM_MAP_LOCAL
				auto& laneInfo = HDMapStandalone::MHDMap::GetLaneSample(rightLane);
#else
				HDMapStandalone::MLaneInfo laneInfo;
				SimOneAPI::GetLaneSample(rightLane, laneInfo);
#endif
				LaneSample_ sample;
				sample.laneCode = NDM_Util::codeLane++;
				long juncId = -1;
#ifdef NDM_MAP_LOCAL
				sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(laneInfo.laneName, juncId);
#else
				sample.inJunction = SimOneAPI::IsInJunction(laneInfo.laneName, juncId);
#endif
				sample.leftBoundary = std::move(laneInfo.leftBoundary);
				sample.rightBoundary = std::move(laneInfo.rightBoundary);
				sample.centerLine = std::move(laneInfo.centerLine);

				GetLane_(lanes, rightLane, sample);

#ifdef NDM_MAP_LOCAL
				laneLink = HDMapStandalone::MHDMap::GetLaneLink(rightLane);
#else
				SimOneAPI::GetLaneLink(rightLane, laneLink);
#endif
			}
		}

		void GetLane_(SSD::SimVector<NDM_Lane> &lanes, const SSD::SimString &laneName, const LaneSample_ &laneSample)
		{
			NDM_Lane lane;
			lane.str_id = laneName;
			//std::cout << "------lane name = " << laneName.GetString() << std::endl;

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
				

			while (1)
			{
				SSD::SimString leftLane = laneLink.leftNeighborLaneName;

				if (leftLane.Empty())
				{
					break;
				}

				lane.left_ids.push_back(leftLane);

#ifdef NDM_MAP_LOCAL
				laneLink = HDMapStandalone::MHDMap::GetLaneLink(leftLane);
#else
				SimOneAPI::GetLaneLink(leftLane, laneLink);
#endif
			}

			while (1)
			{
				SSD::SimString rightLane = laneLink.rightNeighborLaneName;

				if (rightLane.Empty())
				{
					break;
				}

				lane.right_ids.push_back(rightLane);

#ifdef NDM_MAP_LOCAL
				laneLink = HDMapStandalone::MHDMap::GetLaneLink(rightLane);
#else
				SimOneAPI::GetLaneLink(rightLane, laneLink);
#endif
			}

			long juncId;
#ifdef NDM_MAP_LOCAL
			if (HDMapStandalone::MHDMap::IsInJunction(laneName, juncId))
#else
			if (SimOneAPI::IsInJunction(laneName, juncId))
#endif

#ifdef NDM_MAP_LOCAL
			lane.lane_length = HDMapStandalone::MHDMap::GetLaneLength(laneName);
			lane.type = int(HDMapStandalone::MHDMap::GetLaneType(laneName));//������Ҫ����ö�����͡�
#else
			lane.lane_length = SimOneAPI::GetLaneLength(laneName);
			HDMapStandalone::MLaneType laneType;
			SimOneAPI::GetLaneType(laneName, laneType);
			lane.type = (int)laneType;//������Ҫ����ö�����͡�
#endif
			lanes.push_back(lane);
		}

		void GetLaneAttr_(SSD::SimVector<NDM_LaneAttr> &attrs,const LaneSample_ &laneSample) {

			double offset_= 0;
			for (int index = 0; index < laneSample.leftBoundary.size(); index++) {
				NDM_LaneAttr laneAttr;
				laneAttr.width = sqrt((laneSample.leftBoundary[index].x - laneSample.rightBoundary[index].x)*(laneSample.leftBoundary[index].x - laneSample.rightBoundary[index].x) + (laneSample.leftBoundary[index].y - laneSample.rightBoundary[index].y)*(laneSample.leftBoundary[index].y - laneSample.rightBoundary[index].y));
				laneAttr.banking = NDM_Util::Get_Slop_Banking_(laneSample.leftBoundary[index], laneSample.rightBoundary[index + 1]);

				if (index == 0 && laneSample.centerLine.size() > 2) {
					laneAttr.curvature = NDM_Util::GetCurvature_(laneSample.centerLine[index], laneSample.centerLine[index + 1], laneSample.centerLine[index + 2]);
					laneAttr.slope = NDM_Util::Get_Slop_Banking_(laneSample.centerLine[index], laneSample.centerLine[index + 1]);

					SSD::SimPoint2D dir(laneSample.centerLine[index + 1].x - laneSample.centerLine[index].x, laneSample.centerLine[index + 1].y - laneSample.centerLine[index].y);
					dir.Normalize();
					laneAttr.headingAngle = NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir) / M_PI * 180;
					laneAttr.offset = offset_;
				}
				else if (index == laneSample.centerLine.size() - 1 && index - 2 > 0) {
					laneAttr.curvature = NDM_Util::GetCurvature_(laneSample.centerLine[index - 2], laneSample.centerLine[index - 1], laneSample.centerLine[index]);
					laneAttr.slope = NDM_Util::Get_Slop_Banking_(laneSample.centerLine[index - 2], laneSample.centerLine[index - 1]);

					SSD::SimPoint2D dir(laneSample.centerLine[index - 1].x - laneSample.centerLine[index - 2].x, laneSample.centerLine[index - 1].y - laneSample.centerLine[index - 2].y);
					dir.Normalize();
					laneAttr.headingAngle = NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir) / M_PI * 180;
					offset_+=sqrt((laneSample.centerLine[index].x - laneSample.centerLine[index - 1].x)*(laneSample.centerLine[index].x - laneSample.centerLine[index - 1].x) + (laneSample.centerLine[index].y - laneSample.centerLine[index - 1].y)*(laneSample.centerLine[index].y - laneSample.centerLine[index - 1].y));
					laneAttr.offset = offset_;
				}
				else {
					laneAttr.curvature = NDM_Util::GetCurvature_(laneSample.centerLine[index - 1], laneSample.centerLine[index], laneSample.centerLine[index + 1]);
					laneAttr.slope = NDM_Util::Get_Slop_Banking_(laneSample.centerLine[index - 1], laneSample.centerLine[index]);

					SSD::SimPoint2D dir(laneSample.centerLine[index].x - laneSample.centerLine[index - 1].x, laneSample.centerLine[index].y - laneSample.centerLine[index - 1].y);
					dir.Normalize();
					laneAttr.headingAngle = NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir) / M_PI * 180;

					offset_+=sqrt((laneSample.centerLine[index].x - laneSample.centerLine[index - 1].x)*(laneSample.centerLine[index].x - laneSample.centerLine[index - 1].x) + (laneSample.centerLine[index].y - laneSample.centerLine[index - 1].y)*(laneSample.centerLine[index].y - laneSample.centerLine[index - 1].y));
					laneAttr.offset = offset_;
				}
				attrs.push_back(laneAttr);
			}
		}

		void Create_VirtualLines(SSD::SimVector<NDM_Line> &virtuallines, const LaneInfo_t &laneInfo) {
			SSD::SimVector<long> JunctionIDs;
			bool isInJunction;
			long junctionId;
#ifdef NDM_MAP_LOCAL
			auto& laneLineInfo = HDMapStandalone::MHDMap::GetLaneLineInfo();
			auto& parkingSpaces = HDMapStandalone::MHDMap::GetParkingSpaceList();
#else
			SSD::SimVector<HDMapStandalone::MLaneLineInfo> laneLineInfo;
			SimOneAPI::GetLaneLineInfo(laneLineInfo);
#endif

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
					GetBoundarySampleList_(virtuallines, laneInfo.currentLane, laneLineInfo, junctionId);
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

		void Create_Lanes(SSD::SimVector<NDM_Lane> &lanes, const LaneInfo_t &laneInfo) {

#ifdef NDM_MAP_LOCAL
			auto& laneLineInfo = HDMapStandalone::MHDMap::GetLaneLineInfo();
			auto& parkingSpaces = HDMapStandalone::MHDMap::GetParkingSpaceList();
#else
			SSD::SimVector<HDMapStandalone::MLaneLineInfo> laneLineInfo;
			SimOneAPI::GetLaneLineInfo(laneLineInfo);
#endif

			for (auto& laneData : laneInfo.dataList) {
				for (int laneIndex = 0; laneIndex < laneData.laneSampleList.size();laneIndex++) {
					GetLane_(lanes, laneData.laneNameList[laneIndex], laneData.laneSampleList[laneIndex]);
					GetLaneLink_Lanes(lanes, laneData.laneNameList[laneIndex]);
				}
			}
		}

		void Create_ParkingSpaces(SSD::SimVector<NDM_ParkingSpace> &parkingspaces, const SSD::SimPoint3D& pos, const double forward) {
		
			NDM_ParkingSpace parkingSpace;
			SSD::SimStringVector ids;

#ifdef NDM_MAP_LOCAL
			auto& parkingSpaces = HDMapStandalone::MHDMap::GetParkingSpaceList();
			if (HDMapStandalone::MHDMap::GetParkingSpaceIds(pos, forward, ids))
#else
			SSD::SimVector<HDMapStandalone::MParkingSpace> parkingSpaces;
			SimOneAPI::GetParkingSpaceList(parkingSpaces);
			if (SimOneAPI::GetParkingSpaceIds(pos, forward, ids))/// TODO
#endif
			{
				for (auto& id : ids)
				{
					for (auto& parkingspace : parkingSpaces)
					{
						if ((std::string(id.GetString()))== std::to_string(parkingspace.id))
						{
							//parking.restriction//TODO
							parkingSpace.id.SetString(id.GetString());
							parkingSpace.parkingslot_ids.push_back((std::string(id.GetString()) + "_0").c_str());
							parkingSpace.link_ids.push_back(std::to_string(parkingspace.roadId).c_str());
							parkingSpace.bounding_polygon.orientation = { parkingspace.heading.x,parkingspace.heading.y,parkingspace.heading.z ,0 };
							parkingSpace.bounding_polygon.normal = {0, 0, 0, 0};
							for (auto& point : parkingspace.boundaryKnots)
							{
								parkingSpace.bounding_polygon.points.push_back({ point.x,point.y,point.z,0});
							}
							parkingSpace.bounding_polygon.edgeline_width = parkingspace.front.width;
							auto iter = std::find_if(parkingspaces.begin(), parkingspaces.end(), [&](const ParkingSpace & item)
							{
								return item.id == id;
							});
							if (iter == parkingspaces.end())
							{
								parkingspaces.push_back(std::move(parkingSpace));
							}
						}
					}
				}
			}
		}

		void GetJunction_(SSD::SimVector<NDM_Junction>&junctions, const SSD::SimString& laneName)
		{
			NDM_Junction junction;
			long juncId = -1;
#ifdef DDS_MAP_LOCAL
			if (HDMapStandalone::MHDMap::IsInJunction(laneName, juncId))
#else
			if (SimOneAPI::IsInJunction(laneName, juncId))
#endif
			{
				junction.id.SetString(std::to_string(juncId).c_str());
				HDMapStandalone::MJunction mjunction = HDMapStandalone::MHDMap::GetJunction(juncId);
				for (auto& connect : mjunction.connectingRoadIds)
				{
					SSD::SimStringVector laneidlist = HDMapStandalone::MHDMap::GetLaneList(connect);
					for (auto& laneid : laneidlist)
					{

						int postion = std::string(laneid.GetString()).find_last_of('_');
						std::string strID = std::string(laneName.GetString()).substr(0, postion);
						auto iter = std::find_if(junction.in_link_ids.begin(), junction.in_link_ids.end(), [&](const SSD::SimString & item)
						{
							return item == strID.c_str();
						});
						if (iter == junction.in_link_ids.end())
						{

							junction.in_link_ids.push_back(std::move(strID.c_str()));
						}

						junction.lane_ids.push_back(laneid);
					}
				}

				for (auto& incomingRoadid : mjunction.incomingRoadIds)
				{
					SSD::SimStringVector laneidlist = HDMapStandalone::MHDMap::GetLaneList(incomingRoadid);
					for (auto& laneid : laneidlist)
					{

						int postion = std::string(laneid.GetString()).find_last_of('_');
						std::string strID = std::string(laneName.GetString()).substr(0, postion);
						auto iter = std::find_if(junction.out_link_ids.begin(), junction.out_link_ids.end(), [&](const SSD::SimString & item)
						{
							return item == strID.c_str();
						});
						if (iter == junction.out_link_ids.end())
						{
							junction.out_link_ids.push_back(std::move(strID.c_str()));
						}
					}
				}
			}

			auto iter = std::find_if(junctions.begin(), junctions.end(), [&](const Junction & item)
			{
				return item.id == junction.id;
			});
			if (iter == junctions.end())
			{
				junctions.push_back(std::move(junction));
			}
		}

		void Create_LogicalLayer(const SSD::SimPoint3D& pos, const double forward) {
			const auto& laneInfo = NDM_Util::GetForwardLaneInfo(pos, forward);
			const auto& idStr = laneInfo.currentLane;
			mCurrentLaneName = idStr;
			if (idStr.Empty())
			{
				return;
			}
			Create_VirtualLines(mLogicalLayer.virtuallines, laneInfo);
			Create_Lanes(mLogicalLayer.lanes, laneInfo);
			Create_ParkingSpaces(mLogicalLayer.parkingspaces, pos, forward);
			//for (auto lane : mLogicalLayer.lanes) {
			//	std::cout << "__________________" << lane.str_id.GetString() << "_____________________" << std::endl;
			//	for(auto offset: lane.attrs)
			//		std::cout <<"offset: "<< offset.offset << std::endl;
			//}
		}
	public:
		NDM_LogicalLayer mLogicalLayer;
		SSD::SimString mCurrentLaneName;
		map<SSD::SimString,bool> mVirtualLines;
	};
}