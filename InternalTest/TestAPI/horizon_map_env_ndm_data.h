#pragma once
#include <string>
#include <algorithm>
#include <iostream>
#include <map>

#include "SSD/SimPoint3D.h"
#include "SSD/SimPoint2D.h"
#include "SSD/SimString.h"
#ifdef NDM_MAP_LOCAL
#include "public/MHDMap.h"
#include "public/MLocation.h"
#include "public/MRouting.h"
#include "public/MLightAndSign.h"
#include "../Utils/UtilString.h"
#else
#include "SimOneHDMapAPI.h"
#include "Service/UtilString.h"
#endif
#include "public/common/MRoutePath.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MRoadMark.h"
#include "public/common/MRoadMark.h"


using namespace std;


#define M_PI 3.1415926
/*******************************************************************************
this is ndm convert from simone hdmap api, the origin data format is opendrive
@ the parameter of each struct have its default, some value is unavailable
@ while the long long type data value is -1 represernet the value is unavailable,
@ while the value of string type is empty string the value is unavailable
********************************************************************************/

namespace HorizonMapEnv {

	typedef enum DrivingSide {
		DrivingSide_RightHandDriving = 0,
		DrivingSide_LeftHandDriving = 1
	}E_NDM_DrivingSide;

	typedef enum UnitOfSystem {
		UnitOfSystem_Imperial = 0,
		UnitOfSystem_Metric = 1
	}E_NDM_UnitOfSystem;

	typedef struct GlobalData {
		unsigned int country_code = 156;    // 国家编码, 如: CHN:156
		unsigned int region_code = 0;    // 地区编码
		E_NDM_DrivingSide driving_side = DrivingSide_RightHandDriving;    // 驾驶方向, 左侧驾驶或右侧驾驶
		E_NDM_UnitOfSystem unit = UnitOfSystem_Imperial;    // 速度单位, km/h或mph
		unsigned int protocol_version = 1;    // 地图格式版本号 major.minor.sub, int表示方法: 2的24次方 * major + 2的16次方 * minor + sub
		unsigned int hardware_version = 0;    // 软件版本
		unsigned int map_version = 0;    // 地图数据版本号
		unsigned int map_age = 0;    // 地图更新时间
		unsigned int map_provider = 0;    // 地图供应商
		unsigned int map_status = 2;   // 地图状态
		int time_zone_offset = 8;   // 时区，东八区
		unsigned int mapping_version = 0;   // mapping软件版本: 3.2.0 (如果来自于自建地图, 即map_provider = MapProvider_provider_HORIZON)
		unsigned int aggregation_version = 0; // aggregation软件版本: 3.2.0 (如果来自自建地图, 即map_provider = MapProvider_provider_HORIZON)
		unsigned int ota_status = 0; // OTA 状态
	}NDM_GlobalData;


	typedef struct Point {
		double x;
		double y;
		double z;
		/// size = 3 or size = 9
		double *cov;
	}NDM_Point;

	typedef struct Polygon {
		// anticlockwise order（look to the front）
		SSD::SimVector<NDM_Point> points;
		// can be computed with the points（on the same plane）
		NDM_Point normal;
		// perpendicular to the normal
		float edgeline_width;
		NDM_Point orientation;
	}NDM_Polygon;

	typedef struct PerHeader {
		int cam_idx = 0;
		int frame_id = 0;
		int time_stamp = 0;
	}NDM_PerHeader;

	typedef enum LineMarking {
		LineMarking_Unknown = 0,
		LineMarking_SolidLine = 1,				// 单实线
		LineMarking_DashedLine = 2,				// 单虚线
		LineMarking_ShortDashedLine = 3,		// 短虚线
		LineMarking_DoubleSolidLine = 4,		// 双实线
		LineMarking_DoubleDashedLine = 5,		// 双虚线
		LineMarking_LeftSolidRightDashed = 6,   // 左实右虚
		LineMarking_RightSolidLeftDashed = 7,   // 右实左虚
		LineMarking_ShadedArea = 8,				// 导流线
		LineMarking_LaneVirtualMarking = 9,		// 车道虚拟线
		LineMarking_IntersectionVirualMarking = 10,   // 路口虚拟线
		LineMarking_CurbVirtualMarking = 11,	// 路边缘虚拟线
		LineMarking_UnclosedRoad = 12,			// 非封闭路段(线)
		LineMarking_RoadVirtualLine = 13,		// 道路虚拟线
		LineMarking_Other = 99
	}NDM_LineMarking;

	typedef enum LineColor {
		UNKNOWN_LINE_COLOR = 0,		// Color is not detected
		WHITE = 1,					// White color
		YELLOW = 2,					// Yellow color
		ORANGE = 3,					// 橙色
		BLUE = 4,					// 蓝色
		GREEN = 5,					// Green color
		GRAY = 6,					// Gray color
		LETF_GRAY_RIGHT_YELLOW = 7,	// 左白右黄
		LETF_YELLOW_RIGHT_WHITE = 8	// 左黄右白
	}NDM_LineColor;


	typedef struct TyrePosInfo
	{
		SSD::SimPoint3D frontLeft;
		SSD::SimPoint3D frontRight;
		SSD::SimPoint3D rearLeft;
		SSD::SimPoint3D rearRight;
	}TyrePosInfo_;

	typedef struct LaneSample
	{
		int laneCode;  //1, 2, ...
		bool inJunction = false;
		SSD::SimPoint3DVector leftBoundary;    //Left boundary sample data lane_line_left_data
		SSD::SimPoint3DVector rightBoundary;  //Right boundary sample data lane_line_right_data
		SSD::SimPoint3DVector centerLine;  //Right boundary sample data lane_line_right_data
	}LaneSample_;

	typedef struct LaneData
	{
		SSD::SimVector<LaneSample_> laneSampleList;
		SSD::SimStringVector laneNameList;
		HDMapStandalone::MRoadMark leftRoadMark;
		HDMapStandalone::MRoadMark rightRoadMark;
	}LaneData_;

	typedef struct LaneInfo
	{
		SSD::SimString currentLane;
		SSD::SimVector<LaneData_> dataList;
	}LaneInfo_t;

	class NDM_Util {
	public:
		static NDM_LineColor GetColor_(const HDMapStandalone::MRoadMark& roadMark)
		{
			NDM_LineColor color;
			switch (roadMark.color)
			{
			case HDMapStandalone::ERoadMarkColor::blue:
			{
				color = NDM_LineColor::BLUE;
			}
			break;
			case HDMapStandalone::ERoadMarkColor::green:
			{
				color = NDM_LineColor::GREEN;
			}
			break;
			case HDMapStandalone::ERoadMarkColor::red:
			{
				color = NDM_LineColor::UNKNOWN_LINE_COLOR;
			}
			break;
			case HDMapStandalone::ERoadMarkColor::white:
			{
				color = NDM_LineColor::WHITE;
			}
			break;
			case HDMapStandalone::ERoadMarkColor::yellow:
			{
				color = NDM_LineColor::YELLOW;
			}
			break;
			case HDMapStandalone::ERoadMarkColor::standard:
			{
				color = NDM_LineColor::UNKNOWN_LINE_COLOR;
			}
			break;
			default:
				break;
			}
			return color;
		}

		static NDM_LineMarking GetMarking_(const HDMapStandalone::MRoadMark& roadMark)
		{
			NDM_LineMarking marking;
			switch (roadMark.type)
			{
			case HDMapStandalone::ERoadMarkType::none:
			{
				marking = NDM_LineMarking::LineMarking_Unknown;
			}
			break;
			case HDMapStandalone::ERoadMarkType::solid:
			{
				marking = NDM_LineMarking::LineMarking_SolidLine;
			}
			break;
			case HDMapStandalone::ERoadMarkType::broken:
			{
				marking = NDM_LineMarking::LineMarking_DashedLine;
			}
			break;
			case HDMapStandalone::ERoadMarkType::solid_solid:
			{
				marking = NDM_LineMarking::LineMarking_DoubleSolidLine;
			}
			break;
			case HDMapStandalone::ERoadMarkType::solid_broken:
			{
				marking = NDM_LineMarking::LineMarking_LeftSolidRightDashed;
			}
			break;
			case HDMapStandalone::ERoadMarkType::broken_solid:
			{
				marking = NDM_LineMarking::LineMarking_RightSolidLeftDashed;
			}
			break;
			case HDMapStandalone::ERoadMarkType::broken_broken:
			{
				marking = NDM_LineMarking::LineMarking_DoubleDashedLine;
			}
			break;
			case HDMapStandalone::ERoadMarkType::curb:
			{
				marking = NDM_LineMarking::LineMarking_CurbVirtualMarking;
			}
			break;
			case HDMapStandalone::ERoadMarkType::grass:
			{
				marking = NDM_LineMarking::LineMarking_Other;
			}
			case HDMapStandalone::ERoadMarkType::botts_dots:
			{
				marking = NDM_LineMarking::LineMarking_Other;
			}
			break;
			default:
				break;
			}
			return marking;
		}

		static LaneInfo_t GetForwardLaneInfo(SSD::SimPoint3D inputpt, const double forward) {
			TyrePosInfo_ tyrePosInfo;
			tyrePosInfo.frontLeft = inputpt;
			tyrePosInfo.frontRight = inputpt;
			SSD::SimPoint3D pos2(inputpt.x - 3, inputpt.y, inputpt.z);
			tyrePosInfo.rearLeft = pos2;
			tyrePosInfo.rearRight = pos2;

			LaneInfo_t laneInfo;
			SSD::SimString idStr;
			double s, t, s_toCenterLine, t_toCenterLine;
			bool insideLane = false;
			bool drivingOnly = true;
#ifdef NDM_MAP_LOCAL
			if (HDMapStandalone::MLocation::GetNearMostLaneWithHeight_V2(inputpt, drivingOnly, idStr, s, t, s_toCenterLine, t_toCenterLine, insideLane))
#else
			if (SimOneAPI::GetNearMostLane(inputpt, idStr, s, t, s_toCenterLine, t_toCenterLine))
#endif
			{
				laneInfo.currentLane = idStr;
				std::cout << "idStr: " << idStr.GetString() << std::endl;
			}

			long juncId = -1;
#ifdef NDM_MAP_LOCAL
			if (HDMapStandalone::MHDMap::IsInJunction(idStr, juncId))
#else
			if (SimOneAPI::IsInJunction(idStr, juncId))
#endif
			{
				//For lanes in junction, need to cover all of them.
				//Only calculate laneSampleList.
				//
				SSD::SimStringVector laneList;
				SSD::SimPoint2D p1(tyrePosInfo.rearLeft.x, tyrePosInfo.rearLeft.y);
				SSD::SimPoint2D p2(tyrePosInfo.frontLeft.x, tyrePosInfo.frontLeft.y);
				SSD::SimPoint2D dir(tyrePosInfo.frontLeft.x - tyrePosInfo.rearLeft.x, tyrePosInfo.frontLeft.y - tyrePosInfo.rearLeft.y);
				dir.Normalize();
				double angle = NDM_Util::GetAngle_(SSD::SimPoint2D(1, 0), dir);
				NDM_Util::GetValidJunctionLanes_(inputpt, drivingOnly, angle, idStr, insideLane, juncId, laneList);
				//dataList
				//
				for (auto& laneId : laneList)
				{
					LaneData_ data;

					//laneSampleList
#ifdef NDM_MAP_LOCAL
					auto& laneInfoList = HDMapStandalone::MHDMap::GetLaneSample(inputpt, laneId, forward);
#else
					SSD::SimVector<HDMapStandalone::MLaneInfo> laneInfoList;
					SimOneAPI::GetForwardLaneSample(inputpt, laneId, forward, laneInfoList);
#endif
					for (auto& laneInfot : laneInfoList)
					{
						data.laneNameList.push_back(laneInfot.laneName);
					}
					data.laneSampleList = std::move(ToLaneSample(laneInfoList));

#ifdef NDM_MAP_LOCAL
					HDMapStandalone::MLocation::GetRoadMark(inputpt, laneId, data.leftRoadMark, data.rightRoadMark);
#else
					SimOneAPI::GetRoadMark(inputpt, laneId, data.leftRoadMark, data.rightRoadMark);
#endif
					laneInfo.dataList.push_back(data);
				}
				return std::move(laneInfo);
			}
			LaneData_ data;

			//laneSampleList
#ifdef NDM_MAP_LOCAL
			auto& laneInfoList = HDMapStandalone::MHDMap::GetLaneSample(inputpt, idStr, 100);
#else
			SSD::SimVector<HDMapStandalone::MLaneInfo> laneInfoList;
			SimOneAPI::GetForwardLaneSample(inputpt, idStr, forward, laneInfoList);
#endif
			for (auto& laneInfot : laneInfoList)
			{
				const char * mlaneinfo = laneInfot.laneName.GetString();
				data.laneNameList.push_back(laneInfot.laneName);
			}
			data.laneSampleList = std::move(NDM_Util::ToLaneSample(laneInfoList));

			//laneLineTypeInfo
#ifdef NDM_MAP_LOCAL
			HDMapStandalone::MLocation::GetRoadMark(inputpt, idStr, data.leftRoadMark, data.rightRoadMark);
#else
			SimOneAPI::GetRoadMark(inputpt, idStr, data.leftRoadMark, data.rightRoadMark);
#endif
			laneInfo.dataList.push_back(data);
			return std::move(laneInfo);
		}

		static SSD::SimVector<LaneSample_> ToLaneSample(const SSD::SimVector<HDMapStandalone::MLaneInfo>& laneSampleList)
		{
			SSD::SimVector<LaneSample> ret;

			for (auto& item : laneSampleList)
			{
				codeLane++;
				LaneSample_ sample;
				sample.laneCode = codeLane;
				long juncId = -1;
#ifdef NDM_MAP_LOCAL
				sample.inJunction = HDMapStandalone::MHDMap::IsInJunction(item.laneName, juncId);
#else
				sample.inJunction = SimOneAPI::IsInJunction(item.laneName, juncId);
#endif
				sample.leftBoundary = std::move(item.leftBoundary);
				sample.rightBoundary = std::move(item.rightBoundary);
				sample.centerLine = std::move(item.centerLine);
				ret.push_back(sample);
			}
			return std::move(ret);
		}

		static double dot_(const SSD::SimPoint2D& lhs, const SSD::SimPoint2D& rhs)
		{
			return lhs.x * rhs.x + lhs.y * rhs.y;
		}

		static double AngleBetween_(const SSD::SimPoint2D& p1, const SSD::SimPoint2D& p2)
		{
			double dotVal = dot_(p1, p2);
			if (std::fabs(dotVal) > 1)
			{
				dotVal = dotVal > 0 ? 1 : -1;
			}
			return std::acos(dotVal);
		}

		static double Cross_(const SSD::SimPoint2D& p1, const SSD::SimPoint2D& p2, const SSD::SimPoint2D& p3, const SSD::SimPoint2D& p4)
		{
			return (p2.x - p1.x)*(p4.y - p3.y) - (p2.y - p1.y)*(p4.x - p3.x);
		}


		static double GetAngle_(const SSD::SimPoint2D& dirFrom, const SSD::SimPoint2D& dirTo)
		{
			double angle = AngleBetween_(dirFrom, dirTo);
			const SSD::SimPoint2D p0;
			const auto& cross = Cross_(p0, dirFrom, p0, dirTo);
			if (cross < 0)
			{
				angle = -angle;  //Just switch sign
			}
			return angle;
		}

		static void GetValidJunctionLanes_(const SSD::SimPoint3D& pos, bool drivingOnly, const double& heading, const SSD::SimString& nearMostLane, bool insideNearMost,
			const long& junctionId, SSD::SimStringVector& laneList)
		{
			const double pi = M_PI;
			const double kHeightGap = 2.0;
			const double kRadius = 3;
			double angleShift = pi / 6;

			SSD::SimStringVector laneNameList;
#ifdef NDM_MAP_LOCAL
			if (!HDMapStandalone::MLocation::GetNearLanes(pos, kRadius, heading, angleShift, laneNameList))
#else
			if (!SimOneAPI::GetNearLanesWithAngle(pos, kRadius, heading, angleShift, laneNameList))
#endif
			{
				return;
			}

			SSD::SimPoint2D headingDir(std::cos(heading), std::sin(heading));
			SSD::SimPoint3D targetPoint, dir;
			SSD::SimPoint2D dir2D;
			for (auto& laneName : laneNameList)
			{
#ifdef NDM_MAP_LOCAL
				if (drivingOnly && !HDMapStandalone::MHDMap::IsDriving(laneName))
#else
				if (drivingOnly && !SimOneAPI::IsDriving(laneName))
#endif
				{
					continue;
				}
				long juncId;
#ifdef NDM_MAP_LOCAL
				bool inJunc = HDMapStandalone::MHDMap::IsInJunction(laneName, juncId);
#else
				bool inJunc = SimOneAPI::IsInJunction(laneName, juncId);
#endif
				if (!inJunc || juncId != junctionId)
				{
					continue;
				}

#ifdef NDM_MAP_LOCAL
				if (!HDMapStandalone::MLocation::GetLaneMiddlePoint(pos, laneName, targetPoint, dir))
#else
				if (!SimOneAPI::GetLaneMiddlePoint(pos, laneName, targetPoint, dir))
#endif
				{
					continue;
				}

				if (std::abs(targetPoint.z - pos.z) > kHeightGap)
				{
					continue;
				}
				dir2D.x = dir.x;
				dir2D.y = dir.y;
				dir2D.Normalize();
				double angleBetween = AngleBetween_(headingDir, dir2D);
				if (std::abs(angleBetween) > pi / 5)
				{
					continue;
				}

				if (std::string(laneName.GetString()) == std::string(nearMostLane.GetString()))
				{
					laneList.push_back(laneName);
					continue;
				}
				HDMapStandalone::MSideState sideState;
				//If nearMost is already inside, no need to add more that vehicle is outside of the lane.
#ifdef NDM_MAP_LOCAL
				if (insideNearMost && !HDMapStandalone::MLocation::IsInsideLane(pos, laneName, sideState))
#else
				if (insideNearMost && !SimOneAPI::IsInsideLane(pos, laneName, sideState))
#endif
				{
					continue;
				}

				laneList.push_back(laneName);
			}
		}

		static double GetCurvature_(SSD::SimPoint3D pt1, SSD::SimPoint3D pt2, SSD::SimPoint3D pt3) {
			double curvity;
			if ((abs(pt1.x - pt2.x) < 1e-3 && abs(pt1.x - pt3.x) < 1e-3) || (abs(pt1.y - pt2.y) < 1e-3 && abs(pt1.y - pt3.y) < 1e-3))
			{
				curvity = 0;
			}
			else
			{
				double dis1, dis2, dis3;
				double cosA, sinA, dis;
				dis1 = sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
				dis2 = sqrt((pt1.x - pt3.x)*(pt1.x - pt3.x) + (pt1.y - pt3.y)*(pt1.y - pt3.y));
				dis3 = sqrt((pt2.x - pt3.x)*(pt2.x - pt3.x) + (pt2.y - pt3.y)*(pt2.y - pt3.y));
				dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
				cosA = dis / (2 * dis1*dis3);
				sinA = sqrt(1 - cosA * cosA);
				curvity = 0.5*dis2 / sinA;
				curvity = 1 / curvity;
			}
			return curvity;
		}

		static double Get_Slop_Banking_(SSD::SimPoint3D pt_left, SSD::SimPoint3D pt_right) {
			double delta_z = pt_right.z - pt_left.z;
			double dis = sqrt((pt_left.x - pt_right.x)*(pt_left.x - pt_right.x) + (pt_left.y - pt_right.y)*(pt_left.y - pt_right.y));
			if (dis > 1e-3) {
				return atan(delta_z / dis) / M_PI * 180;
			}
			return 0;
		}
		public:
			static int codeLane;
	};
	int NDM_Util::codeLane = 0;

}

