#pragma once

#include "horizon_map_env_ndm_data.h"

#include <iostream>

namespace HorizonMapEnv{

	typedef enum NaviAction {
		NaviAction_Invalid = 0,    // 无效动作
		NaviAction_Straight = 1,   // 直行
		NaviAction_HalfRight = 2,  // 向右前方行驶
		NaviAction_Right = 3,      // 右转
		NaviAction_MuchRight = 4,  // 向右后方驶
		NaviAction_Uturn = 5,      // 掉头（左掉头，右侧叫右转）
		NaviAction_Left = 6,       // 向左
		NaviAction_HalfLeft = 7,   // 左前方
		NaviAction_None = 8        // 未知动作
	}NDM_NaviAction_;

	typedef struct RoadRoute{
		SSD::SimString id;
		long long stamp;
		SSD::SimStringVector link_ids; // 导航结果中依次经过的road_edges_nodes
		float time_cost_in_minutes;
		float distance_cost_in_meters;
	}NDM_RoadRoute;

	typedef struct LaneRoute{
		SSD::SimString id;
		long long stamp;
		SSD::SimStringVector lane_ids; // 导航结果中依次经过的lanes
		SSD::SimString road_route_id; 	// 当前LaneRoute关联的RoadRoute
		float time_cost_in_minutes;
		float distance_cost_in_meters;
		unsigned int recommendation;     // true 强推荐,false 弱推荐
	}NDM_LaneRoute;


	// 道路诱导信息
	typedef struct NaviGuideInfo{
		NDM_NaviAction_ navi_action;	// 到达下条道路连接处的动作
		float speed_limit;				// 当前道路的限速
		float remain_distance;			// 到达下条道路连接处的剩余距离
		SSD::SimString next_road_name;	// 下条道路名称
	}NDM_NaviGuideInfo;

	// 车道属性
	typedef struct LaneOrientation{
		bool uturn_able=false;     // 是否可掉头车道
		bool left_able=false;      // 是否可左转车道
		bool straight_able=false;  // 是否可直行车道
		bool right_able=false;     // 是否可右转车道
		bool bus_only=false;       // 公共汽车专用道
		bool change_able=false;    // 用途可变车道
		bool right_uturn=false;    // 是否（香港交通规则）“右转掉头”车道类型，0-左转掉头，1-右转掉头
		bool highlight=false;      // 是否（推荐路径经过的）“高亮”车道
	}NDM_LaneOrientation;

	// 推荐车道信息
	typedef struct RecommendLane{
	  unsigned int idx;                           // 车道编号，最左侧是0号车道
	  NDM_LaneOrientation lane_orientation;       // 车道属性
	}NDM_RecommendLane;

	typedef struct  NaviLaneInfo{
		unsigned int total_lane_num;								// 车道数量
		SSD::SimVector<NDM_RecommendLane> recommend_lanes;			// 推荐车道信息
	}NDM_NaviLaneInfo;

	typedef struct RoadNavi{
		SSD::SimString id;
		long long stamp;
		NDM_NaviGuideInfo navi_guide_info;
		NDM_NaviLaneInfo navi_lane_info;
	}NDM_RoadNavi;

	typedef struct Navigation{
		SSD::SimString id;
		long long stamp;
		NDM_Point start_location;
		NDM_Point destination;
		SSD::SimVector<NDM_RoadRoute> road_routes;
		SSD::SimVector<NDM_LaneRoute> lane_routes;
		NDM_RoadNavi road_navi;
		// navigation status
		// NavStatus_NoNavigation = 0, no navigation data
		// NavStatus_Normal = 1, navigation normal
		// NavStatus_Abnormal = 2, navigation abnormal
		unsigned int status;
		// navigation diagnosis code when status is NavStatus_NoNavigation
		// ERROR_NOT_RECEIVED = 0, no navigation results received
		// ERROR_MAPPING_FAILED = 1, SD and HD navigation results failed to match 
		// ERROR_NAV_FINISH = 2, end of the navigation

		// navigation diagnosis code when status is NavStatus_Abnormal
		// ERROR_OUT_OF_ROUTE = 10, out of the route
		unsigned int diag_code;
	}NDM_Navigation;

	class Navigation_Creator {
	public:
		Navigation_Creator() {};
		~Navigation_Creator() {};

		void GetRoadRoute_(const HDMapStandalone::MRoutePath &path) {
			NDM_RoadRoute roadRoute;
			std::string roadRouteId = "";
			std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
			for (auto segmentInfo : path.segmentInfos) {
				roadRouteId += std::to_string(segmentInfo.roadId);
				roadRoute.link_ids.push_back(std::to_string(segmentInfo.roadId).c_str());
			}
			std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
			roadRoute.distance_cost_in_meters = path.waypoints.size();
			roadRoute.id = roadRouteId.c_str();
			mNavigation.road_routes.push_back(roadRoute);
		}

		void GetLaneRoute_(const HDMapStandalone::MRoutePath &path,
			const SSD::SimVector<HDMapStandalone::MRoutePoint> &routePtList) {

			NDM_LaneRoute laneRoute;
			std::string laneRouteId = "";
			for (auto route_pt_ : routePtList) {
				laneRouteId += route_pt_.laneId.ToString().GetString();
				laneRoute.lane_ids.push_back(route_pt_.laneId.ToString());
			}
			laneRoute.id = laneRouteId.c_str();
			laneRoute.road_route_id = mNavigation.road_routes[0].id;
			laneRoute.recommendation = 1;
			bool isInJunction = false;
			long junctionId = -1;
			HDMapStandalone::MLaneId LaneIdInJunction;
			for(auto laneName:routePtList){
				
#ifdef NDM_MAP_LOCAL
				isInJunction = HDMapStandalone::MHDMap::IsInJunction(laneName.laneId.ToString(), junctionId);
#else
				isInJunction = SimOneAPI::IsInJunction(laneName.laneId.ToString(), junctionId);
#endif
				if (isInJunction) {
					LaneIdInJunction = laneName.laneId;
					break;
				}
			}
			if (isInJunction) {
				for (auto segmentInfo : path.segmentInfos) {

					if (LaneIdInJunction.roadId == segmentInfo.roadId) {
						if (segmentInfo.from >= LANE_ROUTE_REMOMMENDATION_DISTANCE) {
						
							laneRoute.recommendation = 2;
						}
						else if (segmentInfo.from < LANE_ROUTE_REMOMMENDATION_DISTANCE) {
							laneRoute.recommendation = 3;
						}
						break;
					}
				}
			}			
			laneRoute.distance_cost_in_meters = path.waypoints.size();
			mNavigation.lane_routes.push_back(laneRoute);
		}

		void GetRoadNavi_() {
			
		}

		void CreateNavigation(const SSD::SimPoint3DVector& inputPoints) {

			if (inputPoints.size() < 2) {
				return;
			}
			NDM_Point point_pt_start, point_pt_end;
			int end_index = inputPoints.size() - 1;
			point_pt_start.x = inputPoints[0].x; point_pt_start.y = inputPoints[0].y; point_pt_start.z = inputPoints[0].z;
			point_pt_end.x = inputPoints[end_index].x; point_pt_end.y = inputPoints[end_index].y; point_pt_end.z = inputPoints[end_index].z;
			
			mNavigation.start_location = point_pt_start;
			mNavigation.destination = point_pt_end;

			SSD::SimVector<int> indexOfValidPoints;
			HDMapStandalone::MRoutePath path;
			SSD::SimVector<HDMapStandalone::MRoutePoint> routePtList;

#ifdef NDM_MAP_LOCAL
			if (HDMapStandalone::MRouting::GenerateRoute(inputPoints, indexOfValidPoints, path, routePtList))
#else
			if (SimOneAPI::GenerateRoute_V2(inputPoints, indexOfValidPoints, path, routePtList))
#endif
			{
				GetRoadRoute_(path);
				GetLaneRoute_(path,routePtList);
			}
			else {
				std::cout << "Fail to GenerateRoute from inputPoints!! " << std::endl;
			}
		}

	public:
		NDM_Navigation mNavigation;
	};
}