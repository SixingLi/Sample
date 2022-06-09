#pragma once

#include "horizon_map_env_ndm_data.h"

#include <iostream>

namespace HorizonMapEnv{

	typedef enum NaviAction {
		NaviAction_Invalid = 0,    // ��Ч����
		NaviAction_Straight = 1,   // ֱ��
		NaviAction_HalfRight = 2,  // ����ǰ����ʻ
		NaviAction_Right = 3,      // ��ת
		NaviAction_MuchRight = 4,  // ���Һ�ʻ
		NaviAction_Uturn = 5,      // ��ͷ�����ͷ���Ҳ����ת��
		NaviAction_Left = 6,       // ����
		NaviAction_HalfLeft = 7,   // ��ǰ��
		NaviAction_None = 8        // δ֪����
	}NDM_NaviAction_;

	typedef struct RoadRoute{
		SSD::SimString id;
		long long stamp;
		SSD::SimStringVector link_ids; // ������������ξ�����road_edges_nodes
		float time_cost_in_minutes;
		float distance_cost_in_meters;
	}NDM_RoadRoute;

	typedef struct LaneRoute{
		SSD::SimString id;
		long long stamp;
		SSD::SimStringVector lane_ids; // ������������ξ�����lanes
		SSD::SimString road_route_id; 	// ��ǰLaneRoute������RoadRoute
		float time_cost_in_minutes;
		float distance_cost_in_meters;
		unsigned int recommendation;     // true ǿ�Ƽ�,false ���Ƽ�
	}NDM_LaneRoute;


	// ��·�յ���Ϣ
	typedef struct NaviGuideInfo{
		NDM_NaviAction_ navi_action;	// ����������·���Ӵ��Ķ���
		float speed_limit;				// ��ǰ��·������
		float remain_distance;			// ����������·���Ӵ���ʣ�����
		SSD::SimString next_road_name;	// ������·����
	}NDM_NaviGuideInfo;

	// ��������
	typedef struct LaneOrientation{
		bool uturn_able=false;     // �Ƿ�ɵ�ͷ����
		bool left_able=false;      // �Ƿ����ת����
		bool straight_able=false;  // �Ƿ��ֱ�г���
		bool right_able=false;     // �Ƿ����ת����
		bool bus_only=false;       // ��������ר�õ�
		bool change_able=false;    // ��;�ɱ䳵��
		bool right_uturn=false;    // �Ƿ���۽�ͨ���򣩡���ת��ͷ���������ͣ�0-��ת��ͷ��1-��ת��ͷ
		bool highlight=false;      // �Ƿ��Ƽ�·�������ģ�������������
	}NDM_LaneOrientation;

	// �Ƽ�������Ϣ
	typedef struct RecommendLane{
	  unsigned int idx;                           // ������ţ��������0�ų���
	  NDM_LaneOrientation lane_orientation;       // ��������
	}NDM_RecommendLane;

	typedef struct  NaviLaneInfo{
		unsigned int total_lane_num;								// ��������
		SSD::SimVector<NDM_RecommendLane> recommend_lanes;			// �Ƽ�������Ϣ
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