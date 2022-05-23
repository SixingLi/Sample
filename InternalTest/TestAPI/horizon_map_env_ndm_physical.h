#pragma once

#include "horizon_map_env_ndm_data.h"

#include <iostream>

/******************************************objects objects objects ************************************/
namespace HorizonMapEnv {
	typedef enum ObjectType {
		ObjectType_Unknown = 0,
		ObjectType_TrafficSign = 1,
		ObjectType_TrafficLight = 2,
		ObjectType_TrafficLightBulb = 3,
		ObjectType_LaneMarking = 4,
		ObjectType_StopLine = 5,
		ObjectType_SpeedBump = 6,
		ObjectType_Pole = 7,
		ObjectType_CrossWalk = 8,
		ObjectType_Zone = 9,
		ObjectType_ParkingSlot = 10,
		ObjectType_TrafficCone = 11
	}NDM_ObjectType;

	typedef enum LineType {
		LineType_Unknown = 0,		// 0 << 0
		LineType_LaneLine = 2,		// 1 << 1
		LineType_Curb = 4,			// 1 << 2
		LineType_Center = 8,		// 1 << 3, Center line, virtual and in the median of lane
		LineType_Guardrail = 16,	// 1 << 4
		LineType_ConcreteBarrier = 32,  // 1 << 5
		LineType_Fence = 64,		// 1 << 6
		LineType_Wall = 128,		// 1 << 7
		LineType_Canopy = 256,		// 1 << 8
		LineType_Virtual = 512,		// 1 << 9
		LineType_Cone = 1024		// 1 << 10
	}NDM_LineType;

	// Object's subtype
	typedef enum TrafficLightType {
		TrafficLightType_UNKNOWN = 0,
		TrafficLightType_CIRCLE = 1,
		TrafficLightType_CROSS = 2,
		TrafficLightType_PEDESTRIAN = 3,
		TrafficLightType_BICYCLE = 4,
		TrafficLightType_ARROW = 5,
		TrafficLightType_TIME = 6,
		TrafficLightType_TEXT = 7,
		TrafficLightType_MULTI_LEN = 8
	}NDM_TrafficLightType;

	typedef enum LaneMarkingType {
		LaneMarkingType_UNKNOWN = 0,
		LaneMarkingType_ARROW_LEFT = 1,
		LaneMarkingType_ARROW_FORWARD = 2,
		LaneMarkingType_ARROW_RIGHT = 3,
		LaneMarkingType_ARROW_LEFT_AND_FORWARD = 4,
		LaneMarkingType_ARROW_RIGHT_AND_FORWARD = 5,
		LaneMarkingType_ARROW_LEFT_AND_RIGHT = 6,
		LaneMarkingType_ARROW_U_TURN = 7,
		LaneMarkingType_ARROW_U_TURN_AND_FORWARD = 8,
		LaneMarkingType_ARROW_U_TURN_AND_LEFT = 9,
		LaneMarkingType_ARROW_MERGE_LEFT = 10,
		LaneMarkingType_ARROW_MERGE_RIGHT = 11,
		LaneMarkingType_CROSSWALK_NOTICE = 12,
		LaneMarkingType_SPEED_LIMIT_LOW = 13,
		LaneMarkingType_SPEED_LIMIT_HIGH = 14,
		LaneMarkingType_ARROW_NO_LEFT_TURN = 15,
		LaneMarkingType_ARROW_NO_RIGHT_TURN = 16,
		LaneMarkingType_ARROW_NO_U_TURN = 17,
		LaneMarkingType_ARROW_FORWARD_AND_LEFT_AND_RIGHT = 18,
		LaneMarkingType_ARROW_FORWARD_AND_U_TURN_AND_LEFT = 19,
		LaneMarkingType_ARROW_RIGHT_AND_U_TURN = 20,
		LaneMarkingType_TEXT = 21,
		LaneMarkingType_TIME = 22,
		LaneMarkingType_CHECK_FOLLOWING_DISTANCE = 23,
		LaneMarkingType_STOP_TO_GIVE_WAY = 24,
		LaneMarkingType_SLOWDOWN_TO_GIVE_WAY = 25,
		LaneMarkingType_STOP_MARK = 26,
		LaneMarkingType_NETS = 27
	}NDM_LaneMarkingType;

	enum TrafficSignType {
		TrafficSignType_UNKNOWN = 0,
		// Prohibit
		TrafficSignType_P_Stop_For = 1,    // stop to give way
		TrafficSignType_P_Slow_For = 2,    // slow down and give way
		TrafficSignType_P_Give_Way = 3,    // give way
		TrafficSignType_P_Noway = 4,    // no way
		TrafficSignType_P_No_Entry = 5,    // no entry
		TrafficSignType_P_No_Parking = 6,    // no parking
		TrafficSignType_P_No_Long_Parking = 7,    // no long-term parking
		TrafficSignType_P_Parking_Check = 8,    // stop for check
		TrafficSignType_P_No_Motor = 9,    // no entry of motor vehicles
		TrafficSignType_P_No_Motorcycle = 10,   // no entry of motorcycles
		TrafficSignType_P_No_Truck = 11,   // no entry of truck
		TrafficSignType_P_No_Moto_Tricycle = 12,   // no entry of tricycles
		TrafficSignType_P_No_Bus = 13,   // no entry of Bus
		TrafficSignType_P_No_Car = 14,   // no entry of car
		TrafficSignType_P_No_Trailer = 15,   // no entry of trailer
		TrafficSignType_P_No_Tractor = 16,   // no entry of tractor
		TrafficSignType_P_No_Cargo_Tricycle = 17,   // no entry of cargo tricycles
		TrafficSignType_P_No_Non_Motor = 18,   // no entry of non-motor vehicles
		TrafficSignType_P_No_Animal_Vehicle = 19,   // no entry of animal vehicles
		TrafficSignType_P_No_Human_Vehicle = 20,   // no entry of rickshaws
		TrafficSignType_P_No_Human_Cargo_Triangle = 21,   // no entry of manpower cargo tricycles
		TrafficSignType_P_No_Human_Passenger_Triangle = 22,   // no entry of manpower passenger tricycles
		TrafficSignType_P_No_Human = 23,   // no entry of pedestrians
		TrafficSignType_P_No_Left_Turn = 24,   // no left turn
		TrafficSignType_P_No_Right_Turn = 25,   // no right turn
		TrafficSignType_P_No_Left_Right_Turn = 26,   // no turning left or right
		TrafficSignType_P_No_Forward = 27,   // no forward
		TrafficSignType_P_No_Forward_Left = 28,   // no forward and turn left
		TrafficSignType_P_No_Forward_Right = 29,   // no forward and turn right
		TrafficSignType_P_No_Return = 30,   // no u-turn
		TrafficSignType_P_No_Horning = 31,   // no horning
		TrafficSignType_P_Height_Lim = 32,   // height limit
		TrafficSignType_P_Width_Lim = 33,   // width limit
		TrafficSignType_P_Weight_Lim = 34,   // weight limit
		TrafficSignType_P_Weight_Lim_wheel = 35,   // limit of axle load
		TrafficSignType_P_Speed_Lim = 36,   // speed limit
		TrafficSignType_P_Speed_Lim_Rev = 37,   // remove the speed limit
		TrafficSignType_P_No_Passing = 38,   // no passing
		TrafficSignType_P_No_Dangerous = 39,   // no entry of dangerous goods vehicles
		TrafficSignType_P_Custom = 40,   // the customs
		TrafficSignType_P_Other = 41,   // other prohibitions

		// Warning
		TrafficSignType_W_T_Shap = 42,   // warning T-junction
		TrafficSignType_W_T_Shap_Left = 43,   // warning left T-junction
		TrafficSignType_W_T_Shap_Right = 44,   // warning right T-junction
		TrafficSignType_W_T_Shaps = 45,   // warning continuous T-junctions
		TrafficSignType_W_Cross = 46,   // warning intersection
		TrafficSignType_W_Circle = 47,   // warning roundabouts
		TrafficSignType_W_Y_Left = 48,   // warning the fork in the front left
		TrafficSignType_W_Y_Right = 49,   // warning the fork in the front right
		TrafficSignType_W_YB_Left = 50,   // warning the fork at the rear left
		TrafficSignType_W_YB_Right = 51,   // warning the fork at the rear right
		TrafficSignType_W_Left_Turn = 52,   // warning the sharp left turns
		TrafficSignType_W_Right_Turn = 53,   // warning the sharp right turns
		TrafficSignType_W_RL_Turn = 54,   // warning the reverse bend on the right
		TrafficSignType_W_LR_Turn = 55,   // warning the reverse bend on the left
		TrafficSignType_W_Continuous_Turn = 56,   // warning continuous bends
		TrafficSignType_W_Up = 57,   // warning uphill
		TrafficSignType_W_Down = 58,   // warning downhill
		TrafficSignType_W_Continuous_Down = 59,   // warning continuous downhill
		TrafficSignType_W_Accident_Prone = 60,   // warning accident-prone road sections
		TrafficSignType_W_Danger = 61,   // warning dangerous
		TrafficSignType_W_Left_Narrow = 62,   // warning the narrow on the left side
		TrafficSignType_W_Right_Narrow = 63,   // warning the narrow on the right side
		TrafficSignType_W_LR_Narrow = 64,   // warning the narrow on both sides
		TrafficSignType_W_Narrow_Bridge = 65,   // warning the narrow bridge
		TrafficSignType_W_Slip = 66,   // warning the slippery sections
		TrafficSignType_W_Pedestrain = 67,   // warning the pedestrian
		TrafficSignType_W_Children = 68,   // warning children
		TrafficSignType_W_Cycle = 69,   // warning non-motor vehicles
		TrafficSignType_W_Disabled = 70,   // warning disabled
		TrafficSignType_W_Side_Wind = 71,   // warning crosswinds
		TrafficSignType_W_Domestic = 72,   // warning livestock
		TrafficSignType_W_Animal = 73,   // warning wild animal
		TrafficSignType_W_Tunnel = 74,   // warning tunnel
		TrafficSignType_W_Tunnel_Headlight = 75,   // warning head lights in the tunnel
		TrafficSignType_W_Traffic_Light = 76,   // warning traffic light
		TrafficSignType_W_Left_Falling = 77,   // warning the failing rocks on the left side
		TrafficSignType_W_Right_Falling = 78,   // warning the failing rocks on the right side
		TrafficSignType_W_Mount_Left = 79,   // warning the mountain road on the left side
		TrafficSignType_W_Mount_Right = 80,   // warning the mountain road on the right side
		TrafficSignType_W_Village = 81,   // warning village
		TrafficSignType_W_Dam_Right = 82,   // warning dam on the right side
		TrafficSignType_W_Dam_Left = 83,   // warning dam on the left side
		TrafficSignType_W_Ferry = 84,   // warning ferry
		TrafficSignType_W_Ford = 85,   // warning ford
		TrafficSignType_W_Slow = 86,   // warning slow down
		TrafficSignType_W_Hump_Bridge = 87,   // warning the hump bridge
		TrafficSignType_W_Bumpy = 88,   // warning the bumpy roads
		TrafficSignType_W_Bump = 89,   // warning bumps
		TrafficSignType_W_Low_Lying = 90,   // warning low-lying roads
		TrafficSignType_W_Working = 91,   // warning construction
		TrafficSignType_W_Guarded_Railway = 92,   // warning the guarded railway
		TrafficSignType_W_Railway = 93,   // warning the unguarded railway
		TrafficSignType_W_Detour_Around = 94,   // warning detouring around
		TrafficSignType_W_Detour_Left = 95,   // warning detouring on the left
		TrafficSignType_W_Detour_Right = 96,   // warning detouring on the right
		TrafficSignType_W_Merge_Left = 97,   // warning merge on the left
		TrafficSignType_W_Merge_Right = 98,   // warning merge on the right
		TrafficSignType_W_Two_Way = 99,   // warning two-way traffic
		TrafficSignType_W_Tidal = 100,  // warning tidal
		TrafficSignType_W_Keep_Distance = 101,  // warning keeping the distance
		TrafficSignType_W_Cross_Intersection = 102,  // warning cross intersection
		TrafficSignType_W_T_Intersection = 103,  // warning T-intersection
		TrafficSignType_W_Vehicle_Queue = 104,  // warning the queue of vehicles
		TrafficSignType_W_Ice = 105,  // warning the icy roads
		TrafficSignType_W_Rain = 106,  // waring rain and snow weather
		TrafficSignType_W_Fog = 107,  // warning the fog
		TrafficSignType_W_Bad_Weather = 108,  // warning the bad weather
		TrafficSignType_W_Other = 109,  // other warnings

		// instruction
		TrafficSignType_I_Forward = 110,  // straight
		TrafficSignType_I_Left_Turn = 111,  // turn left
		TrafficSignType_I_Right_Turn = 112,  // turn right
		TrafficSignType_I_Forward_Left = 113,  // go straight and turn left
		TrafficSignType_I_Forward_Right = 114,  // go straight and turn right
		TrafficSignType_I_Left_Right = 115,  // turn left and right
		TrafficSignType_I_Right = 116,  // drive on the right
		TrafficSignType_I_Left = 117,  // drive on the left
		TrafficSignType_I_Forward_Left_Stereo = 118,  // go straight and turn left at an interchange
		TrafficSignType_I_Forward_Right_Stereo = 119,  // go straight and turn right at an interchange
		TrafficSignType_I_Circle = 120,  // driving around the island
		TrafficSignType_I_Walk = 121,  // walk
		TrafficSignType_I_Honk = 122,  // honk
		TrafficSignType_I_Min_Speed_Lim = 123,  // minium speed limit
		TrafficSignType_I_Motors = 124,  // motor vehicles
		TrafficSignType_I_Non_Motors = 125,  // non-motor vehicles
		TrafficSignType_I_Pedestrian_Cross = 126,  // crosswalk
		TrafficSignType_I_Other = 127,  // other instructions
		TrafficSignType_P_No_Passing_Rev = 128,  // remove the prohibition of no passing
		TrafficSignType_P_All_Speed_Limit_Cancel = 129,  // remove all speed limits
		TrafficSignType_W_Tripod = 130,  // warning the tripod
		TrafficSignType_P_MultiVehicle = 131,  // multivehicle
		TrafficSignType_P_No_Bike_Down_Slope = 132,  // no bike down slop
		TrafficSignType_P_No_Bike_Up_Slope = 133,  // no bike up slop
		TrafficSignType_P_No_Mini_bus = 134,  // no mini bus
		TrafficSignType_P_No_Specific_Left_Turn = 135,  // no specific left turn
		TrafficSignType_P_No_Specific_Right_Turn = 136,  // no specific right turn
		TrafficSignType_P_Speed_Lim_ele = 137,  // speed lim ele
		TrafficSignType_P_Vehicle_Other = 138,  // other vehicle
		TrafficSignType_P_Other_Speed_Lim_ele = 139,  // other speed lim ele
		TrafficSignType_W_Slow_Down = 140,  // warning slow down
		TrafficSignType_W_Split_Left = 141,  // warning split left
		TrafficSignType_W_Split_Right = 142,  // warning split right
		TrafficSignType_I_Allow_Uturn = 143,  // allow uturn
		TrafficSignType_I_Fast_Bus_Lane = 144,  // fast bus lane
		TrafficSignType_I_Forward_Lane = 145,  // forward lane
		TrafficSignType_I_Forward_Left_Lane = 146,  // forward left lane
		TrafficSignType_I_Forward_Right_Lane = 147,  // forward right lane
		TrafficSignType_I_Left_Bus_Lane = 148,  // left bus lane
		TrafficSignType_I_Left_Line = 149,  // left line
		TrafficSignType_I_Left_Turn_Lane = 150,  // left turn lane
		TrafficSignType_I_Motors_Lane = 151,  // motors lane
		TrafficSignType_I_Multi_Occupant_Motors_Lane = 152,  // multi occupant motors lane
		TrafficSignType_I_Non_Motors_Lane = 153,  // non motors lane
		TrafficSignType_I_Other_circle = 154,  // other circle
		TrafficSignType_I_Other_rectangle = 155,  // other rectangle
		TrafficSignType_I_Parking_Space = 156,  // parking space
		TrafficSignType_I_Priority_Intereesction = 157,  // Priority intereesction
		TrafficSignType_I_Right_Line = 158,  // right line
		TrafficSignType_I_Right_Turn_Lane = 159,  // right turn lane
		TrafficSignType_I_Split_Drive_Line = 160,  // split drive line
		TrafficSignType_I_Straight_Line = 161,  // straight line
		TrafficSignType_I_UTurn_And_Left_Lane = 162,  // Uturn and left lane
		TrafficSignType_I_UTurn_Lane = 163,  // Uturn lane
		TrafficSignType_I_Uturn = 164,  // Uturn
		TrafficSignType_Route_Sign_Misc = 166,  // Route Sign Misc
		TrafficSignType_Multi_Misc = 167,  // multi Misc
		TrafficSignType_GSWB_Misc = 168,  // GSWB Misc
		TrafficSignType_G_Guide_Sign_Green_White = 169,  // Guide Sign Green White
		TrafficSignType_G_Guide_Sign_White_Black = 170,  // Guide Sign White Black
		TrafficSignType_IR_Ramp = 171,  // Ramp
		TrafficSignType_IR_Ramp_Arrow = 172,  // Ramp Arrow
		TrafficSignType_IR_Zones = 173,  // IR_Zones
		TrafficSignType_IR_Zones_II = 174,  // Zones II
		TrafficSignType_R_Road_Sign_Blue_White = 175,  // Road Sign Blue White
		TrafficSignType_Route_ID_Other = 176,  // Other Route ID
		TrafficSignType_Route_ID_Red_White = 177,  // Route ID Red White
		TrafficSignType_Route_ID_Yellow_Black = 178,  // Yellow Black
		TrafficSignType_T_Travel_Sign_Brown_White = 179  // Travel Sign Brown White
	}NDM_TrafficSignType;


	typedef struct Object {
		int id = -1;				//-;
		SSD::SimString str_id;		// global id, UUID
		NDM_Polygon border;			// need
		NDM_ObjectType type;		// ObjectType need
		int sub_type;				// LaneMarkingType、TrafficSignType，etc. need
		int conf = -1;				//-
		int life_time = -1;			//-
		int age = -1;				//-
	}NDM_Object;


	typedef struct Objects {
		NDM_PerHeader header;
		SSD::SimVector<NDM_Object> objects;
		float conf_scale = 0.1;
	}NDM_Objects;


	/****************************************** Lines Lines Lines ************************************/

	typedef struct CurveLine {
		SSD::SimVector<NDM_Point> points;
		int color;  // LineColor unknown is 0, use ==
		int marking;  // LineMarking unknown is 0, use ==
	}NDM_CurveLine;

	typedef struct Line {
		SSD::SimString str_id;
		SSD::SimVector<CurveLine> lines_3d;  // 3D curve lines
		int type;   // LineType unknown is 0, use &
	}NDM_Line;

	typedef struct Lines {
		NDM_PerHeader header;
		SSD::SimVector <NDM_Line> lines;
	}NDM_Lines;

	typedef struct PhysicalLayer {
		/*************************************************
		交通灯，车道线，交通牌，杆子，交通锥
		车道标线，车道停止线，减速带，杆子，人行道，停车位
		**************************************************/
		NDM_Objects objects;
		/*************************************************
		车道线，路基缘石，虚拟车道中心线，车道线护栏，
		混泥土护栏，围栏
		**************************************************/
		NDM_Lines lines;
	}NDM_PhysicalLayer;


	/****************************************** create physical layer info ************************************/


	class NDM_PhysicalLayer_Creator {
	public:
		NDM_PhysicalLayer_Creator() {

		};

		//*********************获取和车道绑定object信息
		void Create_Objects(NDM_Objects& objects, const SSD::SimString &laneName)
		{
			//*********************获取和车道绑定的交通灯牌信息
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MSignal> trafficLight = HDMapStandalone::MLightAndSign::GetTrafficLightList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MSignal> trafficLight;
			SimOneAPI::GetSpecifiedLaneTrafficLightList(laneName, trafficLight);
#endif
			for (auto& signal : trafficLight)
			{
				NDM_Object object;
				object.str_id.SetString(std::to_string(signal.id).c_str());
				object.type = ObjectType_TrafficLight;
				//object.border.normal = signal.pt;
				object.border.orientation = { signal.heading.x,signal.heading.y,signal.heading.z ,0 };
				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{
					objects.objects.push_back(std::move(object));
				}
			}

			//*********************获取和车道绑定的交通标牌信息
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MSignal> signalList = HDMapStandalone::MLightAndSign::GetTrafficSignList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MSignal> signalList;
			SimOneAPI::GetSpecifiedLaneTrafficSignalList(laneName, signalList);
#endif
			for (auto& signal : signalList)
			{
				Object object;
				object.str_id.SetString(std::to_string(signal.id).c_str());
				object.type = ObjectType_TrafficSign;
				object.border.orientation = { signal.heading.x,signal.heading.y,signal.heading.z ,0 };
				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{
					objects.objects.push_back(std::move(object));
				}
			}

			//*********************获取和车道绑定的车道停止线信息
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MObject> stoplineList = HDMapStandalone::MLightAndSign::GetStoplineList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MObject> stoplineList;
			SimOneAPI::GetSpecifiedLaneStoplineList(laneName, stoplineList);
#endif
			for (auto& stopline : stoplineList)
			{
				Object object;
				object.str_id.SetString(std::to_string(stopline.id).c_str());
				object.type = ObjectType_StopLine;

				for (auto& point : stopline.boundaryKnots)
				{
					object.border.points.push_back({ point.x,point.y,point.z,0 });
				}
				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{
					objects.objects.push_back(std::move(object));
				}
			}

			//*********************获取和车道绑定的人行横道信息
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MObject> crosswalkList = HDMapStandalone::MLightAndSign::GetCrosswalkList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MObject> crosswalkList;
			SimOneAPI::GetSpecifiedLaneCrosswalkList(laneName, crosswalkList);
#endif
			for (auto& crosswalk : crosswalkList)
			{
				Object object;
				object.str_id.SetString(std::to_string(crosswalk.id).c_str());
				object.type = ObjectType_CrossWalk;

				for (auto& point : crosswalk.boundaryKnots)
				{
					object.border.points.push_back({ point.x,point.y,point.z,0 });
				}
				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{

					objects.objects.push_back(std::move(object));
				}
			}
		}

		void Create_Lines(NDM_Lines & lines, const SSD::SimString &laneName, const SSD::SimVector<HDMapStandalone::MLaneLineInfo> &laneLineInfo) {
			GetBoundarySampleList_(lines, laneName, laneLineInfo);
		}

		void GetBoundarySampleList_(NDM_Lines &lines, const SSD::SimString& laneName, const SSD::SimVector<HDMapStandalone::MLaneLineInfo>& laneLineInfo)
		{
			NDM_Line left;
			left.str_id.SetString((std::string(laneName.GetString()) + "_l").c_str());
			NDM_Line right;
			right.str_id.SetString((std::string(laneName.GetString()) + "_r").c_str());
			const char * mlanename = laneName.GetString();
			//std::cout << "L_boundaryInfo:" << (std::string(laneName.GetString()) + "_l") << std::endl;
			//std::cout << "R_boundaryInfo:" << (std::string(laneName.GetString()) + "_r") << std::endl;

			NDM_LineType type_left;
			type_left = LineType_LaneLine;
			left.type = type_left;

			NDM_LineType type_right;
			type_right = LineType_LaneLine;
			right.type = type_right;

			NDM_CurveLine lineleft, lineright;
			for (HDMapStandalone::MLaneLineInfo mLaneinfo : laneLineInfo)
			{
				if (laneName == mLaneinfo.laneName)
				{
					lineleft.marking = NDM_Util::GetMarking_(mLaneinfo.leftBoundary.roadmarkList[0]);  //TODO:
					lineright.marking = NDM_Util::GetMarking_(mLaneinfo.rightBoundary.roadmarkList[0]);  //TODO:
					lineleft.color = NDM_Util::GetColor_(mLaneinfo.leftBoundary.roadmarkList[0]);
					lineright.color = NDM_Util::GetColor_(mLaneinfo.rightBoundary.roadmarkList[0]);
					for (SSD::SimPoint3DVector points : mLaneinfo.leftBoundary.segmentList)
					{
						for (auto & segmentPonit : points) {
							lineleft.points.push_back({ segmentPonit.x,segmentPonit.y,segmentPonit.z,0 });
						}
					}
					for (SSD::SimPoint3DVector points : mLaneinfo.rightBoundary.segmentList)
					{
						for (auto & segmentPonit : points) {
							lineright.points.push_back({ segmentPonit.x,segmentPonit.y,segmentPonit.z,0 });
						}
					}
					break;
				}
			}

			left.lines_3d.push_back(std::move(lineleft));
			right.lines_3d.push_back(std::move(lineright));

			lines.lines.push_back(std::move(left));
			lines.lines.push_back(std::move(right));
		}

		void GetLaneLink_Physical(const SSD::SimString &idStr, const SSD::SimVector<HDMapStandalone::MLaneLineInfo> &laneLineInfo) {

#ifdef NDM_MAP_LOCAL
			auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(idStr);
#else
			HDMapStandalone::MLaneLink laneLink;
			SimOneAPI::GetLaneLink(idStr, laneLink);
#endif
			//predecessorLaneNameList
			//std::cout << "---------------predecessorLaneNameList-----------------" << std::endl;
			if (idStr == mCurrentLaneName) {
				for (SSD::SimString predName : laneLink.predecessorLaneNameList)
				{
					Create_Lines(mPhysicalLayer.lines, predName, laneLineInfo);
					Create_Objects(mPhysicalLayer.objects, predName);
					GetLaneLink_Physical(predName, laneLineInfo);
				}
			}

			//leftNeighborLane 
			//std::cout << "---------------leftNeighborLaneName-----------------" << std::endl;
			while (1)
			{
				SSD::SimString leftLane = laneLink.leftNeighborLaneName;

				if (leftLane.Empty())
				{
					break;
				}
				Create_Lines(mPhysicalLayer.lines, leftLane, laneLineInfo);
				Create_Objects(mPhysicalLayer.objects, leftLane);


#ifdef NDM_MAP_LOCAL
				laneLink = HDMapStandalone::MHDMap::GetLaneLink(leftLane);
#else
				SimOneAPI::GetLaneLink(leftLane, laneLink);
#endif
			}

			//rightNeighborLane
			//std::cout << "---------------rightNeighborLaneName-----------------" << std::endl;
			while (1)
			{
				auto& rightLane = laneLink.rightNeighborLaneName;
				if (rightLane.Empty())
				{
					break;
				}
				Create_Lines(mPhysicalLayer.lines, rightLane, laneLineInfo);
				Create_Objects(mPhysicalLayer.objects, rightLane);

#ifdef NDM_MAP_LOCAL
				laneLink = HDMapStandalone::MHDMap::GetLaneLink(rightLane);
#else
				SimOneAPI::GetLaneLink(rightLane, laneLink);
#endif
			}
		}

		void CreatePhysicalLayer(const SSD::SimPoint3D& pos, const double forward) {

#ifdef NDM_MAP_LOCAL
			auto& laneLineInfo = HDMapStandalone::MHDMap::GetLaneLineInfo();
#else
			SSD::SimVector<HDMapStandalone::MLaneLineInfo> laneLineInfo;
			SimOneAPI::GetLaneLineInfo(laneLineInfo);
#endif
			LaneInfo_t laneinfo = NDM_Util::GetForwardLaneInfo(pos, forward);
			const auto& idStr = laneinfo.currentLane;
			if (idStr.Empty())
			{
				return;
			}
			mCurrentLaneName = idStr;
			Create_Lines(mPhysicalLayer.lines, idStr,laneLineInfo);
			Create_Objects(mPhysicalLayer.objects, idStr);
			GetLaneLink_Physical(idStr,laneLineInfo);

			for (int i = 1; i < (int)laneinfo.dataList[0].laneNameList.size(); i++)
			{
				SSD::SimString & laneName = laneinfo.dataList[0].laneNameList[i];

				std::cout << "current NameLaneNmae: " << std::string(laneName.GetString()) << std::endl;
				Create_Lines(mPhysicalLayer.lines, laneName,laneLineInfo);
				Create_Objects(mPhysicalLayer.objects, laneName);
				GetLaneLink_Physical(laneName, laneLineInfo);
			}
		}

	public:
		NDM_PhysicalLayer mPhysicalLayer;
		SSD::SimString mCurrentLaneName;
	};
}