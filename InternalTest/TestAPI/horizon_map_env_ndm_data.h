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
#include "public/common/MTopoGraph.h"

using namespace std;

#define M_PI 3.1415926
#define BOUNDARY_SAMPLE_DISTANCE 5
#define ROAD_SPLIT_MIN_LENGTH 3000
#define BACKWARD_DISTANCE 200

/*******************************************************************************
this is ndm convert from simone hdmap api, the origin data format is opendrive
@ the parameter of each struct have its default, some value is unavailable
@ while the long long type data value is -1 represernet the value is unavailable,
@ while the value of string type is empty string the value is unavailable
********************************************************************************/

namespace HorizonMapEnv {
	static std::map<std::string, int> mSignalMap =
	{
		{"1010200100001914" , 1},
		{"1010200200002012" , 2},
		{"1010200300002113" , 3},
		{"1010200500001513" , 5},
		{"1010203111001713" , 6},
		{"1010203200001713" , 7},
		{"1010204000001413" , 8},
		{"1010200600001413" , 9},
		{"1010201400001413" , 10},
		{"1010201300001413" , 11},
		{"1010200800001413" , 12},
		{"1010200900001413" , 13},
		{"1010201000001413" , 14},
		{"1010201100001413" , 15},
		{"1010201200001413" , 16},
		{"1010201600001413" , 18},
		{"1010202000001413" , 20},
		{"1010201800001413" , 21},
		{"1010201900001413" , 22},
		{"1010202100001413" , 23},
		{"1010202211001413" , 24},
		{"1010202311001413" , 25},
		{"1010202500001413" , 26},
		{"1010202400001413" , 27},
		{"1010202600001413" , 28},
		{"1010202700001413" , 29},
		{"1010202800001413" , 30},
		{"1010203300001413" , 31},
		{"1010203500001413" , 32},
		{"1010203400001413" , 33},
		{"1010203600001413" , 34},
		{"1010203700001413" , 35},
		{"1010203800001413" , 36},
		{"1010203900001613" , 37},
		{"1010200400001213" , 38},
		{"1010204200001413" , 40},
		{"1010100121001111" , 43},
		{"1010100122001111" , 44},
		{"1010100123001111" , 45},
		{"1010100111001111" , 46},
		{"1010100141001111" , 47},
		{"1010100133001111" , 48},
		{"1010100134001111" , 49},
		{"1010100131001111" , 50},
		{"1010100132001111" , 51},
		{"1010100211001111" , 52},
		{"1010100212001111" , 53},
		{"1010100312001111" , 54},
		{"1010100311001111" , 55},
		{"1010100400001111" , 56},
		{"1010100511001111" , 57},
		{"1010100512001111" , 58},
		{"1010100600001111" , 59},
		{"1010103100001111" , 60},
		{"1010103400001111" , 61},
		{"1010100711001111" , 62},
		{"1010100712001111" , 63},
		{"1010100713001111" , 64},
		{"1010100800001111" , 65},
		{"1010101700001111" , 66},
		{"1010101000001111" , 67},
		{"1010101100001111" , 68},
		{"1010102900001111" , 69},
		{"1010103000001111" , 70},
		{"1010101600001111" , 71},
		{"1010101200001111" , 72},
		{"1010101300001111" , 73},
		{"1010102100001111" , 74},
		{"1010103700001111" , 75},
		{"1010101400001111" , 76},
		{"1010101511001111" , 77},
		{"1010101512001111" , 78},
		{"1010101811001111" , 79},
		{"1010101812001111" , 80},
		{"1010102000001111" , 81},
		{"1010101911001111" , 82},
		{"1010101912001111" , 83},
		{"1010102200001111" , 84},
		{"1010102700001111" , 85},
		{"1010102300001111" , 87},
		{"1010100400001111" , 88},
		{"1010103900001111" , 89},
		{"1010102600001111" , 90},
		{"1010103500001111" , 91},
		{"1010102811001111" , 92},
		{"1010102812001111" , 93},
		{"1010103311001111" , 94},
		{"1010103312001111" , 95},
		{"1010103313001111" , 96},
		{"1010104111001111" , 97},
		{"1010104112001111" , 98},
		{"1010100900001111" , 99},
		{"1010103800001111" , 100},
		{"1010103900001111" , 101},
		{"1010100141001111" , 102},
		{"1010104400001111" , 104},
		{"1010104311001111" , 105},
		{"1010104312001111" , 106},
		{"1010104313001111" , 107},
		{"1010104314001111" , 108},
		{"1010300100002413" , 110},
		{"1010300200002413" , 111},
		{"1010300300002413" , 112},
		{"1010300400002413" , 113},
		{"1010300500002413" , 114},
		{"1010300600002413" , 115},
		{"1010300700002413" , 116},
		{"1010300800002413" , 117},
		{"1010301100002416" , 120},
		{"1010301300002413" , 121},
		{"1010301400002413" , 122},
		{"1010301500002413" , 123},
		{"1010302012002413" , 124},
		{"1010302014002413" , 125},
		{"1010301800002616" , 126},
		{"1010302017002416" , 131},
		{"1010302200002416" , 143},
		{"1010302011002416" , 144},
		{"1010301913002416" , 145},
		{"1010301915002416" , 146},
		{"1010301914002416" , 147},
		{"1010302013002416" , 151},
		{"1010302015002416" , 153},
		{"1010302111002416" , 156},
		{"1010301600002416" , 157},
		{"1010301911002416" , 159},
		{"1010301917002416" , 162},
		{ "1010301916002416" , 163 }
	};
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
		SSD::SimVector<double> cov;
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

	typedef enum TrafficSignType {
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
				color = NDM_LineColor::ORANGE;
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
			SSD::SimPoint3D targetPoint, dir;
#ifdef NDM_MAP_LOCAL
			HDMapStandalone::MLocation::GetNearMostLaneMiddlePoint(inputpt, targetPoint, dir);
#else
			SSD::SimString id;
			double s1, t1, s_toCenterLine1, t_toCenterLine1;
			if (SimOneAPI::GetNearMostLane(inputpt, id, s1, t1, s_toCenterLine1, t_toCenterLine1)) {
				SimOneAPI::GetLaneMiddlePoint(inputpt,id,targetPoint,dir);
			}
			else {
				std::cout << "there is not lane info in current input pt" << std::endl;
			}
#endif
			SSD::SimPoint3D dir_t(-dir.y, dir.x, dir.z);

			TyrePosInfo_ tyrePosInfo;
			tyrePosInfo.frontLeft = SSD::SimPoint3D(inputpt.x + dir.x * 1.5 + dir_t.x * 1, inputpt.y + dir.y * 1.5 + dir_t.y * 1, inputpt.z + dir.z * 1.5 + dir_t.z * 1);
			tyrePosInfo.frontRight = SSD::SimPoint3D(inputpt.x + dir.x * 1.5 - dir_t.x * 1, inputpt.y + dir.y * 1.5 - dir_t.y * 1, inputpt.z + dir.z * 1.5 - dir_t.z * 1);
			tyrePosInfo.rearLeft = SSD::SimPoint3D(inputpt.x - dir.x * 1.5 + dir_t.x * 1, inputpt.y - dir.y * 1.5 + dir_t.y * 1, inputpt.z - dir.z * 1.5 + dir_t.z * 1);
			tyrePosInfo.rearRight = SSD::SimPoint3D(inputpt.x - dir.x * 1.5 - dir_t.x * 1, inputpt.y - dir.y * 1.5 - dir_t.y * 1, inputpt.z - dir.z * 1.5 - dir_t.z * 1);

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
			auto& laneInfoList = HDMapStandalone::MHDMap::GetLaneSample(inputpt, idStr, forward);
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
				LaneSample_ sample;
				sample.laneCode = 0;
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

		static NDM_LaneMarkingType Get_LaneMarkingType(const SSD::SimString &SubType) {

			const char * sub_t = SubType.GetString();
			NDM_LaneMarkingType laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_UNKNOWN;
			if (SubType == "StraightAheadArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_FORWARD;
			}
			else if (SubType == "LeftOrRightTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_LEFT_AND_RIGHT;
			}
			else if (SubType == "LeftOrUTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_U_TURN_AND_LEFT;
			}
			else if (SubType == "LeftTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_LEFT;
			}
			else if (SubType == "LeftChangeArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_MERGE_LEFT;
			}
			else if (SubType == "RightChangeArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_MERGE_RIGHT;
			}
			else if (SubType == "RightTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_RIGHT;
			}
			else if (SubType == "StraightOrLeftTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_U_TURN_AND_LEFT;
			}
			else if (SubType == "StraightOrLeftOrRightTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_FORWARD_AND_LEFT_AND_RIGHT;
			}
			else if (SubType == "StraightOrRightTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_RIGHT_AND_FORWARD;
			}
			else if (SubType == "StraightOrUTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_U_TURN_AND_FORWARD;
			}
			else if (SubType == "UTurnArrow")
			{
				laneMarkingType = NDM_LaneMarkingType::LaneMarkingType_ARROW_U_TURN;
			}
			return laneMarkingType;

		}
		static NDM_TrafficSignType Get_SignalType(const SSD::SimString &SubType) {
			NDM_TrafficSignType current_sign;
			auto iter = mSignalMap.find(std::string(SubType.GetString()));
			if (iter != mSignalMap.end()) {
				current_sign = (NDM_TrafficSignType)iter->second;
			}
			else {
				current_sign = NDM_TrafficSignType::TrafficSignType_UNKNOWN;
			}
			return current_sign;
		}

		static double ConvertHeading(double angle) {
			double tempAngle;
			if (angle<0 && angle>(-M_PI)) {
				tempAngle = M_PI / 2 + abs(angle);
			}
			else if (0 <= angle && angle <= M_PI / 2) {
				tempAngle = M_PI / 2 - angle;
			}
			else {
				tempAngle = 5 * M_PI / 2 - angle;
			}
			tempAngle = (tempAngle / (2 * M_PI)) * 360;
			return tempAngle;
		}
		static long GetRoadID(const SSD::SimString &laneName) {
			
		}
	public:
	};
}

