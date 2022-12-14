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

	typedef struct Object {
		int id = -1;				//-;
		SSD::SimString str_id;		// global id, UUID
		NDM_Polygon border;			// need
		NDM_ObjectType type;		// ObjectType need
		int sub_type=0;				// LaneMarkingType¡¢TrafficSignType£¬etc. need
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
		½»Í¨µÆ£¬³µµÀÏß£¬½»Í¨ÅÆ£¬¸Ë×Ó£¬½»Í¨×¶
		³µµÀ±êÏß£¬³µµÀÍ£Ö¹Ïß£¬¼õËÙ´ø£¬¸Ë×Ó£¬ÈËÐÐµÀ£¬Í£³µÎ»
		**************************************************/
		NDM_Objects objects;
		/*************************************************
		³µµÀÏß£¬Â·»ùÔµÊ¯£¬ÐéÄâ³µµÀÖÐÐÄÏß£¬³µµÀÏß»¤À¸£¬
		»ìÄàÍÁ»¤À¸£¬Î§À¸
		**************************************************/
		NDM_Lines lines;
	}NDM_PhysicalLayer;


	/****************************************** create physical layer info ************************************/


	class NDM_PhysicalLayer_Creator {
	public:
		NDM_PhysicalLayer_Creator() {

		};

		//*********************»ñÈ¡ºÍ³µµÀ°ó¶¨objectÐÅÏ¢
		void Create_Objects(NDM_Objects& objects, const SSD::SimString &laneName)
		{

			//*********************»ñÈ¡ºÍ³µµÀ°ó¶¨µØÃæ±ê¼ÇÐÅÏ¢
#ifdef NDM_MAP_LOCAL
			const auto& graphicsSignals = HDMapStandalone::MLightAndSign::GetSignalListOnLaneByType(laneName, SSD::SimString("Graphics"));
#else
			SSD::SimVector<HDMapStandalone::MSignal> graphicsSignals;
			SimOneAPI::GetSignalListOnLaneByType(laneName, SSD::SimString("Graphics"), graphicsSignals);
#endif
			for (auto signal : graphicsSignals) {
			
				NDM_Object object;
				object.str_id.SetString(("lanemarking_" + std::to_string(signal.id)).c_str());
				object.type = ObjectType_LaneMarking;
				object.sub_type = (int)NDM_Util::Get_LaneMarkingType(signal.subType);
				NDM_Point point_orientation;
				point_orientation.x = signal.heading.x; point_orientation.y = signal.heading.y; point_orientation.z = signal.heading.z;
				object.border.orientation = point_orientation;
				NDM_Point point_pt;
				point_pt.x = signal.pt.x; point_pt.y = signal.pt.y; point_pt.z = signal.pt.z;
				object.border.points.push_back(point_pt);
				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{
					objects.objects.push_back(std::move(object));
				}
			}

			//*********************»ñÈ¡ºÍ³µµÀ°ó¶¨µÄ½»Í¨µÆÐÅÏ¢
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MSignal> trafficLight = HDMapStandalone::MLightAndSign::GetTrafficLightList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MSignal> trafficLight;
			SimOneAPI::GetSpecifiedLaneTrafficLightList(laneName, trafficLight);
#endif
			for (auto& signal : trafficLight)
			{
				NDM_Object object;
				object.str_id.SetString(("traffic_"+std::to_string(signal.id)).c_str());
				object.type = ObjectType_TrafficLight;
				//object.border.normal = signal.pt;
				NDM_Point point_orientation;
				point_orientation.x = signal.heading.x; point_orientation.y = signal.heading.y; point_orientation.z = signal.heading.z;
				object.border.orientation = point_orientation;
				NDM_Point point_pt;
				point_pt.x = signal.pt.x; point_pt.y = signal.pt.y; point_pt.z = signal.pt.z;
				object.border.points.push_back(point_pt);

				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{
					objects.objects.push_back(std::move(object));
				}
			}

			//*********************»ñÈ¡ºÍ³µµÀ°ó¶¨µÄ½»Í¨±êÅÆÐÅÏ¢
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MSignal> signalList = HDMapStandalone::MLightAndSign::GetTrafficSignList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MSignal> signalList;
			SimOneAPI::GetSpecifiedLaneTrafficSignalList(laneName, signalList);
#endif
			for (auto& signal : signalList)
			{
				NDM_Object object;
				object.str_id.SetString(("signal_"+std::to_string(signal.id)).c_str());
				object.type = ObjectType_TrafficSign;
				object.sub_type = (int)NDM_Util::Get_SignalType(signal.type);

				NDM_Point point_orientation;
				point_orientation.x = signal.heading.x; point_orientation.y = signal.heading.y; point_orientation.z = signal.heading.z;
				object.border.orientation = point_orientation;
				NDM_Point point_pt;
				point_pt.x = signal.pt.x; point_pt.y = signal.pt.y; point_pt.z = signal.pt.z;
				object.border.points.push_back(point_pt);
				auto iter = std::find_if(objects.objects.begin(), objects.objects.end(), [&](const Object & item)
				{
					return item.str_id == object.str_id;
				});
				if (iter == objects.objects.end())
				{
					objects.objects.push_back(std::move(object));
				}
			}

			//*********************»ñÈ¡ºÍ³µµÀ°ó¶¨µÄ³µµÀÍ£Ö¹ÏßÐÅÏ¢
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MObject> stoplineList = HDMapStandalone::MLightAndSign::GetStoplineList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MObject> stoplineList;
			SimOneAPI::GetSpecifiedLaneStoplineList(laneName, stoplineList);
#endif
			for (auto& stopline : stoplineList)
			{
				NDM_Object object;
				object.str_id.SetString(("stopline_"+std::to_string(stopline.id)).c_str());
				object.type = ObjectType_StopLine;

				for (auto& point : stopline.boundaryKnots)
				{
					NDM_Point point_pt;
					point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
					object.border.points.push_back(point_pt);
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

			//*********************»ñÈ¡ºÍ³µµÀ°ó¶¨µÄÈËÐÐºáµÀÐÅÏ¢
#ifdef NDM_MAP_LOCAL
			SSD::SimVector<HDMapStandalone::MObject> crosswalkList = HDMapStandalone::MLightAndSign::GetCrosswalkList(laneName);
#else
			SSD::SimVector<HDMapStandalone::MObject> crosswalkList;
			SimOneAPI::GetSpecifiedLaneCrosswalkList(laneName, crosswalkList);
#endif
			for (auto& crosswalk : crosswalkList)
			{
				NDM_Object object;
				object.str_id.SetString(("crosswalk_"+std::to_string(crosswalk.id)).c_str());
				object.type = ObjectType_CrossWalk;

				for (auto& point : crosswalk.boundaryKnots)
				{
					NDM_Point point_pt;
					point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
					object.border.points.push_back(point_pt);
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
			auto iter = mLaneIds.find(laneName);
			if (iter == mLaneIds.end()) {
				mLaneIds[laneName] = true;
			}
			else {
				return;
			}

			NDM_Line left;
			left.str_id.SetString((std::string(laneName.GetString()) + "_l").c_str());
			NDM_Line right;
			right.str_id.SetString((std::string(laneName.GetString()) + "_r").c_str());

			NDM_Line center;
			center.str_id.SetString((std::string(laneName.GetString()) + "_c").c_str());

			const char * mlanename = laneName.GetString();

			NDM_LineType type_left;
			type_left = NDM_LineType::LineType_LaneLine;
			left.type = type_left;

			NDM_LineType type_right;
			type_right = NDM_LineType::LineType_LaneLine;
			right.type = type_right;

			NDM_LineType type_center;
			type_center = NDM_LineType::LineType_Center;
			center.type = type_center;

			double s_road, t_road, z_road;
			bool isGetSTSuccess = false;
			long sample_point_count = 0;

#ifdef NDM_MAP_LOCAL
			isGetSTSuccess = HDMapStandalone::MLocation::GetST(mCurrentLaneName, mCurrentPos, s_road, t_road, z_road);
#else
			isGetSTSuccess = SimOneAPI::GetRoadST(mCurrentLaneName, mCurrentPos, s_road, t_road, z_road);
#endif
			NDM_CurveLine lineleft, lineright, linecenter;
			for (auto & mLaneinfo : laneLineInfo)
			{

				if (mCurrentLaneLength > ROAD_SPLIT_MIN_LENGTH) {
					std::vector<std::string> splitItem_Current = UtilString::split(mCurrentLaneName.GetString(), "_");
					std::vector<std::string> splitItem_LaneName = UtilString::split(laneName.GetString(), "_");

					if (laneName == mLaneinfo.laneName && splitItem_Current[0] == splitItem_LaneName[0] && isGetSTSuccess) {
#ifdef NDM_MAP_LOCAL
						HDMapStandalone::MLaneInfo cLaneinfo = HDMapStandalone::MHDMap::GetLaneSample(mLaneinfo.laneName);
#else
						HDMapStandalone::MLaneInfo cLaneinfo;
						SimOneAPI::GetLaneSample(mLaneinfo.laneName, cLaneinfo);
#endif
						lineleft.marking = NDM_Util::GetMarking_(mLaneinfo.leftBoundary.roadmarkList[0]);  //TODO:
						lineright.marking = NDM_Util::GetMarking_(mLaneinfo.rightBoundary.roadmarkList[0]);  //TODO:
						linecenter.marking = NDM_LineMarking::LineMarking_LaneVirtualMarking;

						lineleft.color = NDM_Util::GetColor_(mLaneinfo.leftBoundary.roadmarkList[0]);
						lineright.color = NDM_Util::GetColor_(mLaneinfo.rightBoundary.roadmarkList[0]);
						linecenter.color = NDM_LineColor::UNKNOWN_LINE_COLOR;

						int nearstIndex = 0;
						float distMin = std::numeric_limits<float>::max();
						sample_point_count = 0;
						for (int i = 0;i< cLaneinfo.centerLine.size();i++)
						{
							auto &point = cLaneinfo.centerLine[i];
							if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
								float dist_temp = sqrt((point.x - mCurrentPos.x)*(point.x - mCurrentPos.x) + (point.y - mCurrentPos.y)*(point.y - mCurrentPos.y));
								if (distMin > dist_temp) {
									distMin = dist_temp;
									nearstIndex = i;
								}
							}
							sample_point_count++;
						}
						sample_point_count = 0;
						if (atoi(splitItem_LaneName[2].c_str())< 0) { //ÍùnearstIndex+ºóÕÒ2000m£¬ÍùnearstIndex-Ç°ÕÒ200m
							for (int i = nearstIndex; i < cLaneinfo.centerLine.size(); i++)
							{
								auto &point = cLaneinfo.centerLine[i];
								if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
									NDM_Point point_pt;
									point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
									linecenter.points.push_back(point_pt);
									linecenter.points.push_back(point_pt);
									if (linecenter.points.size()*BOUNDARY_SAMPLE_DISTANCE > mForwardDistance) {
										break;
									}
								}
								sample_point_count++;
							}

							sample_point_count = 0;
							
							for (auto  &points : mLaneinfo.leftBoundary.segmentList)
							{
								for (int i = nearstIndex; i < points.size(); i++) {
									auto segmentPonit = points[i];
									if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
										NDM_Point point_pt;
										point_pt.x = segmentPonit.x; point_pt.y = segmentPonit.y; point_pt.z = segmentPonit.z;
										lineleft.points.push_back(point_pt);
										if (lineleft.points.size()*BOUNDARY_SAMPLE_DISTANCE > mForwardDistance) {
											break;
										}
									}
									sample_point_count++;
								}
							}

							sample_point_count = 0;
							for (auto &points : mLaneinfo.rightBoundary.segmentList)
							{
								for (int i = nearstIndex; i < points.size(); i++) {
									auto segmentPonit = points[i];
									if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
										NDM_Point point_pt;
										point_pt.x = segmentPonit.x; point_pt.y = segmentPonit.y; point_pt.z = segmentPonit.z;
										lineright.points.push_back(point_pt);
										if (lineright.points.size()*BOUNDARY_SAMPLE_DISTANCE > mForwardDistance) {
											break;
										}
									}
									sample_point_count++;
								}
							}

						}else{ //ÍùnearstIndex-Ç°ÕÒ2000m£¬ÍùnearstIndex+ºóÕÒ200m
							for (int i = nearstIndex; i < cLaneinfo.centerLine.size(); i--)
							{
								if (i > 0) {
									auto &point = cLaneinfo.centerLine[i];
									if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
										NDM_Point point_pt;
										point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
										linecenter.points.push_back(point_pt);
										linecenter.points.push_back(point_pt);
										if (linecenter.points.size()*BOUNDARY_SAMPLE_DISTANCE > mForwardDistance) {
											break;
										}
									}
									sample_point_count++;
								}
							}

							sample_point_count = 0;

							for (auto &points : mLaneinfo.leftBoundary.segmentList)
							{
								for (int i = nearstIndex; i < points.size(); i--) {
									if (i > 0) {
										auto segmentPonit = points[i];
										if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
											NDM_Point point_pt;
											point_pt.x = segmentPonit.x; point_pt.y = segmentPonit.y; point_pt.z = segmentPonit.z;
											lineleft.points.push_back(point_pt);
											if (lineleft.points.size()*BOUNDARY_SAMPLE_DISTANCE > mForwardDistance) {
												break;
											}
										}
										sample_point_count++;
									}
								}
							}

							sample_point_count = 0;
							for (auto &points : mLaneinfo.rightBoundary.segmentList)
							{
								for (int i = nearstIndex; i < points.size(); i--) {
									if (i > 0) {
										auto segmentPonit = points[i];
										if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
											NDM_Point point_pt;
											point_pt.x = segmentPonit.x; point_pt.y = segmentPonit.y; point_pt.z = segmentPonit.z;
											lineright.points.push_back(point_pt);
											if (lineright.points.size()*BOUNDARY_SAMPLE_DISTANCE > mForwardDistance) {
												break;
											}
										}
										sample_point_count++;
									}
								}
							}
						}
					}
				}
				else {
					if (laneName == mLaneinfo.laneName)
					{
						lineleft.marking = NDM_Util::GetMarking_(mLaneinfo.leftBoundary.roadmarkList[0]);  //TODO:
						lineright.marking = NDM_Util::GetMarking_(mLaneinfo.rightBoundary.roadmarkList[0]);  //TODO:
						linecenter.marking = NDM_LineMarking::LineMarking_LaneVirtualMarking;

						lineleft.color = NDM_Util::GetColor_(mLaneinfo.leftBoundary.roadmarkList[0]);
						lineright.color = NDM_Util::GetColor_(mLaneinfo.rightBoundary.roadmarkList[0]);
						linecenter.color = NDM_LineColor::UNKNOWN_LINE_COLOR;

						sample_point_count = 0;
						for (auto & points : mLaneinfo.leftBoundary.segmentList)
						{
							for (auto & segmentPonit : points) {
								if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
									NDM_Point point_pt;
									point_pt.x = segmentPonit.x; point_pt.y = segmentPonit.y; point_pt.z = segmentPonit.z;
									lineleft.points.push_back(point_pt);
								}
								sample_point_count++;
							}
						}
						sample_point_count = 0;
						for (auto & points : mLaneinfo.rightBoundary.segmentList)
						{
							for (auto & segmentPonit : points) {
								if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
									NDM_Point point_pt;
									point_pt.x = segmentPonit.x; point_pt.y = segmentPonit.y; point_pt.z = segmentPonit.z;
									lineright.points.push_back(point_pt);
								}
								sample_point_count++;
							}
						}

#ifdef NDM_MAP_LOCAL
						HDMapStandalone::MLaneInfo cLaneinfo = HDMapStandalone::MHDMap::GetLaneSample(mLaneinfo.laneName);
#else
						HDMapStandalone::MLaneInfo cLaneinfo;
						SimOneAPI::GetLaneSample(mLaneinfo.laneName, cLaneinfo);
#endif
						sample_point_count = 0;
						for (auto &point : cLaneinfo.centerLine)
						{
							if (sample_point_count%BOUNDARY_SAMPLE_DISTANCE == 0) {
								NDM_Point point_pt;
								point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
								linecenter.points.push_back(point_pt);
							}
							sample_point_count++;
						}
						break;
					}
				}
			}

			left.lines_3d.push_back(std::move(lineleft));
			right.lines_3d.push_back(std::move(lineright));
			center.lines_3d.push_back(std::move(linecenter));

			lines.lines.push_back(std::move(left));
			lines.lines.push_back(std::move(right));
			lines.lines.push_back(std::move(center));
		}

		void GetLanesInSameSection(const SSD::SimString &idStr, const SSD::SimVector<HDMapStandalone::MLaneLineInfo> &laneLineInfo) {

#ifdef NDM_MAP_LOCAL
			auto& laneLink = HDMapStandalone::MHDMap::GetLaneLink(idStr);
#else
			HDMapStandalone::MLaneLink laneLink;
			SimOneAPI::GetLaneLink(idStr, laneLink);
#endif
			//predecessorLaneNameList
			//std::cout << "---------------idStr-----------------"<<idStr.GetString() <<" mCurrentLaneName:" << mCurrentLaneName.GetString() <<  std::endl;
			if (idStr == mCurrentLaneName) {
				for (SSD::SimString predName : laneLink.predecessorLaneNameList)
				{
					bool isInJunction = false;
					long junctionId = -1;
#ifdef NDM_MAP_LOCAL
					isInJunction = HDMapStandalone::MHDMap::IsInJunction(predName, junctionId);
#else
					isInJunction = SimOneAPI::IsInJunction(predName, junctionId);
#endif

					if (isInJunction) {
						//»ñÈ¡ËùÓÐºÍÊ®×ÖÂ·¿Ú¹ØÁªµÄµÀÂ·
						GetJunctionConectRoad(junctionId);
						//»ñÈ¡ºÍµ±Ç°Ïà¹ØµÄÊ®×ÖÂ·¿ÚµÄÁ¬½ÓµÀÂ·
//#ifdef NDM_MAP_LOCAL
//						auto& preLaneLink = HDMapStandalone::MHDMap::GetLaneLink(predName);
//#else
//						HDMapStandalone::MLaneLink preLaneLink;
//						SimOneAPI::GetLaneLink(predName, preLaneLink);
//#endif
//						for(SSD::SimString pre_predName : preLaneLink.predecessorLaneNameList)
//						{
//							Create_Lines(mPhysicalLayer.lines, pre_predName, laneLineInfo);
//							Create_Objects(mPhysicalLayer.objects, pre_predName);
//							GetLanesInSameSection(pre_predName, laneLineInfo);
//						}
					}
					else {
						Create_Lines(mPhysicalLayer.lines, predName, laneLineInfo);
						Create_Objects(mPhysicalLayer.objects, predName);
						GetLanesInSameSection(predName, laneLineInfo);
					}
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
				if (splitItem[1] == splitItem_s[1]) {
					Create_Lines(mPhysicalLayer.lines, laneName_s, laneLineInfo);
					Create_Objects(mPhysicalLayer.objects, laneName_s);
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

		void GetParkingSpaces_() {

#ifdef NDM_MAP_LOCAL
			auto& parkingSpaces = HDMapStandalone::MHDMap::GetParkingSpaceList();
#else
			SSD::SimVector<HDMapStandalone::MParkingSpace> parkingSpaces;
#endif

			for (auto roadkv : mRoadIds) {
				for (auto parkingSpace : parkingSpaces) {
					if (parkingSpace.roadId== roadkv.first) {
						mParkingSpaces.push_back(parkingSpace);
						NDM_Object object;
						object.str_id.SetString(("parking_"+std::to_string(parkingSpace.id)).c_str());
						object.type = ObjectType_ParkingSlot;

						for (auto& point : parkingSpace.boundaryKnots)
						{
							NDM_Point point_pt;
							point_pt.x = point.x; point_pt.y = point.y; point_pt.z = point.z;
							object.border.points.push_back(point_pt);
						}
						auto iter = std::find_if(mPhysicalLayer.objects.objects.begin(), mPhysicalLayer.objects.objects.end(), [&](const Object & item)
						{
							return item.str_id == object.str_id;
						});
						if (iter == mPhysicalLayer.objects.objects.end())
						{
							mPhysicalLayer.objects.objects.push_back(std::move(object));
						}
					}
				}	
			}
		}
		void CreatePhysicalLayer(const SSD::SimPoint3D& pos, const double forward) {
			std::cout << "=============================start to create ndm map======================" << std::endl;
			mForwardDistance = forward;
			mCurrentPos = pos;
#ifdef NDM_MAP_LOCAL
			auto& laneLineInfo = HDMapStandalone::MHDMap::GetLaneLineInfo();
#else
			SSD::SimVector<HDMapStandalone::MLaneLineInfo> laneLineInfo;
			SimOneAPI::GetLaneLineInfo(laneLineInfo);
#endif
			LaneInfo_t laneinfo = NDM_Util::GetForwardLaneInfo(pos, forward);
			const auto& idStr = laneinfo.currentLane;
			mCurrentLaneName = laneinfo.currentLane;

			if (idStr.Empty())
			{
				return;
			}
#ifdef NDM_MAP_LOCAL
			mCurrentLaneLength = HDMapStandalone::MHDMap::GetLaneLength(mCurrentLaneName);
#else
			mCurrentLaneLength = SimOneAPI::GetLaneLength(mCurrentLaneName);
#endif
			for (auto lane_data : laneinfo.dataList) {
				for (int i = 0; i < (int)lane_data.laneNameList.size(); i++) {

					SSD::SimString & laneName = lane_data.laneNameList[i];
					//std::cout << "lanename:" << laneName.GetString()<<std::endl;
					bool isInJunction = false;
					long junctionId = -1;
#ifdef NDM_MAP_LOCAL
					isInJunction = HDMapStandalone::MHDMap::IsInJunction(laneName, junctionId);
#else
					isInJunction = SimOneAPI::IsInJunction(laneName, junctionId);
#endif
					if (isInJunction) {
						GetJunctionConectRoad(junctionId);
					}
				}
			}

			for (auto lane_data : laneinfo.dataList) {
				for (int i = 0; i < (int)lane_data.laneNameList.size(); i++)
				{
					SSD::SimString & laneName = lane_data.laneNameList[i];
					bool isInJunction = false;
					long junctionId = -1;
#ifdef NDM_MAP_LOCAL
					isInJunction = HDMapStandalone::MHDMap::IsInJunction(laneName, junctionId);
#else
					isInJunction = SimOneAPI::IsInJunction(laneName, junctionId);
#endif
					// delete lane in junction, because it is virtuallane
					if (isInJunction) {
						continue;
					}
					GetLanesInSameSection(laneName, laneLineInfo);
				}
			}

			for (auto kv : mRoadIds) {
				if (kv.second == false) {
					continue;
				}
#ifdef NDM_MAP_LOCAL

				const auto& sectionLanes = HDMapStandalone::MHDMap::GetLaneList(kv.first);
#else
				SSD::SimStringVector sectionLanes;
				SimOneAPI::GetLaneList(kv.first, sectionLanes);
#endif
				for (auto laneName : sectionLanes) {
					//std::cout << "current NameLaneNmae: " << std::string(laneName.GetString()) << std::endl;
					GetLanesInSameSection(laneName, laneLineInfo);
					break;
				}
			}
			GetParkingSpaces_();
			mLaneLineInfo = std::move(laneLineInfo);
			mLaneInfo = laneinfo;
		}

	public:
		NDM_PhysicalLayer mPhysicalLayer;
		SSD::SimString mCurrentLaneName;
		std::map<long, bool> mRoadIds;
		std::map<SSD::SimString,bool> mLaneIds;
		SSD::SimVector<HDMapStandalone::MParkingSpace> mParkingSpaces;
		SSD::SimVector<HDMapStandalone::MLaneLineInfo> mLaneLineInfo;
		LaneInfo_t mLaneInfo;
		SSD::SimPoint3D mCurrentPos;
		double mForwardDistance;
		double mCurrentLaneLength;
	};
}