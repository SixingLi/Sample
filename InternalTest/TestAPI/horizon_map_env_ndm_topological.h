#pragma once

#include "horizon_map_env_ndm_data.h"

#include <iostream>

namespace HorizonMapEnv {

	typedef struct RoadNode{
		SSD::SimString id;
		SSD::SimStringVector include_ids;    //Section, Junction
		// from_edge --> to_edge
		SSD::SimStringVector in_link_ids;    //RoadEdge
		SSD::SimStringVector out_link_ids;   //RoadEdge
		float probability = 0;     // 车辆到达此Path的可能性
		float turn_angle = 0;      // 转向角度
	}NDM_RoadNode;

	typedef struct  RoadEdge{
		SSD::SimString id;
		SSD::SimVector<NDM_Link> sections;
	}NDM_RoadEdge;

	typedef enum FormOfWay {
		FormOfWay_Unknown = 0,
		FormOfWay_MultipleCarriageWay = 2,
		FormOfWay_SingleCarriageWay = 3,
		FormOfWay_RoundaboutCircle = 4,
		FormOfWay_Service = 13,
		FormOfWay_RampEntry = 16,
		FormOfWay_RampExit = 17,
		FormOfWay_CrossLink = 18,
		FormOfWay_JCT = 19,
		FormOfWay_SlipRoad = 20,
		FormOfWay_SideRoad = 21,
		FormOfWay_SlipAndJCT = 22,
		FormOfWay_TurnRightLineA = 23,
		FormOfWay_TurnRightLineB = 24,
		FormOfWay_TurnLeftLineA = 25,
		FormOfWay_TurnLeftLineB = 26,
		FormOfWay_TurnLeftRightLine = 27,
		FormOfWay_ServiceAndSlipRoad = 28,
		FormOfWay_ServiceAndJCT = 29,
		FormOfWay_ServiceAndSlipRoadAndJCT = 30,
		FormOfWay_Other = 99
	}NDM_FormOfWay;

	typedef struct Road{
		SSD::SimString id;
		SSD::SimString name;
		SSD::SimVector<NDM_Link> forward_links;
		SSD::SimVector<NDM_Link> backward_links;
		NDM_FormOfWay form;		//form of way
		int function;
		SSD::SimVector<NDM_Link> links;  ///Area, Parking
	}NDM_Road;

	typedef struct TopologicalLayer {
		SSD::SimVector<NDM_Line> reflines;
		SSD::SimVector<NDM_Road> roads;
		SSD::SimVector<NDM_RoadNode> road_nodes;
		SSD::SimVector<NDM_RoadEdge> road_edges;
	}NDM_TopologicalLayer;

	class NDM_TopologicalLayer_Creator {

	public:
		NDM_TopologicalLayer_Creator() {
			
		}

		void Print(const NDM_RoadEdge& roadEdge)
		{
			std::cout << "RoadEdge id:" << roadEdge.id.GetString() << std::endl;
			std::cout << "sections count=" << roadEdge.sections.size();
			for (auto& section : roadEdge.sections)
			{
				std::cout << ", section.id:" << section.id.GetString();
				std::cout << ", section.offset:" << section.offset;
				std::cout << ", section.end_offset:" << section.end_offset;
			}
			std::cout << std::endl;
		}

		void Create_TopologicalLayer(NDM_LogicalLayer logicalLayer) {

			HDMapStandalone::MTopoGraph graph;
			static long roadNodeId = 0;
			std::map<long, std::vector<HDMapStandalone::MTopoNode>> junctionToTopoNode;

#ifdef NDM_MAP_LOCAL
			HDMapStandalone::MRouting::GetTopoGraph(graph);
#else
			//SimOneAPI::GetTopoGraph(graph);
#endif

			//RoadEdge
			//
			std::cout << "==================" << std::endl
				<< "Test RoadEdge" << std::endl
				<< "==================" << std::endl;
			std::vector<long> roadIdList;
//			for (auto& topoNode : graph.topoNodes)
//			{
//				if (topoNode.laneId.laneId == 0)
//				{
//					continue;
//				}
//
//				long juncId;
//#ifdef NDM_MAP_LOCAL
//				if (HDMapStandalone::MHDMap::IsInJunction(topoNode.laneId.ToString(), juncId))
//#else
//				if (SimOneAPI::IsInJunction(topoNode.laneId.ToString(), juncId))
//#endif
//				{
//					junctionToTopoNode[juncId].push_back(topoNode);
//				}
//
//				NDM_RoadEdge edge;
//				edge.id = topoNode.laneId.ToString();
//				NDM_Link link;
//				//sectionIndex as id
//				link.id.SetString((UtilString::ToString<long>(topoNode.laneId.roadId) + "_" + UtilString::ToString<int>(topoNode.laneId.sectionIndex)).c_str());
//				link.offset = topoNode.startS;
//				link.end_offset = topoNode.endS;
//				edge.sections.push_back(link);
//				Print(edge);
//				mTopologicalLayer.road_edges.push_back(edge);
//
//				auto iter = std::find(roadIdList.begin(), roadIdList.end(), topoNode.laneId.roadId);
//				if (iter == roadIdList.end())
//				{
//					roadIdList.push_back(topoNode.laneId.roadId);
//				}
//			}
		}
	public:
		NDM_TopologicalLayer mTopologicalLayer;
	};
}	