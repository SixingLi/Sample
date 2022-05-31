#ifndef NDM_MAP_LOCAL
//#define NDM_MAP_LOCAL

#include "horizon_map_env_ndm_data.h"
#include "horizon_map_env_ndm_physical.h"
#include "horizon_map_env_ndm_logical.h"
#include "horizon_map_env_ndm_topological.h"

#include <string>
#include <iostream>
#include <fstream>

using namespace std;
/*******************************************************************************
@ this is ndm convert from simone hdmap api, the origin data format is opendrive
@ the parameter of each struct have its default value, some value is unavailable
@ while the long long type data value is -1 represernet the value is unavailable,
@ while the value of string type is empty string the value is unavailable
********************************************************************************/
namespace HorizonMapEnv {
	
	typedef struct MapEnvMsg {
		//NDM_NDMHeader header;
		NDM_PhysicalLayer physical_layer;
		NDM_LogicalLayer logical_layer;
		NDM_TopologicalLayer topological_layer;
		//NDM_DynamicLayer *dynamic_layer = nullptr;
		//NDM_EgoLayer *ego_layer = nullptr;  // 自车信息, 用于实时三维重建信息回传
		//NDM_MapEnvStatistics *statistics = nullptr;
		unsigned int status = 1;
		unsigned int diag_code = -1;

	}NDM_MapEnvMsg;

	class MapEnvMsg_Creator {
	public:
		MapEnvMsg_Creator(SSD::SimPoint3D input,double forward)
			:mInput(input),mForward (forward)
		{
			ofs.open("ndmenv_data.txt");
			
		}
		void PrintPhysicalLayer(const NDM_PhysicalLayer &physicalLayer) {
			ofs << "+++++++++++++++++++++++++++++++++++ physicalLayer.objects.objects+++++++++++++++++++++++++++++++++ size="<< physicalLayer.objects.objects.size()<< std::endl;
			for (auto physicalobject : physicalLayer.objects.objects) {

				ofs << "str_id:" << physicalobject.str_id.GetString() << std::endl
					<< "type:" << physicalobject.type << std::endl
					<< "subtype:" << physicalobject.sub_type << std::endl
					<< "border:" << std::endl;
				for (auto point : physicalobject.border.points) {
					ofs << "	point:" << point.x << "	" << point.y << "	" << point.z << std::endl;
				}
			}

			ofs << "++++++++++++++++++++++++++++++physicalLayer.lines.lines++++++++++++++++++++++++++++++++++++++ size=" << physicalLayer.lines.lines .size()<< std::endl;
			for (auto physicalline : physicalLayer.lines.lines) {
				ofs << "str_id:" << physicalline.str_id.GetString() << std::endl
					<< "type:" << physicalline.type << std::endl;
				ofs << "line_3d:" << std::endl;
				for (auto line3d_t : physicalline.lines_3d) {
					ofs << "	color:" << line3d_t.color << std::endl
						<<"	marking:"<< line3d_t.marking<<std::endl;
					ofs << "	point:" << std::endl;
					for (auto point : line3d_t.points) {
						ofs << "		point:" << point.x << "	" << point.y << "	" << point.z << std::endl;
					}
				}
			}
			ofs.flush();
		}

		void PrintLogicalLayer(const NDM_LogicalLayer &logicalLayer) {

			ofs << "++++++++++++++++++++++++++++++logicalLayer.virtuallines++++++++++++++++++++++++++++++++++++++ size="<< logicalLayer.virtuallines.size() << std::endl;
			for (auto vitline : logicalLayer.virtuallines) {
				ofs << "str_id:" << vitline.str_id.GetString() << std::endl
					<< "type:"<< vitline.type<<std::endl;
				ofs << "line_3d:" << std::endl;
				for (auto line3d_t : vitline.lines_3d) {
					ofs << "	color:" << line3d_t.color << std::endl
						<< "	marking:" << line3d_t.marking << std::endl;
					ofs << "	point:" << std::endl;
					for (auto point : line3d_t.points) {
						ofs << "		point:" << point.x << "	" << point.y << "	" << point.z << std::endl;
					}
				}
			}

			ofs << "++++++++++++++++++++++++++++++logicalLayer.lanes++++++++++++++++++++++++++++++++++++++ size="<< logicalLayer.lanes.size() << std::endl;
			for (auto lane : logicalLayer.lanes) {
				//typedef struct Lane {
				//	SSD::SimString str_id;
				//	SSD::SimStringVector l_laneline_ids;
				//	SSD::SimStringVector r_laneline_ids;
				//	SSD::SimString driveline_id;
				//	NDM_LaneDirection direction;
				//	NDM_LaneTransition transition;
				//	SSD::SimVector<NDM_LaneAttr>  attrs;
				//	float lane_length;
				//	int type;
				//	SSD::SimVector<NDM_Link> objs;
				//	//SSD::SimVector<NDM_Link> obstacles;
				//	SSD::SimStringVector pred_ids;
				//	SSD::SimStringVector succ_ids;
				//	SSD::SimStringVector left_ids;
				//	SSD::SimStringVector right_ids;
				//	SSD::SimVector<NDM_LaneRestriction> restrictions;
				//}NDM_Lane;
				ofs << "str_id:	" << lane.str_id.GetString() << std::endl
					<< "	driveline_id:" << lane.driveline_id.GetString() << std::endl
					<< "	direction:" << lane.direction << std::endl
					<< "	transition:" << lane.transition << std::endl
					<< "	lane_length:" << lane.lane_length << std::endl
					<< "	type:" << lane.type << std::endl;
				ofs << "	l_laneline_ids:" << std::endl;
				for (auto l_laneline_id : lane.l_laneline_ids) {
					ofs << "		l_laneline_id:" << l_laneline_id.GetString()<< std::endl;
				}

				ofs << "	r_laneline_ids:" << std::endl;
				for (auto r_laneline_id : lane.r_laneline_ids) {
					ofs << "		r_laneline_id:" << r_laneline_id.GetString() << std::endl;
				}

				ofs << "	pred_ids:" << std::endl;
				for (auto pred_id : lane.pred_ids) {
					ofs << "		pred_id:" << pred_id.GetString() << std::endl;
				}
				ofs << "	succ_ids:" << std::endl;
				for (auto succ_id : lane.succ_ids) {
					ofs << "		succ_id:" << succ_id.GetString() << std::endl;
				}

				ofs << "	left_ids:" << std::endl;
				for (auto left_id : lane.left_ids) {
					ofs << "		left_id:" << left_id.GetString() << std::endl;
				}

				ofs << "	right_ids:" << std::endl;
				for (auto right_id : lane.right_ids) {
					ofs << "		right_id:" << right_id.GetString() << std::endl;
				}

				ofs << "	attrs:" << std::endl;
				for (auto attr : lane.attrs) {
					ofs << "		curvature:" << attr.curvature << std::endl;
					ofs << "		slope:" << attr.slope << std::endl;
					ofs << "		banking:" << attr.banking << std::endl;
					ofs << "		headingAngle:" << attr.headingAngle << std::endl;
					ofs << "		offset:" << attr.offset << std::endl;
					ofs << "		width:" << attr.width << std::endl << std::endl;
				}
			}
			ofs.flush();

			for (auto parkingspace : logicalLayer.parkingspaces) {


			}

			for (auto section : logicalLayer.sections) {


			}

			for (auto junction : logicalLayer.junctions) {


			}
		}

		void CreateMapEnvMsg() {
			SSD::SimPoint3D inputPt = mInput;
			inputPt.z = 0;
			double forward = mForward;

			HorizonMapEnv::NDM_PhysicalLayer_Creator physicalLayerCreator;
			physicalLayerCreator.CreatePhysicalLayer(inputPt, forward);
			mNDM_Msg.physical_layer = physicalLayerCreator.mPhysicalLayer;
			PrintPhysicalLayer(mNDM_Msg.physical_layer);

			HorizonMapEnv::NDM_LogicalLayer_Creator logicalLayerCreator;
			logicalLayerCreator.Create_LogicalLayer(inputPt, forward, physicalLayerCreator);
			mNDM_Msg.logical_layer = logicalLayerCreator.mLogicalLayer;
			PrintLogicalLayer(mNDM_Msg.logical_layer);

			//HorizonMapEnv::NDM_TopologicalLayer_Creator topologicalLayerCreator;
			//topologicalLayerCreator.Create_TopologicalLayer(logicalLayerCreator.mLogicalLayer);
			//mNDM_Msg.topological_layer = topologicalLayerCreator.mTopologicalLayer;
			ofs.close();
		}

		~MapEnvMsg_Creator() {

		}
	
	public:
		SSD::SimPoint3D mInput;
		double mForward;
		NDM_MapEnvMsg mNDM_Msg;
		ofstream ofs;
		LaneInfo_t mLaneInfo;
	};
}

#endif //NDM_MAP_LOCAL
