#ifndef NDM_MAP_LOCAL
//#define NDM_MAP_LOCAL

#include "horizon_map_env_ndm_data.h"
#include "horizon_map_env_ndm_physical.h"
#include "horizon_map_env_ndm_logical.h"
#include "horizon_map_env_ndm_topological.h"

#include <string>
#include <iostream>

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
			
			
		}

		void CreateMapEnvMsg() {
			SSD::SimPoint3D inputPt = mInput;
			inputPt.z = 0;
			double forward = mForward;

			HorizonMapEnv::NDM_PhysicalLayer_Creator physicalLayerCreator;
			HorizonMapEnv::NDM_LogicalLayer_Creator logicalLayerCreator;
			physicalLayerCreator.CreatePhysicalLayer(inputPt, forward);
			mNDM_Msg.physical_layer = physicalLayerCreator.mPhysicalLayer;
			logicalLayerCreator.Create_LogicalLayer(inputPt, forward);
			mNDM_Msg.logical_layer = logicalLayerCreator.mLogicalLayer;
		}

		~MapEnvMsg_Creator() {

		}
	
	public:
		SSD::SimPoint3D mInput;
		double mForward;
		NDM_MapEnvMsg mNDM_Msg;
	};
}

#endif //NDM_MAP_LOCAL
