#include "Python.h"
#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>

#include "../../Source/HDMap/SimOneHDMapAPI.h"
#include "public/MHDMap.h"

using namespace boost::python;

struct NearMostLaneInfo
{
	bool exists = false;
	SSD::SimString laneId;
	double s;
	double t;
	double s_toCenterLine;
	double t_toCenterLine;
};

struct NearLanesInfo
{
	bool exists = false;
	SSD::SimStringVector laneIdList;
};

struct DistanceToLaneBoundaryInfo
{
	bool exists = false;
	SSD::SimString laneId;
	double distToLeft;
	double distToRight;
	double distToLeft2D;
	double distToRight2D;
};

struct LaneSampleInfo
{
	bool exists = false;
	HDMapStandalone::MLaneInfo laneInfo;
};

struct LaneLinkInfo
{
	bool exists = false;
	HDMapStandalone::MLaneLink laneLink;
};

struct LaneTypeInfo
{
	bool exists = false;
	HDMapStandalone::MLaneType laneType;
};

struct LaneWidthInfo
{
	bool exists = false;
	double width;
};

struct STInfo
{
	bool exists = false;
	double s;
	double t;
};

struct STZInfo
{
	bool exists = false;
	double s;
	double t;
	double z;
};

struct PredefinedRouteInfo
{
	bool exists = false;
	SSD::SimPoint3DVector route;
};

struct GenerateRouteInfo
{
	bool exists = false;
	SSD::SimVector<int> indexOfValidPoints;
	SSD::SimPoint3DVector route;
};

struct NavigateInfo
{
	bool exists = false;
	SSD::SimVector<int> indexOfValidPoints;
	SSD::SimVector<long> roadIdList;
};

struct RoadMarkInfo
{
	bool exists = false;
	HDMapStandalone::MRoadMark left;
	HDMapStandalone::MRoadMark right;
};

struct InertialFromLaneSTInfo
{
	bool exists = false;
	SSD::SimPoint3D inertial;
	SSD::SimPoint3D dir;
};

struct LaneMiddlePointInfo
{
	bool exists = false;
	SSD::SimPoint3D targetPoint;
	SSD::SimPoint3D dir;
};

struct HeightsInfo
{
	bool exists = false;
	SSD::SimVector<double> heights;
	SSD::SimVector<long> roadIdList;
	SSD::SimVector<bool> insideRoadStates;
};

bool loadXodr(const SSD::SimString& file) {
	HDMapStandalone::MLoadErrorCode code;
	return HDMapStandalone::MHDMap::LoadData(file.GetString(), code);
};

bool loadHDMap(const int& timeout)
{
	return SimOneAPI::LoadHDMap(timeout);
}

NearMostLaneInfo getNearMostLane(const SSD::SimPoint3D& inputPt)
{
	NearMostLaneInfo info;
	info.exists = SimOneAPI::GetNearMostLane(inputPt, info.laneId, info.s, info.t,
		info.s_toCenterLine, info.t_toCenterLine);
	return std::move(info);
}

NearLanesInfo getNearLanes(const SSD::SimPoint3D& inputPt, const double& distance)
{
	NearLanesInfo info;
	info.exists = SimOneAPI::GetNearLanes(inputPt, distance, info.laneIdList);
	return std::move(info);
}

NearLanesInfo getNearLanesWithAngle(const SSD::SimPoint3D& inputPt,
	const double& distance,
	const double& headingAngle,
	const double& angleShift)
{
	NearLanesInfo info;
	info.exists = SimOneAPI::GetNearLanesWithAngle(inputPt, distance, headingAngle, angleShift, info.laneIdList);
	return std::move(info);
}

DistanceToLaneBoundaryInfo getDistanceToLaneBoundary(const SSD::SimPoint3D& inputPt)
{
	DistanceToLaneBoundaryInfo info;
	info.exists = SimOneAPI::GetDistanceToLaneBoundary(inputPt, info.laneId, info.distToLeft, info.distToRight,
		info.distToLeft2D, info.distToRight2D);
	return std::move(info);
}

LaneSampleInfo getLaneSample(const SSD::SimString& laneId)
{
	LaneSampleInfo info;
	info.exists = SimOneAPI::GetLaneSample(laneId, info.laneInfo);
	return std::move(info);
}

LaneLinkInfo getLaneLink(const SSD::SimString& laneId)
{
	LaneLinkInfo info;
	info.exists = SimOneAPI::GetLaneLink(laneId, info.laneLink);
	return std::move(info);
}

LaneTypeInfo getLaneType(const SSD::SimString& laneId)
{
	LaneTypeInfo info;
	info.exists = SimOneAPI::GetLaneType(laneId, info.laneType);
	return std::move(info);
}

LaneWidthInfo getLaneWidth(const SSD::SimString& laneId, const SSD::SimPoint3D& inputPt)
{
	LaneWidthInfo info;
	info.exists = SimOneAPI::GetLaneWidth(laneId, inputPt, info.width);
	return std::move(info);
}

STInfo getLaneST(const SSD::SimString& laneId, const SSD::SimPoint3D& inputPt)
{
	STInfo info;
	info.exists = SimOneAPI::GetLaneST(laneId, inputPt, info.s, info.t);
	return std::move(info);
}

STZInfo getRoadST(const SSD::SimString& laneId, const SSD::SimPoint3D& inputPt)
{
	STZInfo info;
	info.exists = SimOneAPI::GetRoadST(laneId, inputPt, info.s, info.t, info.z);
	return std::move(info);
}

bool containsLane(const SSD::SimString& laneId)
{
	return SimOneAPI::ContainsLane(laneId);
}

SSD::SimVector<HDMapStandalone::MParkingSpace> getParkingSpaceList()
{
	SSD::SimVector<HDMapStandalone::MParkingSpace> list;
	SimOneAPI::GetParkingSpaceList(list);
	return std::move(list);
}

GenerateRouteInfo generateRoute(const SSD::SimPoint3DVector& inputPoints)
{
	GenerateRouteInfo info;
	info.exists = SimOneAPI::GenerateRoute(inputPoints, info.indexOfValidPoints, info.route);
	return std::move(info);
}

NavigateInfo navigate(const SSD::SimPoint3DVector& inputPoints)
{
	NavigateInfo info;
	info.exists = SimOneAPI::Navigate(inputPoints, info.indexOfValidPoints, info.roadIdList);
	return std::move(info);
}

RoadMarkInfo getRoadMark(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneId)
{
	RoadMarkInfo info;
	info.exists = SimOneAPI::GetRoadMark(inputPt, laneId, info.left, info.right);
	return std::move(info);
}

SSD::SimVector<HDMapStandalone::MSignal> getTrafficLightList()
{
	SSD::SimVector<HDMapStandalone::MSignal> list;
	SimOneAPI::GetTrafficLightList(list);
	return std::move(list);
}

SSD::SimVector<HDMapStandalone::MSignal> getTrafficSignList()
{
	SSD::SimVector<HDMapStandalone::MSignal> list;
	SimOneAPI::GetTrafficSignList(list);
	return std::move(list);
}

SSD::SimVector<HDMapStandalone::MObject> getStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& laneId)
{
	SSD::SimVector<HDMapStandalone::MObject> list;
	SimOneAPI::GetStoplineList(light, laneId, list);
	return std::move(list);
}

SSD::SimVector<HDMapStandalone::MObject> getCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& laneId)
{
	SSD::SimVector<HDMapStandalone::MObject> list;
	SimOneAPI::GetCrosswalkList(light, laneId, list);
	return std::move(list);
}

SSD::SimVector<HDMapStandalone::MObject> getCrossHatchList(const SSD::SimString& laneId)
{
	SSD::SimVector<HDMapStandalone::MObject> list;
	SimOneAPI::GetCrossHatchList(laneId, list);
	return std::move(list);
}

InertialFromLaneSTInfo getInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t)
{
	InertialFromLaneSTInfo info;
	info.exists = SimOneAPI::GetInertialFromLaneST(id, s, t, info.inertial, info.dir);
	return std::move(info);
}

LaneMiddlePointInfo getLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id)
{
	LaneMiddlePointInfo info;
	info.exists = SimOneAPI::GetLaneMiddlePoint(inputPt, id, info.targetPoint, info.dir);
	return std::move(info);
}

HeightsInfo getHeights(const SSD::SimPoint3D& inputPt, const double& radius)
{
	HeightsInfo info;
	info.exists = SimOneAPI::GetHeights(inputPt, radius, info.heights, info.roadIdList, info.insideRoadStates);
	return std::move(info);
}

BOOST_PYTHON_MODULE(pySimOneIO)
{
	//from SimOneAPI
	//
	def("loadXodr", loadXodr);  //usage: test api offline

	def("loadHDMap", loadHDMap);  //usage: test api online with SimOne

	class_<NearMostLaneInfo>("pyNearMostLane")
		.def_readwrite("exists", &NearMostLaneInfo::exists)
		.def_readwrite("laneId", &NearMostLaneInfo::laneId)
		.def_readwrite("s", &NearMostLaneInfo::s_toCenterLine)
		.def_readwrite("t", &NearMostLaneInfo::t_toCenterLine)
		;
	def("getNearMostLane", getNearMostLane);

	class_<NearLanesInfo>("pyNearLanes")
		.def_readwrite("exists", &NearLanesInfo::exists)
		.def_readwrite("laneIdList", &NearLanesInfo::laneIdList)
		;
	def("getNearLanes", getNearLanes);
	def("getNearLanesWithAngle", getNearLanesWithAngle);

	class_<DistanceToLaneBoundaryInfo>("pyDistanceToLaneBoundary")
		.def_readwrite("exists", &DistanceToLaneBoundaryInfo::exists)
		.def_readwrite("laneId", &DistanceToLaneBoundaryInfo::laneId)
		.def_readwrite("distToLeft", &DistanceToLaneBoundaryInfo::distToLeft)
		.def_readwrite("distToRight", &DistanceToLaneBoundaryInfo::distToRight)
		.def_readwrite("distToLeft2D", &DistanceToLaneBoundaryInfo::distToLeft2D)
		.def_readwrite("distToRight2D", &DistanceToLaneBoundaryInfo::distToRight2D)
		;
	def("getDistanceToLaneBoundary", getDistanceToLaneBoundary);

	class_<LaneSampleInfo>("pyLaneSample")
		.def_readwrite("exists", &LaneSampleInfo::exists)
		.def_readwrite("laneInfo", &LaneSampleInfo::laneInfo)
		;
	def("getLaneSample", getLaneSample);

	class_<LaneLinkInfo>("pyLaneLink")
		.def_readwrite("exists", &LaneLinkInfo::exists)
		.def_readwrite("laneLink", &LaneLinkInfo::laneLink)
		;
	def("getLaneLink", getLaneLink);

	class_<LaneTypeInfo>("pyLaneType")
		.def_readwrite("exists", &LaneTypeInfo::exists)
		.def_readwrite("laneType", &LaneTypeInfo::laneType)
		;
	def("getLaneType", getLaneType);

	class_<LaneWidthInfo>("pyLaneWidth")
		.def_readwrite("exists", &LaneWidthInfo::exists)
		.def_readwrite("width", &LaneWidthInfo::width)
		;
	def("getLaneWidth", getLaneWidth);

	class_<STInfo>("pyST")
		.def_readwrite("exists", &STInfo::exists)
		.def_readwrite("s", &STInfo::s)
		.def_readwrite("t", &STInfo::t)
		;
	def("getLaneST", getLaneST);

	class_<STZInfo>("pySTZ")
		.def_readwrite("exists", &STZInfo::exists)
		.def_readwrite("s", &STZInfo::s)
		.def_readwrite("t", &STZInfo::t)
		.def_readwrite("z", &STZInfo::z)
		;
	def("getRoadST", getRoadST);

	def("containsLane", containsLane);

	def("getParkingSpaceList", getParkingSpaceList);

	class_<GenerateRouteInfo>("pyGenerateRoute")
		.def_readwrite("exists", &GenerateRouteInfo::exists)
		.def_readwrite("indexOfValidPoints", &GenerateRouteInfo::indexOfValidPoints)
		.def_readwrite("route", &GenerateRouteInfo::route)
		;
	def("generateRoute", generateRoute);

	class_<NavigateInfo>("pyNavigate")
		.def_readwrite("exists", &NavigateInfo::exists)
		.def_readwrite("indexOfValidPoints", &NavigateInfo::indexOfValidPoints)
		.def_readwrite("roadIdList", &NavigateInfo::roadIdList)
		;
	def("navigate", navigate);

	class_<RoadMarkInfo>("pyRoadMarkInfo")
		.def_readwrite("exists", &RoadMarkInfo::exists)
		.def_readwrite("left", &RoadMarkInfo::left)
		.def_readwrite("right", &RoadMarkInfo::right)
		;
	def("getRoadMark", getRoadMark);

	def("getTrafficLightList", getTrafficLightList);
	def("getTrafficSignList", getTrafficSignList);
	def("getStoplineList", getStoplineList);
	def("getCrosswalkList", getCrosswalkList);
	def("getCrossHatchList", getCrossHatchList);

	class_<InertialFromLaneSTInfo>("pyInertialFromLaneSTInfo")
		.def_readwrite("exists", &InertialFromLaneSTInfo::exists)
		.def_readwrite("inertial", &InertialFromLaneSTInfo::inertial)
		.def_readwrite("dir", &InertialFromLaneSTInfo::dir)
		;
	def("getInertialFromLaneST", getInertialFromLaneST);

	class_<LaneMiddlePointInfo>("pyLaneMiddlePointInfo")
		.def_readwrite("exists", &LaneMiddlePointInfo::exists)
		.def_readwrite("targetPoint", &LaneMiddlePointInfo::targetPoint)
		.def_readwrite("dir", &LaneMiddlePointInfo::dir)
		;
	def("getLaneMiddlePoint", getLaneMiddlePoint);

	class_<HeightsInfo>("pyHeightsInfo")
		.def_readwrite("exists", &HeightsInfo::exists)
		.def_readwrite("heights", &HeightsInfo::heights)
		.def_readwrite("roadIdList", &HeightsInfo::roadIdList)
		.def_readwrite("insideRoadStates", &HeightsInfo::insideRoadStates)
		;
	def("getHeights", getHeights);

	//from SSD
	//
	class_<SSD::SimPoint3D>("pySimPoint3D", init<double, double, double>())
		.def_readwrite("x", &SSD::SimPoint3D::x)
		.def_readwrite("y", &SSD::SimPoint3D::y)
		.def_readwrite("z", &SSD::SimPoint3D::z)
		;

	class_<SSD::SimPoint3DVector>("pySimPoint3DVector")
		.def(init<const SSD::SimPoint3DVector&>())
		//.def("GetElement", &SSD::SimPoint3DVector::GetElement, return_value_policy<copy_const_reference>())
		.def("GetElement", static_cast<const SSD::SimPoint3D& (SSD::SimPoint3DVector::*)(unsigned long long i) const>(&SSD::SimPoint3DVector::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimPoint3DVector::*)(const SSD::SimPoint3D&)>(&SSD::SimPoint3DVector::push_back))
		.def("Size", &SSD::SimPoint3DVector::size)
		;

	class_<SSD::SimString>("pySimString", init<const char*>())
		.def(init<const SSD::SimString&>())
		.def("GetString", &SSD::SimString::GetString)
		.def("SetString", &SSD::SimString::SetString)
		;

	class_<SSD::SimStringVector>("pySimStringVector")
		.def(init<const SSD::SimStringVector&>())
		//.def("GetElement", &SSD::SimStringVector::GetElement, return_value_policy<copy_const_reference>())
		.def("GetElement", static_cast<const SSD::SimString& (SSD::SimStringVector::*)(unsigned long long i) const>(&SSD::SimStringVector::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimStringVector::*)(const SSD::SimString&)>(&SSD::SimStringVector::push_back))
		.def("Size", &SSD::SimStringVector::size)
		;

	class_<SSD::SimVector<int>>("pyIntSimVector")
		.def(init<const SSD::SimVector<int>&>())
		//.def("GetElement", &SSD::SimPoint3DVector::GetElement, return_value_policy<copy_const_reference>())
		.def("GetElement", static_cast<const int& (SSD::SimVector<int>::*)(unsigned long long i) const>(&SSD::SimVector<int>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<int>::*)(const int&)>(&SSD::SimVector<int>::push_back))
		.def("Size", &SSD::SimVector<int>::size)
		;

	class_<SSD::SimVector<long>>("pyLongSimVector")
		.def(init<const SSD::SimVector<long>&>())
		.def("GetElement", static_cast<const long& (SSD::SimVector<long>::*)(unsigned long long i) const>(&SSD::SimVector<long>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<long>::*)(const long&)>(&SSD::SimVector<long>::push_back))
		.def("Size", &SSD::SimVector<long>::size)
		;

	class_<SSD::SimVector<double>>("pyDoubleSimVector")
		.def(init<const SSD::SimVector<double>&>())
		.def("GetElement", static_cast<const double& (SSD::SimVector<double>::*)(unsigned long long i) const>(&SSD::SimVector<double>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<double>::*)(const double&)>(&SSD::SimVector<double>::push_back))
		.def("Size", &SSD::SimVector<double>::size)
		;

	class_<SSD::SimVector<bool>>("pyBoolSimVector")
		.def(init<const SSD::SimVector<bool>&>())
		.def("GetElement", static_cast<const bool& (SSD::SimVector<bool>::*)(unsigned long long i) const>(&SSD::SimVector<bool>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<bool>::*)(const bool&)>(&SSD::SimVector<bool>::push_back))
		.def("Size", &SSD::SimVector<bool>::size)
		;

	class_<SSD::SimVector<HDMapStandalone::MSignal>>("pySignalSimVector")
		.def(init<const SSD::SimVector<HDMapStandalone::MSignal>&>())
		//.def("GetElement", &SSD::SimPoint3DVector::GetElement, return_value_policy<copy_const_reference>())
		.def("GetElement", static_cast<const HDMapStandalone::MSignal& (SSD::SimVector<HDMapStandalone::MSignal>::*)(unsigned long long i) const>(&SSD::SimVector<HDMapStandalone::MSignal>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<HDMapStandalone::MSignal>::*)(const HDMapStandalone::MSignal&)>(&SSD::SimVector<HDMapStandalone::MSignal>::push_back))
		.def("Size", &SSD::SimVector<HDMapStandalone::MSignal>::size)
		;

	class_<SSD::SimVector<HDMapStandalone::MObject>>("pyObjectSimVector")
		.def(init<const SSD::SimVector<HDMapStandalone::MObject>&>())
		//.def("GetElement", &SSD::SimPoint3DVector::GetElement, return_value_policy<copy_const_reference>())
		.def("GetElement", static_cast<const HDMapStandalone::MObject& (SSD::SimVector<HDMapStandalone::MObject>::*)(unsigned long long i) const>(&SSD::SimVector<HDMapStandalone::MObject>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<HDMapStandalone::MObject>::*)(const HDMapStandalone::MObject&)>(&SSD::SimVector<HDMapStandalone::MObject>::push_back))
		.def("Size", &SSD::SimVector<HDMapStandalone::MObject>::size)
		;

	class_<SSD::SimVector<HDMapStandalone::MSignalValidity>>("pySignalValiditySimVector")
		.def(init<const SSD::SimVector<HDMapStandalone::MSignalValidity>&>())
		.def("GetElement", static_cast<const HDMapStandalone::MSignalValidity& (SSD::SimVector<HDMapStandalone::MSignalValidity>::*)(unsigned long long i) const>(&SSD::SimVector<HDMapStandalone::MSignalValidity>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<HDMapStandalone::MSignalValidity>::*)(const HDMapStandalone::MSignalValidity&)>(&SSD::SimVector<HDMapStandalone::MSignalValidity>::push_back))
		.def("Size", &SSD::SimVector<HDMapStandalone::MSignalValidity>::size)
		;

	class_<SSD::SimVector<HDMapStandalone::MParkingSpace>>("pyParkingSpaceSimVector")
		.def(init<const SSD::SimVector<HDMapStandalone::MParkingSpace>&>())
		.def("GetElement", static_cast<const HDMapStandalone::MParkingSpace& (SSD::SimVector<HDMapStandalone::MParkingSpace>::*)(unsigned long long i) const>(&SSD::SimVector<HDMapStandalone::MParkingSpace>::operator[]), return_value_policy<copy_const_reference>())
		.def("AddElement",
			static_cast<void (SSD::SimVector<HDMapStandalone::MParkingSpace>::*)(const HDMapStandalone::MParkingSpace&)>(&SSD::SimVector<HDMapStandalone::MParkingSpace>::push_back))
		.def("Size", &SSD::SimVector<HDMapStandalone::MParkingSpace>::size)
		;

	//from HDMapModule
	//
	enum_<HDMapStandalone::MLoadErrorCode>("pyELoadErrorCode")
		.value("eNotLocalENU", HDMapStandalone::MLoadErrorCode::eNotLocalENU)
		.value("eEmptyRoad", HDMapStandalone::MLoadErrorCode::eEmptyRoad)
		.value("eInvalidData", HDMapStandalone::MLoadErrorCode::eInvalidData)
		;

	enum_<HDMapStandalone::MLaneType>("pyELaneType")
		.value("none", HDMapStandalone::MLaneType::none)
		.value("driving", HDMapStandalone::MLaneType::driving)
		.value("stop", HDMapStandalone::MLaneType::stop)
		.value("shoulder", HDMapStandalone::MLaneType::shoulder)
		.value("biking", HDMapStandalone::MLaneType::biking)
		.value("sidewalk", HDMapStandalone::MLaneType::sidewalk)
		.value("border", HDMapStandalone::MLaneType::border)
		.value("restricted", HDMapStandalone::MLaneType::restricted)
		.value("parking", HDMapStandalone::MLaneType::parking)
		.value("bidirectional", HDMapStandalone::MLaneType::bidirectional)
		.value("median", HDMapStandalone::MLaneType::median)
		.value("special1", HDMapStandalone::MLaneType::special1)
		.value("special2", HDMapStandalone::MLaneType::special2)
		.value("special3", HDMapStandalone::MLaneType::special3)
		.value("roadWorks", HDMapStandalone::MLaneType::roadWorks)
		.value("tram", HDMapStandalone::MLaneType::tram)
		.value("rail", HDMapStandalone::MLaneType::rail)
		.value("entry", HDMapStandalone::MLaneType::entry)
		.value("exit", HDMapStandalone::MLaneType::exit)
		.value("offRamp", HDMapStandalone::MLaneType::offRamp)
		.value("onRamp", HDMapStandalone::MLaneType::onRamp)
		.value("mwyEntry", HDMapStandalone::MLaneType::mwyEntry)
		.value("mwyExit", HDMapStandalone::MLaneType::mwyExit)
		;

	enum_<HDMapStandalone::ERoadMarkType>("pyERoadMarkType")
		.value("none", HDMapStandalone::ERoadMarkType::none)
		.value("solid", HDMapStandalone::ERoadMarkType::solid)
		.value("broken", HDMapStandalone::ERoadMarkType::broken)
		.value("solid_solid", HDMapStandalone::ERoadMarkType::solid_solid)
		.value("solid_broken", HDMapStandalone::ERoadMarkType::solid_broken)
		.value("broken_solid", HDMapStandalone::ERoadMarkType::broken_solid)
		.value("broken_broken", HDMapStandalone::ERoadMarkType::broken_broken)
		.value("botts_dots", HDMapStandalone::ERoadMarkType::botts_dots)
		.value("grass", HDMapStandalone::ERoadMarkType::grass)
		.value("curb", HDMapStandalone::ERoadMarkType::curb)
		;

	enum_<HDMapStandalone::ERoadMarkColor>("pyERoadMarkColor")
		.value("standard", HDMapStandalone::ERoadMarkColor::standard)
		.value("blue", HDMapStandalone::ERoadMarkColor::blue)
		.value("green", HDMapStandalone::ERoadMarkColor::green)
		.value("red", HDMapStandalone::ERoadMarkColor::red)
		.value("white", HDMapStandalone::ERoadMarkColor::white)
		.value("yellow", HDMapStandalone::ERoadMarkColor::yellow)
		;

	class_<HDMapStandalone::MLaneInfo>("pyLaneInfo")
		.def_readwrite("laneId", &HDMapStandalone::MLaneInfo::laneName)
		.def_readwrite("leftBoundary", &HDMapStandalone::MLaneInfo::leftBoundary)
		.def_readwrite("rightBoundary", &HDMapStandalone::MLaneInfo::rightBoundary)
		.def_readwrite("centerLine", &HDMapStandalone::MLaneInfo::centerLine)
		;

	class_<HDMapStandalone::MLaneLink>("pyLaneLink")
		.def_readwrite("predecessorLaneIds", &HDMapStandalone::MLaneLink::predecessorLaneNameList)
		.def_readwrite("successorLaneIds", &HDMapStandalone::MLaneLink::successorLaneNameList)
		.def_readwrite("leftNeighborLaneId", &HDMapStandalone::MLaneLink::leftNeighborLaneName)
		.def_readwrite("rightNeighborLaneId", &HDMapStandalone::MLaneLink::rightNeighborLaneName)
		;

	class_<HDMapStandalone::MRoadMark>("pyRoadMark")
		.def_readwrite("sOffset", &HDMapStandalone::MRoadMark::sOffset)
		.def_readwrite("length", &HDMapStandalone::MRoadMark::length)
		.def_readwrite("type", &HDMapStandalone::MRoadMark::type)
		.def_readwrite("color", &HDMapStandalone::MRoadMark::color)
		.def_readwrite("width", &HDMapStandalone::MRoadMark::width)
		;

	class_<HDMapStandalone::MSignalValidity>("pySignalValidity")
		.def_readwrite("roadId", &HDMapStandalone::MSignalValidity::roadId)
		.def_readwrite("sectionIndex", &HDMapStandalone::MSignalValidity::sectionIndex)
		.def_readwrite("fromLaneId", &HDMapStandalone::MSignalValidity::fromLaneId)
		.def_readwrite("toLaneId", &HDMapStandalone::MSignalValidity::toLaneId)
		.def_readwrite("stopLineIds", &HDMapStandalone::MSignalValidity::stopLineIds)
		.def_readwrite("crosswalkIds", &HDMapStandalone::MSignalValidity::crosswalkIds)
		;

	class_<HDMapStandalone::MSignal>("pySignal")
		.def_readwrite("id", &HDMapStandalone::MSignal::id)
		.def_readwrite("type", &HDMapStandalone::MSignal::type)
		.def_readwrite("subType", &HDMapStandalone::MSignal::subType)
		.def_readwrite("pt", &HDMapStandalone::MSignal::pt)
		.def_readwrite("heading", &HDMapStandalone::MSignal::heading)
		.def_readwrite("value", &HDMapStandalone::MSignal::value)
		.def_readwrite("unit", &HDMapStandalone::MSignal::unit)
		.def_readwrite("isDynamic", &HDMapStandalone::MSignal::isDynamic)
		.def_readwrite("validities", &HDMapStandalone::MSignal::validities)
		;

	class_<HDMapStandalone::MObject>("pyObject")
		.def_readwrite("id", &HDMapStandalone::MObject::id)
		.def_readwrite("type", &HDMapStandalone::MObject::type)
		.def_readwrite("pt", &HDMapStandalone::MObject::pt)
		.def_readwrite("boundaryKnots", &HDMapStandalone::MObject::boundaryKnots)
		;

	class_<HDMapStandalone::MParkingSpace>("pyParkingSpace")
		.def_readwrite("id", &HDMapStandalone::MParkingSpace::id)
		.def_readwrite("pt", &HDMapStandalone::MParkingSpace::pt)
		.def_readwrite("heading", &HDMapStandalone::MParkingSpace::heading)
		.def_readwrite("boundaryKnots", &HDMapStandalone::MParkingSpace::boundaryKnots)
		.def_readwrite("front", &HDMapStandalone::MParkingSpace::front)
		.def_readwrite("rear", &HDMapStandalone::MParkingSpace::rear)
		.def_readwrite("left", &HDMapStandalone::MParkingSpace::left)
		.def_readwrite("right", &HDMapStandalone::MParkingSpace::right)
		;

	class_<HDMapStandalone::MParkingSpaceMarking>("pyParkingSpaceMarking")
		.def_readwrite("side", &HDMapStandalone::MParkingSpaceMarking::side)
		.def_readwrite("type", &HDMapStandalone::MParkingSpaceMarking::type)
		.def_readwrite("color", &HDMapStandalone::MParkingSpaceMarking::color)
		.def_readwrite("width", &HDMapStandalone::MParkingSpaceMarking::width)
		;

	//Can expose MHDMap, MHDMapProto, etc.
	//
}