
#pragma once

#include "libexport.h"
#ifdef STATIC_SSD
#include "SSD/public/SimPoint3D.h"
#include "SSD/public/SimString.h"
#else
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#endif
#include "public/common/MEnum.h"
#include "public/common/MRoadMark.h"

namespace SSD
{
	struct SimPoint2D;
}

namespace HDMapStandalone {

class LIBEXPORT MLocation {
public:

	//@brief Check whether input point is inside specified lane.
	//@param inputPt is a 3d point.
	//laneName with this format roadId_sectionIndex_laneId.
	//sideState is as an output parameter, which returns side state.
	//@pre-condition laneName must exist in current map.
	//@return Returns true if input point is inside, else returns false.
	static bool IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, MSideState& sideState);

	//@brief Check whether input point is inside specified road.
	//@param inputPt is a 3d point.
	//roadId is road id.
	//@pre-condition specified road must exist in current map.
	//@return Returns true if input point is inside, else returns false.
	static bool IsInsideRoad(const SSD::SimPoint3D& inputPt, const long& roadId);

	//@brief Get the lane which is near most to the input point.
	//@param inputPt is a 3d point.
	//laneName is as an output parameter, which returns the laneId(with this format roadId_sectionIndex_laneId) of the found lane.
	//[s, t] is the input point's value pair in s-t coordinate system, relative to the found lane.
	//[s_toCenterLine, t_toCenterLine] is the input point's value pair in s-t coordinate system, 
	//relative to the found lane's owner road's center line. [s_toCenterLine, t_toCenterLine]'s values are fuzzy accurate,
	//please use GetST or GetSTFast for highly accurate values.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearMostLane(const SSD::SimPoint3D& inputPt, SSD::SimString& laneName, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine);

	//@brief Get the lane which is near most to or geometry overlapping the input point.
	//When there are more than one lane's geometry overlaps the input point, will pick the distance near most one.
	//@param inputPt is a 3d point.
	//laneName is as an output parameter, which returns the laneId(with this format roadId_sectionIndex_laneId) of the found lane.
	//[s, t] is the input point's value pair in s-t coordinate system, relative to the found lane.
	//[s_toCenterLine, t_toCenterLine] is the input point's value pair in s-t coordinate system, 
	//relative to the found lane's owner road's center line. [s_toCenterLine, t_toCenterLine]'s values are fuzzy accurate,
	//please use GetST or GetSTFast for highly accurate values for [s_toCenterLine, t_toCenterLine].
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearMostLane_V2(const SSD::SimPoint3D& inputPt, SSD::SimString& laneName, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine);

	//@brief Respecting height of input point, get the lane which is near most to or geometry overlapping the input point.
	//When there are more than one lane's geometry overlaps the input point, will pick the distance near most one.
	//If input point's height is 2 meters gap with lane's height, such lane will be ignored.
	//@param inputPt is a 3d point.
	//drivingOnly is a flag whether to find dirving only lanes or not.
	//laneName is as an output parameter, which returns the laneId(with this format roadId_sectionIndex_laneId) of the found lane.
	//[s, t] is the input point's value pair in s-t coordinate system, relative to the found lane.
	//[s_toCenterLine, t_toCenterLine] is the input point's value pair in s-t coordinate system, 
	//relative to the found lane's owner road's center line. [s_toCenterLine, t_toCenterLine]'s values are fuzzy accurate,
	//please use GetST or GetSTFast for highly accurate values for [s_toCenterLine, t_toCenterLine].
	//insideLane is as an output parameter, which returns whether input point is inside the near most lane or not.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearMostLaneWithHeight_V2(const SSD::SimPoint3D& inputPt, bool drivingOnly, SSD::SimString& laneName, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine, bool& insideLane);

	//@brief Respecting height of input point, get the lane which is near most to or geometry overlapping the input point.
	//When there are more than one lane's geometry overlaps the input point, will pick the distance near most one.
	//If input point's height is 2 meters gap with lane's height, such lane will be ignored.
	//@param inputPt is a 3d point.
	//laneTypeList is white list of lane types.
	//laneName is as an output parameter, which returns the laneId(with this format roadId_sectionIndex_laneId) of the found lane.
	//[s, t] is the input point's value pair in s-t coordinate system, relative to the found lane.
	//[s_toCenterLine, t_toCenterLine] is the input point's value pair in s-t coordinate system, 
	//relative to the found lane's owner road's center line. [s_toCenterLine, t_toCenterLine]'s values are fuzzy accurate,
	//please use GetST or GetSTFast for highly accurate values for [s_toCenterLine, t_toCenterLine].
	//insideLane is as an output parameter, which returns whether input point is inside the near most lane or not.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearMostLaneWithHeight_V2(const SSD::SimPoint3D& inputPt, const SSD::SimVector<MLaneType>& laneTypeList, SSD::SimString& laneName, double& s, double& t,
		double& s_toCenterLine, double& t_toCenterLine, bool& insideLane);

	//@brief Get the lane which is near to the input point in a specified range.
	//@param inputPt is a 3d point. distance is the range distance.
	//laneList is as an output parameter, which returns the laneId list(with this format roadId_sectionIndex_laneId) of the found lanes.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearLanes(const SSD::SimPoint3D& inputPt, const double& distance, SSD::SimStringVector& laneList);

	//@brief Get the lane which is near to the input point in a specified range and also heading to within a specified angle range in 2d-inertial system.
	//@param inputPt is a 3d point. distance is the range distance.
	//headingAngle is a specified heading direction's angle relative to x-axis in 2d-inertial system. headingAngle is defined as radian.
	//angleShift is to help define the range of angles as [headingAngle - angleShift, headingAngle + angleShift], and angleShift is defined as radian.
	//laneList is as an output parameter, which returns the laneId list(with this format roadId_sectionIndex_laneId) of the found lanes.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearLanes(const SSD::SimPoint3D& inputPt, const double& distance, const double& headingAngle, const double& angleShift, SSD::SimStringVector& laneList);

	//@brief Get the lane which is near to the input point in a specified range.
	//@param inputPt is a 3d point. distance is the range distance.
	//laneList is as an output parameter, which returns the laneId list(with this format roadId_sectionIndex_laneId) of the found lanes.
	//s_toCenterLineList is as an output parameter, which returns the s value list relative to center line.
	//t_toCenterLineList is as an output parameter, which returns the t value list relative to center line.
	//[s_toCenterLine, t_toCenterLine]'s values are fuzzy accurate, please use GetST or GetSTFast for highly accurate values.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearLanes(const SSD::SimPoint3D& inputPt, const double& distance, SSD::SimStringVector& laneList,
		SSD::SimVector<double>& s_toCenterLineList, SSD::SimVector<double>& t_toCenterLineList);

	//@brief Get the lane which is near to the input point in a specified range and also heading to within a specified angle range in 2d-inertial system.
	//@param inputPt is a 3d point. distance is the range distance.
	//headingAngle is a specified heading direction's angle relative to x-axis in 2d-inertial system. headingAngle is defined as radian.
	//angleShift is to help define the range of angles as [headingAngle - angleShift, headingAngle + angleShift], and angleShift is defined as radian.
	//laneList is as an output parameter, which returns the laneId list(with this format roadId_sectionIndex_laneId) of the found lanes.
	//s_toCenterLineList is as an output parameter, which returns the s value list relative to center line.
	//t_toCenterLineList is as an output parameter, which returns the t value list relative to center line.
	//[s_toCenterLine, t_toCenterLine]'s values are fuzzy accurate, please use GetST or GetSTFast for highly accurate values.
	//@return Returns true if the lane is found, else returns false.
	static bool GetNearLanes(const SSD::SimPoint3D& inputPt, const double& distance, const double& headingAngle, const double& angleShift,
		SSD::SimStringVector& laneList, SSD::SimVector<double>& s_toCenterLineList, SSD::SimVector<double>& t_toCenterLineList);

	//@brief First, find the near most lane to the input point. Second, find the near most middle point on the target lane in 2d domain.
	//@param inputPt is a 2d point. targetPoint is an output parameter that returns the middle point on the lane.
	//dir is an output parameter that returns the direction at the middle point.
	//@return Returns true if the lane and the middle point is found, else returns false.
	static bool GetNearMostLaneMiddlePoint(const SSD::SimPoint2D& inputPt, SSD::SimPoint2D& targetPoint, SSD::SimPoint2D& dir);

	//@brief First, find the near most lane to the input point. Second, find the near most middle point on the target lane in 3d domain.
	//@param inputPt is a 3d point. targetPoint is an output parameter that returns the middle point on the lane.
	//dir is an output parameter that returns the direction at the middle point.
	//@return Returns true if the lane and the middle point is found, else returns false.
	static bool GetNearMostLaneMiddlePoint(const SSD::SimPoint3D& inputPt, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);

	//@brief First, find the near most lane to the input point. Second, find the near most middle point on the target lane in 2d domain.
	//if near most lane does not satisfies laneType, return false.
	//@param inputPt is a 2d point. laneType is the specified type.
	//targetPoint is an output parameter that returns the middle point on the lane.
	//dir is an output parameter that returns the direction at the middle point.
	//@return Returns true if the lane satifies laneType and the middle point is found, else returns false.
	static bool GetNearMostLaneMiddlePoint(const SSD::SimPoint2D& inputPt, MLaneType laneType, SSD::SimPoint2D& targetPoint, SSD::SimPoint2D& dir);

	//@brief First, find the near most lane to the input point. Second, find the near most middle point on the target lane in 3d domain.
	//if near most lane does not satisfies laneType, return false.
	//@param inputPt is a 3d point. laneType is the specified type.
	//targetPoint is an output parameter that returns the middle point on the lane.
	//dir is an output parameter that returns the direction at the middle point.
	//@return Returns true if the lane satifies laneType and the middle point is found, else returns false.
	static bool GetNearMostLaneMiddlePoint(const SSD::SimPoint3D& inputPt, MLaneType laneType, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);

	//@brief First, get height values of input point. The point may in vertical intersect with multiple roads with different heights.
	//@param inputPt is a 3d point. Its z value is useless, so just set as 0 is ok.
	//radius indicates how far away to detect in circle.
	//heights is an output parameter that returns heights/one height.
	//roadIds is an output parameter that returns the road ids that the target height is based one.
	//insideRoadStates is an output parameter that returns whether inputPt is inside the target roads or not.
	//@return Returns true if any height is found, else returns false.
	static bool GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights, SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates);

	//@brief Get normal of plane that is defined by input 3 points.
	//@param pt1, pt2, pt3 is respectively a 2d point as input point. 
	//laneName indicates the lane that these points are based on to calculate height, and normal eventually.
	//@return Returns normal vector in 3d.
	static SSD::SimPoint3D GetNormal(const SSD::SimPoint2D& pt1, const SSD::SimPoint2D& pt2, const SSD::SimPoint2D& pt3, const SSD::SimString& laneName);

	//@brief Get middle point of the specified lane that is projected into in 2d domain.
	//@param inputPt is a 2d point. laneName with this format roadId_sectionIndex_laneId.
	//targetPoint is an output parameter that returns the middle point on the lane.
	//dir is an output parameter that returns the direction at the middle point.
	//@return Returns true if the middle point is found, else returns false.
	static bool GetLaneMiddlePoint(const SSD::SimPoint2D& inputPt, const SSD::SimString& laneName, SSD::SimPoint2D& targetPoint, SSD::SimPoint2D& dir);

	//@brief Get middle point of the specified lane that is projected into in 3d domain.
	//@param inputPt is a 3d point. laneName with this format roadId_sectionIndex_laneId.
	//targetPoint is an output parameter that returns the middle point on the lane.
	//dir is an output parameter that returns the direction at the middle point.
	//@return Returns true if the middle point is found, else returns false.
	static bool GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);

	//@brief Get lane's width in bitangent direction of specified point.
	//@param inputPt is a 3d point. laneName with this format roadId_sectionIndex_laneId.
	//@pre-condition laneName must exist in current map.
	//@return Returns lane width.
	static double GetLaneWidth(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName);

	//@brief Get the distance info to the near most lane's left and right boundaries.
	//@param inputPt is a 3d point.
	//laneName is as an output parameter, which returns the laneId(with this format roadId_sectionIndex_laneId) of the found lane.
	//distToLeft is an output parameter that returns the 3D distance to the left boundary.
	//distToRight is an output parameter that returns the 3D distance to the right boundary.
	//distToLeftIn2D is an output parameter that returns the 2D distance(ignore z) to the left boundary.
	//distToRightIn2D is an output parameter that returns the 2D distance(ignore z) to the right boundary.
	//@return Returns true if near most lane is found, else returns false.
	static bool DistanceToLaneBoundary(const SSD::SimPoint3D& inputPt, SSD::SimString& laneName, double& distToLeft, double& distToRight,
		double& distToLeftIn2D, double& distToRightIn2D);

	//@brief Get the distance info to specified lane's left and right boundaries.
	//@param laneName is as an input parameter, which specifies the laneId(with this format roadId_sectionIndex_laneId) of the interested lane.
	//inputPt is a 3d point.
	//distToLeft is an output parameter that returns the 3D distance to the left boundary.
	//distToRight is an output parameter that returns the 3D distance to the right boundary.
	//distToLeftIn2D is an output parameter that returns the 2D distance(ignore z) to the left boundary.
	//distToRightIn2D is an output parameter that returns the 2D distance(ignore z) to the right boundary.
	//@return Returns true if specified lane exists and the point is in the s range(from 0 to length) of the lane,
	//else returns false.
	static bool DistanceToLaneBoundary(const SSD::SimString& laneName, const SSD::SimPoint3D& inputPt, double& distToLeft, double& distToRight,
		double& distToLeftIn2D, double& distToRightIn2D);

	//@brief Get the [s, t] value pair in s-t coordinate system relative to the lane's parent road's reference line 
	//and z value for height.
	//@param laneName is laneId this format roadId_sectionIndex_laneId. inputPt is a 3d point.
	//[s, t] is the output value pair in s-t coordinate system, relative to the lane's parent road's reference line.
	//z is the output value that represents the height value of inputPt in localENU.
	//@return Returns true if input laneName is a valid lane and is able to calculate [s, t], else returns false.
	static bool GetST(const SSD::SimString& laneName, const SSD::SimPoint3D& inputPt, double& s, double& t, double& z);

	//@brief Get the [s, t] value pair in s-t coordinate system relative to the lane's parent road's reference line 
	//and z value for height. This function computes faster than GetST.
	//@param laneName is laneId this format roadId_sectionIndex_laneId. inputPt is a 3d point.
	//referenceS is very accurate to s value(error is less than 0.05 meter), and GetSTFast will calculate a high precission result for s value.
	//[s, t] is the output value pair in s-t coordinate system, relative to the lane's parent road's reference line.
	//z is the output value that represents the height value of inputPt in localENU.
	//@return Returns true if input laneName is a valid lane and is able to calculate [s, t], else returns false.
	static bool GetSTFast(const SSD::SimString& laneName, const SSD::SimPoint3D& inputPt, double referenceS, double& s, double& t, double &z);

	//@brief Get the [s, t] value pair in s-t coordinate system relative to the lane's center line.
	//@param laneName is laneId this format roadId_sectionIndex_laneId. inputPt is a 3d point.
	//[s, t] is the output value pair in s-t coordinate system, relative to the lane's center line.
	//@return Returns true if input laneName is a valid lane and is able to calculate [s, t], else returns false.
	static bool GetLaneST(const SSD::SimString& laneName, const SSD::SimPoint3D& inputPt, double& s, double& t);

	//@brief Get inertial [x, y, z] point by specifying [s, t] value pair in s-t coordinate system relative to the lane's center line.
	//@param laneName is laneId this format roadId_sectionIndex_laneId. inputPt is a 3d point.
	//[s, t] is the input value pair in s-t coordinate system, relative to the lane's center line.
	//inertial is the output value that represents inertial point value.
	//dir is the output value that represents direction.
	//@return Returns true if input laneName is a valid lane and is able to calculate [s, t], else returns false.
	static bool GetInertialFromLaneST(const SSD::SimString& laneName, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir);

	//@brief Get inertial [x, y, z] point by specifying [s, t] value pair in s-t coordinate system relative to the road's reference line.
	//It is much faster at calculating speed than GetInertialFromRoadST_V0.
	//@param roadId is specified road id.
	//[s, t] is the input value pair in s-t coordinate system, relative to the road's reference line.
	//inertial is the output value that represents inertial point value.
	//dir is the output value that represents direction.
	//@return Returns true if able to calculate [s, t], else returns false.
	static bool GetInertialFromRoadST(const long& roadId, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir);

	//@brief Check whether position of vehicle tyre overlaps any lane line.
	//@param inputPt is a 3d point. radius is the range.
	//laneId is the output value that returns the lane line's owner lane's id.
	//@return Returns true if overlaps, else returns false.
	static bool IsOverlapLaneLine(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimString& laneId);

	//@brief Check whether the vehicle's body area including positions of 4 tyres overlaps specified lane line.
	//@param laneId is the id of specified lane.
	//overlapLeft and overlapRight, are output values that return whether overlaps on left or right boundary.
	//frontLeft, frontRight, rearLeft, rearRight, each is a 3d point of one tyre.
	//lnaeId is the output value that returns the lane line's owner lane's id.
	//@return Returns true if overlaps, else returns false.
	static bool IsOverlapLaneLine(const SSD::SimString& laneId, const SSD::SimPoint3D& frontLeft, const SSD::SimPoint3D& frontRight,
		const SSD::SimPoint3D& rearLeft, const SSD::SimPoint3D& rearRight, bool& overlapLeft, bool& overlapRight);

	//@brief Get lane's roadmark on the nearmost left and right boundaries.
	//@param inputPt is a 3d point.
	//laneName with this format roadId_sectionIndex_laneId.
	//left is as an output parameter, which returns the roadmark of left boundary.
	//right is as an output parameter, which returns the roadmark of right boundary.
	//@pre-condition laneName must exist in current map.
	//@return Returns MLaneType enum value.
	static bool GetRoadMark(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, MRoadMark& left, MRoadMark& right);

	//@brief Check whether specified point belongs to a junction in geometry.
	//@param inputPt is a 3d point.
	//laneTypeList is white list of lane types.
	//juncId is as an output parameter, which returns the owner junction id.
	//@return Returns true if the speified point belongs to a junction, else returns false.
	static bool IsInJunction(const SSD::SimPoint3D& inputPt, const SSD::SimVector<MLaneType>& laneTypeList, long& juncId);
};
}
