#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include <string>
#include <vector>
#include <unordered_map>

class SkeletonJoint
{
public:

	SkeletonJoint() {};
	~SkeletonJoint() {};

	std::string m_name;
	std::vector<SkeletonJoint*> m_childJoints;
	int m_channelsOrdering[3]; // 0 = x, 1 = y, 2 = z
	btVector3 m_localOffset;

	void PrintJoint();
};

class SkeletalMotion
{
public:

	SkeletalMotion() {};
	~SkeletalMotion() {};

	btScalar m_frameTime; // frameTime in Seconds
	std::vector<std::vector<btVector3>> m_rootTrajectory;

	std::unordered_map<std::string, std::vector<btTransform>> m_jointTransforms;

	std::vector<SkeletonJoint*> m_skeletonRoots;
};

SkeletalMotion* BVHImport(std::string bvhFilePath);