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

	SkeletonJoint(std::string name, std::vector<SkeletonJoint*> childJoints, btVector3 &localOffset)
	{
		m_name = name;
		m_childJoints = childJoints;
		m_localOffset = localOffset;
	}

	~SkeletonJoint() {};

	std::string GetName()		{ return m_name;		}
	btVector3 GetLocalOffset()	{ return m_localOffset; }
	
	std::vector<SkeletonJoint*> GetChildrenPointers() { return m_childJoints; }

	void PrintJoint();

private:
	std::string					m_name;
	btVector3					m_localOffset;
	std::vector<SkeletonJoint*> m_childJoints;
};

class SkeletalMotion
{
public:

	SkeletalMotion(
	std::string name,
	std::vector<std::vector<btVector3>> rootTrajectories,
	std::unordered_map<std::string, std::vector<btTransform>> jointTransforms,
	std::vector<SkeletonJoint*> skeletonRoots,
	float samplingRate,
	int	  frameCount) 
	{
		m_name = name;
		m_rootTrajectories	= rootTrajectories;
		m_jointTransforms	= jointTransforms;
		m_skeletonRoots		= skeletonRoots;
		m_samplingRate		= samplingRate;
		m_frameCount		= frameCount;
	};

	~SkeletalMotion();

	std::string GetName()	{ return m_name;			}
	float GetSamplingRate() { return m_samplingRate;	}
	int GetFrameCount()		{ return m_frameCount;		}
	SkeletonJoint* GetRoot(int index){ return m_skeletonRoots[index]; }

	void GetSkeletalSegments(std::vector<std::pair<btVector3, btVector3>> &result, int skeletonIndex, int frameIndex, bool addRootTrajectory);

	void GetJointPositions(std::vector<btVector3> &result, int skeletonIndex, int frameIndex, bool addRootTrajectory);

	void GetJointPositionsByName(std::unordered_map<std::string, btVector3> &result, int skeletonIndex, int frameIndex, bool addRootTrajectory);
private:
	std::string m_name;
	std::vector<std::vector<btVector3>> m_rootTrajectories;
	std::unordered_map<std::string,
		std::vector<btTransform>>		m_jointTransforms;

	std::vector<SkeletonJoint*>			m_skeletonRoots;
	float								m_samplingRate;
	int									m_frameCount;

public:

	static SkeletalMotion* BVHImport(std::string bvhFilePath);
};

class SkeletalAnimationPlayer
{
private:

	enum AnimationPlayerState
	{
		IS_PLAYING,
		IS_STOPED,
		IS_WAITING_START,
		IS_WAITING_END
	};

	AnimationPlayerState m_playerState;
	SkeletalMotion* m_skeletalMotion;
	float			m_timeElapsedInAnimation;
	bool			m_bIsStop;
};