#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include <string>
#include <vector>
#include <unordered_map>

using namespace std;

class SkeletonJoint
{
public:

	SkeletonJoint(string name, vector<SkeletonJoint*> childJoints, btVector3 &localOffset)
	{
		m_name = name;
		m_childJoints = childJoints;
		m_localOffset = localOffset;
	}

	~SkeletonJoint() {};

	string GetName()		{ return m_name;		}
	btVector3 GetLocalOffset()	{ return m_localOffset; }
	
	vector<SkeletonJoint*> GetChildrenPointers() { return m_childJoints; }
	
	void QuerySkeleton(vector<string> jointsByName, vector<pair<string,string>> bonesByJointNames);

	void PrintJoint();

private:
	string					m_name;
	btVector3				m_localOffset;
	vector<SkeletonJoint*>	m_childJoints;
};

class SkeletalMotion
{
public:

	SkeletalMotion(
	string name,
	vector<vector<btVector3>> rootTrajectories,
	unordered_map<string, vector<btTransform>> jointTransforms,
	vector<SkeletonJoint*> skeletonRoots,
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

	string GetName()		{ return m_name;			}
	float GetSamplingRate() { return m_samplingRate;	}
	int GetFrameCount()		{ return m_frameCount;		}
	SkeletonJoint* GetRoot(int index){ return m_skeletonRoots[index]; }

	void QuerySkeletalAnimation
	(
		/*Defines the query inputs*/
		int frameIndex,
		int skeletonIndex,
		bool addRootOffset,
		/*Defines the query outputs*/
		vector<btVector3>* jointPositions = NULL,
		unordered_map<string, btVector3>* jointPositionsByName = NULL,
		vector<pair<btVector3, btVector3>>* segmentPositions = NULL,
		unordered_map<string, btTransform>* cumulativeTransformsByName = NULL
	);

private:
	string m_name;
	vector<vector<btVector3>> m_rootTrajectories;
	unordered_map<string,
		vector<btTransform>>		m_jointTransforms;

	vector<SkeletonJoint*>			m_skeletonRoots;
	float								m_samplingRate;
	int									m_frameCount;

public:

	static SkeletalMotion* BVHImport(string bvhFilePath);
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