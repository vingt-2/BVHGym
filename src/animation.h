#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include <string>
#include <vector>
#include <iostream>
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

	string		GetName()			{ return m_name; }
	btVector3	GetLocalOffset()	{ return m_localOffset; }
	void		ApplyOffsetNormalization(float normalizer)	{ m_localOffset /= normalizer; }
	vector<SkeletonJoint*> GetDirectChildren()	{ return m_childJoints; }

	void QuerySkeleton(unordered_map<string, SkeletonJoint*>* jointPointersByNames, vector<pair<string, string>>* bonesByJointNames);

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
		m_rootTrajectories = rootTrajectories;
		m_jointTransforms = jointTransforms;
		m_skeletonRoots = skeletonRoots;
		m_samplingRate = samplingRate;
		m_frameCount = frameCount;
		m_skeletonScale = 1.0f;
	};

	~SkeletalMotion();

	string GetName()		{ return m_name; }
	float GetSamplingRate() { return m_samplingRate; }
	int GetFrameCount()		{ return m_frameCount; }
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

	void SetNormalizedScale();

	void SetScale(float scale) { m_skeletonScale = scale; }

private:
	string m_name;
	vector<vector<btVector3>>		m_rootTrajectories;
	unordered_map < string,
		vector < btTransform >>		m_jointTransforms;

	vector<SkeletonJoint*>			m_skeletonRoots;
	float							m_samplingRate;
	int								m_frameCount;
	float							m_skeletonScale;
public:

	static SkeletalMotion* BVHImport(string bvhFilePath);
};

class SkeletalAnimationPlayer
{
public:
	int GetCurrentAnimationFrame()
	{
		if (m_currentAnimationFrame < 0)
			m_currentAnimationFrame = 0;

		if (m_currentAnimationFrame > m_skeletalMotion->GetFrameCount() - 1)
			m_currentAnimationFrame = m_skeletalMotion->GetFrameCount() - 1;


		return (int)m_currentAnimationFrame;
	}

	void InputKeyPressed(int keyPressed)
	{
		if (m_clock.getTimeSeconds() - m_lastInputTime < 0.2)
		{
			return;
		}

		if (m_playerState == IS_WAITING_START)
		{
			if (keyPressed == 0)
			{
				m_currentAnimationFrame = 0;
				m_motionSpeed = 1;
				m_playerState = IS_PLAYING;
			}
			else if (keyPressed == 1)
			{
				std::cout << "Frame --\n";
				m_currentAnimationFrame -= 10;
			}
			else if (keyPressed == 2)
			{
				std::cout << "Frame ++\n";
				m_currentAnimationFrame += 10;
			}
		}
		else if (m_playerState == IS_PLAYING)
		{
			if (keyPressed == 0)
			{
				std::cout << "Stoping Player\n";
				m_playerState = IS_STOPED;
			}
			else if (keyPressed == 1)
			{
				std::cout << "Speed / 2\n";
				m_motionSpeed /= 2.0;
			}
			else if (keyPressed == 2)
			{
				std::cout << "Speed * 2\n";
				m_motionSpeed *= 2.0;
			}
		}
		else if (m_playerState == IS_STOPED)
		{
			if (keyPressed == 0)
			{
				std::cout << "Starting Player\n";
				m_playerState = IS_PLAYING;
			}
			else if (keyPressed == 1)
			{
				std::cout << "Frame --\n";
				m_currentAnimationFrame--;
			}
			else if (keyPressed == 2)
			{
				std::cout << "Frame ++\n";
				m_currentAnimationFrame++;
			}
		}
		m_lastInputTime = m_clock.getTimeSeconds();
	}

	void UpdatePlayer();

	void SetSkeletalMotion(SkeletalMotion* motion)
	{
		m_currentAnimationFrame = 0;
		m_skeletalMotion = motion;
	}

	SkeletalAnimationPlayer()
	{
		m_playerState = IS_WAITING_START;
		m_motionSpeed = 1;
		m_lastTime = m_clock.getTimeSeconds();
		m_lastInputTime = m_clock.getTimeSeconds();
		m_currentAnimationFrame = 0;
		bIsCycleAnimation = true;
	}

	bool bIsCycleAnimation;

private:

	enum AnimationPlayerState
	{
		IS_PLAYING,
		IS_STOPED,
		IS_WAITING_START,
		IS_WAITING_END
	};

	btClock m_clock;
	float	m_lastInputTime;
	float	m_currentAnimationFrame;
	float	m_lastTime;
	AnimationPlayerState m_playerState;
	SkeletalMotion*		 m_skeletalMotion;
	float				 m_timeElapsedInAnimation;

	float	m_motionSpeed;
};