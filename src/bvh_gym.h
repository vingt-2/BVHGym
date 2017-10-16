///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include <stdio.h>
#include <iostream>
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces\CommonMultiBodyBase.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

#include "articulated_ragdoll.h"

class CommonExampleInterface*    BVHGymCreateFunc(struct CommonExampleOptions& options);

struct BVHGym : public CommonMultiBodyBase
{
	BVHGym(struct GUIHelperInterface* helper) : CommonMultiBodyBase(helper)
	{
		m_bDrawSkeleton = false;
		m_bDrawCOMState = true;
	};

	virtual ~BVHGym(){}
	virtual void initPhysics();
	virtual void renderScene();
	void ResetCamera();

	void SetBVHAnimation(SkeletalMotion* motion)
	{
		m_animationPlayer->SetSkeletalMotion(motion);
		m_skeletalMotion = motion;
	}

	bool keyboardCallback(int keycode, int state)
	{
		if (keycode == 65295)
			m_keycode = 1;
		else if (keycode == 65296)
			m_keycode = 2;
		else if (keycode == 65309)
			m_keycode = 0;
		else
			m_keycode = -1;

		if (m_animationPlayer)
			m_animationPlayer->InputKeyPressed(m_keycode);

		return true;
	}

	int GetKeyPressed() { return m_keycode; }

	void ClearKeyPressed() { m_keycode = -1; }

private:
	SkeletalAnimationPlayer* m_animationPlayer;
	SkeletalMotion* m_skeletalMotion;
	ArticulatedRagdoll* m_articulatedRagdoll;
	std::vector<btVector3> m_comPositions;
	std::vector<btVector3> m_comVelocities;
	std::vector<btVector3> m_angularMomentum;

	bool m_bDrawSkeleton;
	bool m_bDrawCOMState;

	int m_keycode = -1;
private:
	btClock m_clock;
};