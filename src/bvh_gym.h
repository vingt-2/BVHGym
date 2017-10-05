///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include <stdio.h>
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

#include "articulated_ragdoll.h"
#include "animation_clip.h"

class CommonExampleInterface*    BVHGymCreateFunc(struct CommonExampleOptions& options);

struct BVHGym : public CommonRigidBodyBase
{
	BVHGym(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper) {};

	virtual ~BVHGym(){}
	virtual void initPhysics();
	virtual void renderScene();
	void ResetCamera();
};