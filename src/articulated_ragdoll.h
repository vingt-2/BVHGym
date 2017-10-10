#include "btBulletDynamicsCommon.h"
#include "animation.h"
#include <vector>
#include <string>

class KinematicMotionState : public btMotionState 
{
public:
	KinematicMotionState(const btTransform &initialpos) { mPos1 = initialpos; }
	virtual ~KinematicMotionState() { }
	virtual void getWorldTransform(btTransform &worldTrans) const 
	{ 
		worldTrans = mPos1; 
	}
	void setKinematicPos(btTransform &currentPos) { mPos1 = currentPos; }
	virtual void setWorldTransform(const btTransform &worldTrans) { }

protected:
	btTransform mPos1;
};

class ArticulatedRagdoll
{
public:
	virtual void UpdateJointPositions(int frameIndex) = 0;

	virtual void GetConstraintsJointPositions(std::vector<btVector3> &resultVector) = 0;

	virtual void KillRagdoll() = 0;

protected:
	SkeletalMotion*						m_skeletalMotion;
	btDynamicsWorld*					m_ownerWorld;
};

class RagdollWithKinematicBodiesConstraints : public ArticulatedRagdoll
{
public:
	RagdollWithKinematicBodiesConstraints(btDynamicsWorld* ownerWorld,
						SkeletalMotion* skeletalMotion,
						int skeletonIndex,
						const btVector3& positionOffset, 
						btScalar scale);

	~RagdollWithKinematicBodiesConstraints();

	virtual void UpdateJointPositions(int frameIndex);
	 
	virtual void GetConstraintsJointPositions(std::vector<btVector3> &resultVector);

	virtual void KillRagdoll();
	
private:
	std::vector<btCollisionShape*>		m_shapes;
	std::vector<btRigidBody*>			m_bodies;

	std::vector<btPoint2PointConstraint*>	m_jointConstraints;

	std::unordered_map<std::string,
		KinematicMotionState*>				m_jointKinematicMotionStates;

	std::unordered_map<std::string,
		btRigidBody*>					m_jointKinematicBody;

	btCompoundShape						m_compoundShape;

	btRigidBody* createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
};

//class MultiBodyArticulatedRagdoll