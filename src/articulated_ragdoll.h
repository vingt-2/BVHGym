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

class ArticulatedRagDoll
{
public:




	ArticulatedRagDoll( btDynamicsWorld* ownerWorld,
						SkeletalMotion* skeletalMotion,
						int skeletonIndex,
						const btVector3& positionOffset, 
						btScalar scale);

	~ArticulatedRagDoll();

	void UpdateJointPositions(int frameIndex);

private:
	SkeletalMotion*						m_skeletalMotion;
	btDynamicsWorld*					m_ownerWorld;
	std::vector<btCollisionShape*>		m_shapes;
	std::vector<btRigidBody*>			m_bodies;

	std::unordered_map<std::string, 
		btTypedConstraint*>				m_jointConstraints;

	std::unordered_map<std::string,
		KinematicMotionState*>					m_jointKinematicMotionStates;

	btRigidBody* createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
};