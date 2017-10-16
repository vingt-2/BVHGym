#include "btBulletDynamicsCommon.h"
#include "BulletDynamics\Featherstone\btMultiBodyDynamicsWorld.h"
#include "BulletDynamics\Featherstone\btMultiBody.h"
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

	virtual btVector3 GetCOMPosition() = 0;
	virtual btVector3 GetCOMVelocity() = 0;
	virtual btVector3 GetCOMAngularMomentum() = 0;


	virtual float		GetTotalMass() = 0;

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

	virtual btVector3 GetCOMPosition() { return m_COMPosition; }
	virtual btVector3 GetCOMVelocity() { return btVector3(); };
	virtual btVector3 GetCOMAngularMomentum(){ return m_angularMomentum; };
	
	virtual float		GetTotalMass()
	{
		float mass = 0;
		for (auto m : m_masses)
		{
			mass += m;
		}
		return mass;
	}
	
private:
	std::vector<btCollisionShape*>			m_shapes;
	std::vector<btRigidBody*>				m_bodies;
	std::vector<float>						m_masses;
	std::vector<btPoint2PointConstraint*>	m_jointConstraints;

	std::unordered_map<std::string,
		KinematicMotionState*>				m_jointKinematicMotionStates;

	std::unordered_map<std::string,
		btRigidBody*>					m_jointKinematicBody;

	btCompoundShape						m_compoundShape;

	btRigidBody* createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

	btVector3 m_COMPosition;
	btVector3 m_COMVelocity;
	btVector3 m_COMAcceleration;

	btVector3 m_angularMomentum;
};

class MultiBodyArticulatedRagdoll : public ArticulatedRagdoll
{
public:
	MultiBodyArticulatedRagdoll(btMultiBodyDynamicsWorld* ownerWorld,
		SkeletalMotion* skeletalMotion,
		int skeletonIndex,
		const btVector3& positionOffset,
		btScalar scale);

	~MultiBodyArticulatedRagdoll();

	virtual void UpdateJointPositions(int frameIndex);

	virtual void GetConstraintsJointPositions(std::vector<btVector3> &resultVector);

	virtual void KillRagdoll();
private:

	btMultiBody* m_multiBody;

	unordered_map<string, int> m_linkIndicesByName;
};