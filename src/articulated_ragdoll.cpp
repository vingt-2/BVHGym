#include "articulated_ragdoll.h"


#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"

#include "LinearMath/btVector3.h"

#include <vector>
#include <stack>

using namespace std;

#ifndef M_PI
#define M_PI       btScalar(3.14159265358979323846)
#endif

#ifndef M_PI_2
#define M_PI_2     btScalar(1.57079632679489661923)
#endif

#ifndef M_PI_4
#define M_PI_4     btScalar(0.785398163397448309616)
#endif

btRigidBody* ArticulatedRagDoll::createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
}

ArticulatedRagDoll::ArticulatedRagDoll(
	btDynamicsWorld* ownerWorld,
	SkeletalMotion* skeletalMotion,
	int skeletonIndex,
	const btVector3& positionOffset,
	btScalar scale)
: m_ownerWorld(ownerWorld), m_skeletalMotion(skeletalMotion)
{
	unordered_map<string, btVector3> jointPositions;
	m_skeletalMotion->QuerySkeletalAnimation(0, 0, true, NULL, &jointPositions, NULL, NULL);

	/*for (auto joint : jointPositions)
	{
		string jointName = joint.first;
		btVector3 jointPosition = joint.second;

		KinematicMotionState* kinematicMotionState = new KinematicMotionState(btTransform(btMatrix3x3::getIdentity(), jointPosition));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(0, kinematicMotionState, new btSphereShape(0.001), btVector3(0, 0, 0));
		btRigidBody* kinematicJointBody = new btRigidBody(rbInfo);
		kinematicJointBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_jointKinematicMotionStates[jointName] = kinematicMotionState;
		m_ownerWorld->addRigidBody(kinematicJointBody);

		btBoxShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
		m_shapes.push_back(shape);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(jointPosition);

		btRigidBody* rigidBody = createRigidBody(btScalar(1.0), transform, shape);

		rigidBody->setDamping(btScalar(0.05), btScalar(0.85));
		rigidBody->setDeactivationTime(btScalar(0.8));
		rigidBody->setSleepingThresholds(btScalar(1.6), btScalar(2.5));
		m_bodies.push_back(createRigidBody(btScalar(1.0), transform, shape));
		
		btPoint2PointConstraint* jointToBoxContraint = new btPoint2PointConstraint(*kinematicJointBody, *rigidBody, btVector3(0, 0, 0), btVector3(0, 0, 0));
		m_ownerWorld->addConstraint(jointToBoxContraint);
		m_jointConstraints[jointName] = jointToBoxContraint;
	}*/

	stack<SkeletonJoint*> jointStack = stack<SkeletonJoint*>({ m_skeletalMotion->GetRoot(0) });
	while (jointStack.size())
	{
		SkeletonJoint* currentJoint = jointStack.top();
		jointStack.pop();

		btVector3 jointPosition = jointPositions[currentJoint->GetName()];
		btVector4 jointPositionH(jointPosition[0], jointPosition[1], jointPosition[2], 1.0);

		KinematicMotionState* kinematicMotionState = new KinematicMotionState(btTransform(btMatrix3x3::getIdentity(), jointPosition));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(0, kinematicMotionState, new btSphereShape(0.001), btVector3(0, 0, 0));
		btRigidBody* kinematicJointBody = new btRigidBody(rbInfo);
		kinematicJointBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_jointKinematicMotionStates[currentJoint->GetName()] = kinematicMotionState;
		m_ownerWorld->addRigidBody(kinematicJointBody);

		for (auto child : currentJoint->GetChildrenPointers())
		{
			btVector3 childPosition = jointPositions[child->GetName()];
			float segmentLength = jointPosition.distance(childPosition);
			btCapsuleShape* shape = new btCapsuleShape(0.1, segmentLength);

			m_shapes.push_back(shape);

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(0.5 * (jointPosition + childPosition));

			btRigidBody* rigidBody = createRigidBody(btScalar(1.0), transform, shape);

			rigidBody->setDamping(btScalar(0.05), btScalar(0.85));
			rigidBody->setDeactivationTime(btScalar(0.8));
			rigidBody->setSleepingThresholds(btScalar(1.6), btScalar(2.5));
			rigidBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
			m_bodies.push_back(createRigidBody(btScalar(1.0), transform, shape));

			btVector3 jointInRigidBodyLocal = transform.inverse() * jointPositionH;

			btPoint2PointConstraint* childToJointContraint = new btPoint2PointConstraint(*kinematicJointBody, *rigidBody, btVector3(0, 0, 0), jointInRigidBodyLocal);

			m_jointConstraints[currentJoint->GetName()] = childToJointContraint;
			m_ownerWorld->addConstraint(childToJointContraint);

			jointStack.push(child);
		}

		//btPoint2PointConstraint* parentToJointContraint = new btPoint2PointConstraint(*kinematicJointBody, *currentJoint, btVector3(0, 0, 0), jointInRigidBodyLocal);
	}

}

ArticulatedRagDoll::~ArticulatedRagDoll()
{
	int i;

	// Remove all constraints
	for (auto joint : m_jointConstraints)
	{
		m_ownerWorld->removeConstraint(joint.second);
	}

	// Remove all bodies and shapes
	for (i = 0; i < m_bodies.size(); ++i)
	{
		m_ownerWorld->removeRigidBody(m_bodies[i]);

		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	}
}

void ArticulatedRagDoll::UpdateJointPositions(int frameIndex)
{
	unordered_map<string, btVector3> jointPositions;
	m_skeletalMotion->QuerySkeletalAnimation(frameIndex, 0, true, NULL, &jointPositions, NULL, NULL);

	for (auto joint : jointPositions)
	{
		if (m_jointKinematicMotionStates[joint.first])
			m_jointKinematicMotionStates[joint.first]->setKinematicPos(btTransform(btMatrix3x3::getIdentity(), joint.second));
	}
}