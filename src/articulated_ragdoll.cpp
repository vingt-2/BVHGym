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

btRigidBody* RagdollWithKinematicBodiesConstraints::createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
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

RagdollWithKinematicBodiesConstraints::RagdollWithKinematicBodiesConstraints(
	btDynamicsWorld* ownerWorld,
	SkeletalMotion* skeletalMotion,
	int skeletonIndex,
	const btVector3& positionOffset,
	btScalar scale)
{
	m_ownerWorld = ownerWorld;
	m_skeletalMotion = skeletalMotion;
	unordered_map<string, btVector3> jointPositionsByNames;
	unordered_map<string, btTransform> jointTransformsByNames;
	m_skeletalMotion->QuerySkeletalAnimation(0, 0, true, NULL, &jointPositionsByNames, NULL, &jointTransformsByNames);

	vector<pair<string, string>> bonesByJointNames;
	unordered_map<string, SkeletonJoint*> jointsByNames;
	m_skeletalMotion->GetRoot(0)->QuerySkeleton(&jointsByNames, &bonesByJointNames);

	for (auto jointEntry : jointPositionsByNames)
	{
		string jointName = jointEntry.first;
		btVector3 jointPosition = jointPositionsByNames[jointName];

		KinematicMotionState* kinematicMotionState = new KinematicMotionState(btTransform(btMatrix3x3::getIdentity(), jointPosition));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(0, kinematicMotionState, new btSphereShape(0.001), btVector3(0, 0, 0));
		btRigidBody* kinematicJointBody = new btRigidBody(rbInfo);
		kinematicJointBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_jointKinematicMotionStates[jointName] = kinematicMotionState;
		m_jointKinematicBody[jointName] = kinematicJointBody;
		m_ownerWorld->addRigidBody(kinematicJointBody);
	}

	// Capsule dude
	//for (auto bone : bonesByJointNames)
	//{
	//	string parentJointName = bone.first;
	//	string childJointName = bone.second;

	//	btVector3 parentPosition = jointPositionsByNames[parentJointName];
	//	btVector3 childPosition = jointPositionsByNames[childJointName];


	//	// Comput look at rotation to align the capsule 
	//	btQuaternion localOrn = btQuaternion::getIdentity();

	//	btVector3 diff = childPosition - parentPosition;
	//	btScalar lenSqr = diff.length2();
	//	btScalar height = 0.f;

	//	if (lenSqr > 1.192092896e-07F)
	//	{
	//		height = btSqrt(lenSqr);
	//		btVector3 ax = diff / height;

	//		btVector3 zAxis(0, 0, 1);
	//		localOrn = shortestArcQuat(zAxis, ax);
	//	}
	//	btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(0.08*height, 0.6*height);

	//	btTransform localTransform(localOrn, 0.5*(childPosition + parentPosition));
	//	
	//	m_shapes.push_back(capsuleShape);

	//	btRigidBody* rigidBody = createRigidBody(btScalar(10.0), localTransform, capsuleShape);

	//	rigidBody->setDamping(btScalar(0.3), btScalar(0.85));
	//	rigidBody->setDeactivationTime(btScalar(2));
	//	rigidBody->setSleepingThresholds(btScalar(0), btScalar(0));
	//	//rigidBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	//	m_bodies.push_back(rigidBody);

	//	btPoint2PointConstraint* childToJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[parentJointName], *rigidBody, btVector3(0, 0, 0), localTransform.inverse()*parentPosition);
	//	btPoint2PointConstraint* childToChildJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[childJointName], *rigidBody, btVector3(0, 0, 0), localTransform.inverse()*childPosition);

	//	m_jointConstraints.push_back(childToJointContraint);
	//	m_jointConstraints.push_back(childToChildJointContraint);
	//	m_ownerWorld->addConstraint(childToJointContraint);
	//	m_ownerWorld->addConstraint(childToChildJointContraint);
	//}

	// Christmas tree person !
	for (auto bone : bonesByJointNames)
	{
		string parentJointName = bone.first;
		string childJointName = bone.second;

		btVector3 parentPosition = jointPositionsByNames[parentJointName];
		btVector3 childPosition = jointPositionsByNames[childJointName];


		float segmentLength = parentPosition.distance(childPosition);

		//btCapsuleShape* shape = new btCapsuleShape(0.1, segmentLength);

		btBoxShape* shape = new btBoxShape(segmentLength*btVector3(0.1, 0.1, 0.1));

		m_shapes.push_back(shape);

		btTransform transform = jointTransformsByNames[parentJointName];
		transform.setOrigin(0.5 * (parentPosition + childPosition));

		btRigidBody* rigidBody = createRigidBody(btScalar(10.0), transform, shape);

		rigidBody->setDamping(btScalar(0.3), btScalar(0.85));
		rigidBody->setDeactivationTime(btScalar(2));
		rigidBody->setSleepingThresholds(btScalar(0), btScalar(0));
		//rigidBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_bodies.push_back(rigidBody);

		btPoint2PointConstraint* childToJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[parentJointName], *rigidBody, btVector3(0, 0, 0), transform.inverse()*parentPosition);

		m_jointConstraints.push_back(childToJointContraint);
		m_ownerWorld->addConstraint(childToJointContraint);

	}
}

void RagdollWithKinematicBodiesConstraints::GetConstraintsJointPositions(vector<btVector3> &resultVector)
{
	for (auto constraint : m_jointConstraints)
	{
		btRigidBody rgbdA = constraint->getRigidBodyA();
		btRigidBody rgbdB = constraint->getRigidBodyB();

		resultVector.push_back(rgbdA.getWorldTransform() * constraint->getPivotInA());
		resultVector.push_back(rgbdB.getWorldTransform() * constraint->getPivotInB());
	}
}

RagdollWithKinematicBodiesConstraints::~RagdollWithKinematicBodiesConstraints()
{
	int i;

	// Remove all constraints
	for (auto constraint : m_jointConstraints)
	{
		m_ownerWorld->removeConstraint(constraint);
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

void RagdollWithKinematicBodiesConstraints::UpdateJointPositions(int frameIndex)
{
	unordered_map<string, btVector3> jointPositions;
	m_skeletalMotion->QuerySkeletalAnimation(frameIndex, 0, true, NULL, &jointPositions, NULL, NULL);

	for (auto joint : jointPositions)
	{
		if (m_jointKinematicBody[joint.first])
			m_jointKinematicBody[joint.first]->setActivationState(DISABLE_DEACTIVATION);

		if (m_jointKinematicMotionStates[joint.first])
			m_jointKinematicMotionStates[joint.first]->setKinematicPos(btTransform(btMatrix3x3::getIdentity(), joint.second));
	}
}

void RagdollWithKinematicBodiesConstraints::KillRagdoll()
{
	for (auto kinematicBody : m_jointKinematicBody)
	{
		kinematicBody.second->setCollisionFlags(0);
		kinematicBody.second->setMotionState(new btDefaultMotionState(kinematicBody.second->getWorldTransform()));
		kinematicBody.second->setMassProps(1.0, btVector3(1, 1, 1));
	}
}