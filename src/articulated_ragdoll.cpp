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

btMatrix3x3 RotationMatrix(int axis, btScalar angle)
{
	angle *= M_PI / 180.0;

	if (axis == 0)
	{
		return btMatrix3x3
			(
			1, 0, 0,
			0, cos(angle), -sin(angle),
			0, sin(angle), cos(angle)
			);
	}
	else if (axis == 1)
	{
		return btMatrix3x3
			(
			cos(angle), 0, sin(angle),
			0, 1, 0,
			-sin(angle), 0, cos(angle)
			);
	}
	else if (axis == 2)
	{
		return btMatrix3x3
			(
			cos(angle), -sin(angle), 0,
			sin(angle), cos(angle), 0,
			-0, 0, 1
			);
	}
}

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

btMatrix3x3 upLookAt(btVector3& direction)
{
	// compute the forward vector
	btVector3 up = direction;
	up.normalize();

	// compute temporal up vector based on the forward vector
	// watch out when look up/down at 90 degree
	// for example, forward vector is on the Y axis
	btVector3 left = btVector3(0, 1, 0);;

	// compute the left vector
	btVector3 forward = up.cross(left);  // cross product
	forward.normalize();

	// re-calculate the orthonormal up vector
	left = up.cross(forward);  // cross product
	left.normalize();

	return btMatrix3x3(left[0], left[1], left[2], up[0], up[1], up[2], forward[0], forward[1], forward[2]);
}

ArticulatedRagDoll::ArticulatedRagDoll(
	btDynamicsWorld* ownerWorld,
	SkeletalMotion* skeletalMotion,
	int skeletonIndex,
	const btVector3& positionOffset,
	btScalar scale)
: m_ownerWorld(ownerWorld), m_skeletalMotion(skeletalMotion)
{
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
		rigidBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_bodies.push_back(rigidBody);

		btPoint2PointConstraint* childToJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[parentJointName], *rigidBody, btVector3(0, 0, 0), transform.inverse()*parentPosition);

		m_jointConstraints[parentJointName] = childToJointContraint;
		m_ownerWorld->addConstraint(childToJointContraint);

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
		if (m_jointKinematicBody[joint.first])
			m_jointKinematicBody[joint.first]->setActivationState(DISABLE_DEACTIVATION);

		if (m_jointKinematicMotionStates[joint.first])
			m_jointKinematicMotionStates[joint.first]->setKinematicPos(btTransform(btMatrix3x3::getIdentity(), joint.second));
	}
}