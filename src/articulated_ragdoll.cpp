#include "articulated_ragdoll.h"


#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"

#include "LinearMath/btVector3.h"

#include <vector>

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
	m_skeletalMotion->GetJointPositionsByName(jointPositions, 0, 0, true);

	SkeletonJoint* currentJoint = m_skeletalMotion->GetRoot(0);

	for (auto joint : jointPositions)
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
	}
	//while (currentJoint)
	//{
	//	btVector3 jointPosition = jointPositions[currentJoint->GetName()];
	//	btVector4 jointPositionH(jointPosition[0], jointPosition[1], jointPosition[2], 1.0);

	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(0, new KinematicMotionState(btTransform(btMatrix3x3::getIdentity(),jointPosition)), new btSphereShape(0.01), btVector3(0, 0, 0));
	//	btRigidBody* kinematicJointBody = new btRigidBody(rbInfo);

	//	for (auto child : currentJoint->GetChildrenPointers())
	//	{
	//		btVector3 childPosition = jointPositions[child->GetName()];
	//		float segmentLength = jointPosition.distance(childPosition);
	//		//btCapsuleShape* shape = new btCapsuleShape(0.1*segmentLength, segmentLength);

	//		btBoxShape* shape = new btBoxShape(btVector3(1, 1, 1));
	//		m_shapes.push_back(shape);

	//		btTransform transform;
	//		transform.setIdentity();
	//		transform.setOrigin(0.5 * (jointPosition + childPosition));

	//		btRigidBody* rigidBody = createRigidBody(btScalar(1.0), transform, shape);

	//		rigidBody->setDamping(btScalar(0.05), btScalar(0.85));
	//		rigidBody->setDeactivationTime(btScalar(0.8));
	//		rigidBody->setSleepingThresholds(btScalar(1.6), btScalar(2.5));
	//		m_bodies.push_back(createRigidBody(btScalar(1.0), transform, shape));

	//		btVector3 jointInRigidBodyLocal = transform.inverse() * jointPositionH;

	//		btPoint2PointConstraint* ChildToJointContraint = new btPoint2PointConstraint(*kinematicJointBody, *rigidBody, btVector3(0, 0, 0), jointInRigidBodyLocal);
	//	}

	//	//btPoint2PointConstraint* parentToJointContraint = new btPoint2PointConstraint(*kinematicJointBody, *rigidBody, btVector3(0, 0, 0), jointInRigidBodyLocal);
	//}

	//for (auto joint : joints)
	//{
	//	string jointName = joint.first;
	//	btVector3 jointPosition
	//	btBoxShape* box = new btBoxShape(btVector3(1, 1, 1));
	//	m_shapes.push_back(box);
	//	
	//	btRigidBody* rigidBody = createRigidBody(btScalar(1.0), btTransform::getIdentity(), box);
	//	
	//	//btRigidBody* rigidBody
	//	
	//	rigidBody->setDamping(btScalar(0.05), btScalar(0.85));
	//	rigidBody->setDeactivationTime(btScalar(0.8));
	//	rigidBody->setSleepingThresholds(btScalar(1.6), btScalar(2.5));

	//	m_bodies.push_back(rigidBody);
	//}

	//vector<pair<btVector3, btVector3>> segments;
	//m_skeletalMotion->GetSkeletalSegments(segments, 0, 0, true);
	//for (auto segment : segments)
	//{
	//	float segmentLength = segment.first.distance(segment.second);
	//	btCapsuleShape* capsule = new btCapsuleShape(0.1*segmentLength, segmentLength);
	//	m_shapes.push_back(capsule);
	//	
	//	btTransform transform;
	//	transform.setIdentity();
	//	transform.setOrigin(0.5 * (segment.first+segment.second));

	//	btRigidBody* rigidBody = createRigidBody(btScalar(1.0), transform, capsule);

	//	rigidBody->setDamping(btScalar(0.05), btScalar(0.85));
	//	rigidBody->setDeactivationTime(btScalar(0.8));
	//	rigidBody->setSleepingThresholds(btScalar(1.6), btScalar(2.5));
	//	m_bodies.push_back(createRigidBody(btScalar(1.0), transform, capsule));
	//}

	//// Now setup the constraints
	//btHingeConstraint* hingeC;
	//btConeTwistConstraint* coneC;

	//btTransform localA, localB;

	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, M_PI_2, 0); localA.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, M_PI_2, 0); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
	//hingeC = new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
	//hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
	//m_joints[JOINT_PELVIS_SPINE] = hingeC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, 0, M_PI_2); localA.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, 0, M_PI_2); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
	//coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
	//coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
	//m_joints[JOINT_SPINE_HEAD] = coneC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, 0, -M_PI_4 * 5); localA.setOrigin(scale*btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, 0, -M_PI_4 * 5); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
	//coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
	//coneC->setLimit(M_PI_4, M_PI_4, 0);
	//m_joints[JOINT_LEFT_HIP] = coneC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, M_PI_2, 0); localA.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, M_PI_2, 0); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
	//hingeC = new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
	//hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	//m_joints[JOINT_LEFT_KNEE] = hingeC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, 0, M_PI_4); localA.setOrigin(scale*btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, 0, M_PI_4); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
	//coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
	//coneC->setLimit(M_PI_4, M_PI_4, 0);
	//m_joints[JOINT_RIGHT_HIP] = coneC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, M_PI_2, 0); localA.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, M_PI_2, 0); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
	//hingeC = new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
	//hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	//m_joints[JOINT_RIGHT_KNEE] = hingeC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, 0, M_PI); localA.setOrigin(scale*btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, 0, M_PI_2); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
	//coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
	//coneC->setLimit(M_PI_2, M_PI_2, 0);
	//m_joints[JOINT_LEFT_SHOULDER] = coneC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, M_PI_2, 0); localA.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, M_PI_2, 0); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
	//hingeC = new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
	//hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
	//m_joints[JOINT_LEFT_ELBOW] = hingeC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, 0, 0); localA.setOrigin(scale*btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, 0, M_PI_2); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
	//coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
	//coneC->setLimit(M_PI_2, M_PI_2, 0);
	//m_joints[JOINT_RIGHT_SHOULDER] = coneC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

	//localA.setIdentity(); localB.setIdentity();
	//localA.getBasis().setEulerZYX(0, M_PI_2, 0); localA.setOrigin(scale*btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
	//localB.getBasis().setEulerZYX(0, M_PI_2, 0); localB.setOrigin(scale*btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
	//hingeC = new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
	//hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
	//m_joints[JOINT_RIGHT_ELBOW] = hingeC;
	//m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
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
	m_skeletalMotion->GetJointPositionsByName(jointPositions, 0, frameIndex, true);

	for (auto joint : jointPositions)
	{
		m_jointKinematicMotionStates[joint.first]->setKinematicPos(btTransform(btMatrix3x3::getIdentity(), joint.second));
	}
}