/* 
	BVHGYM: Loads and plays skeletal animation in a physically simulated environment
	Copyright(C) 2017 Vincent Petrella

	This program is free software : you can redistribute it and / or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.If not, see <https://www.gnu.org/licenses/>.

	////// Bullet license information

		Bullet Continuous Collision Detection and Physics Library
		Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

		This software is provided 'as-is', without any express or implied warranty.
		In no event will the authors be held liable for any damages arising from the use of this software.
		Permission is granted to anyone to use this software for any purpose,
		including commercial applications, and to alter it and redistribute it freely,
		subject to the following restrictions:

		1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software.
		If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
		2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
		3. This notice may not be removed or altered from any source distribution.

	//////
*/

#include "articulated_ragdoll.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

#include <vector>
#include <stack>

using namespace std;

#ifndef M_PI
#define M_PI		btScalar(3.14159265358979323846)
#endif

#ifndef M_PI_2
#define M_PI_2		btScalar(1.57079632679489661923)
#endif

#ifndef M_PI_4
#define M_PI_4		btScalar(0.785398163397448309616)
#endif

#ifndef EPSILON
#define EPSILON		1.192092896e-07F
#endif

void UpdateCOMBuffer(btVector3 &com, btVector3 *buffer)
{
	btVector3 lastValue = buffer[0];
	for (int i = 0; i < SMOOTHING_WINDOW_SIZE - 1; i++)
	{
		btVector3 tmpValue = buffer[i + 1];
		buffer[i + 1] = lastValue;
		lastValue = tmpValue;
	}

	buffer[0] = com;
}

btVector3 SmoothedBuffer(btVector3 *buffer)
{
	// ASSUMING SMOOTHING_WINDOW_SIZE 4
	//return 0.5*buffer[0] + 0.3*buffer[1] + 0.1*buffer[2] + 0.1*buffer[3];
	btVector3 result(0, 0, 0);

	for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++)
	{
		result += buffer[i];
	}

	return (result / SMOOTHING_WINDOW_SIZE);
}

btVector3 RagdollWithKinematicBodiesConstraints::GetSmoothCOMVelocity()
{
	return SmoothedBuffer(m_COMVelocities);
}

btVector3 RagdollWithKinematicBodiesConstraints::GetSmoothCOMAcceleration()
{
	return SmoothedBuffer(m_COMAccelerations);
}

btVector3 RagdollWithKinematicBodiesConstraints::GetSmoothCOMAngularMomentum()
{
	return SmoothedBuffer(m_angularMomentums);
}

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
	m_lastTime = m_clock.getTimeSeconds();
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
	for (auto bone : bonesByJointNames)
	{
		string parentJointName = bone.first;
		string childJointName = bone.second;

		btVector3 parentPosition = jointPositionsByNames[parentJointName];
		btVector3 childPosition = jointPositionsByNames[childJointName];


		// Comput look at rotation to align the capsule 
		btQuaternion localOrn = btQuaternion::getIdentity();

		btVector3 diff = childPosition - parentPosition;
		btScalar lenSqr = diff.length2();
		btScalar height = 0.f;

		if (lenSqr > 1.192092896e-07F + EPSILON)
		{
			height = btSqrt(lenSqr);
			btVector3 ax = diff / height;

			btVector3 zAxis(0, 0, 1);
			localOrn = shortestArcQuat(zAxis, ax);
		}
		btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(0.07*height, 0.6*height);

		m_masses.push_back(height + EPSILON);
		btTransform localTransform(localOrn, 0.5*(childPosition + parentPosition));
		
		m_shapes.push_back(capsuleShape);

		btRigidBody* rigidBody = createRigidBody(btScalar(10.0), localTransform, capsuleShape);

		m_compoundShape.addChildShape(localTransform, capsuleShape);

		rigidBody->setDamping(btScalar(0.3), btScalar(0.85));
		rigidBody->setDeactivationTime(btScalar(2));
		rigidBody->setSleepingThresholds(btScalar(0), btScalar(0));
		rigidBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_bodies.push_back(rigidBody);

		btPoint2PointConstraint* childToJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[parentJointName], *rigidBody, btVector3(0, 0, 0), localTransform.inverse()*parentPosition);
		btPoint2PointConstraint* childToChildJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[childJointName], *rigidBody, btVector3(0, 0, 0), localTransform.inverse()*childPosition);

		m_jointConstraints.push_back(childToJointContraint);
		m_jointConstraints.push_back(childToChildJointContraint);
		m_ownerWorld->addConstraint(childToJointContraint);
		m_ownerWorld->addConstraint(childToChildJointContraint);
	}

	// Christmas tree person !
	//for (auto bone : bonesByJointNames)
	//{
	//	string parentJointName = bone.first;
	//	string childJointName = bone.second;

	//	btVector3 parentPosition = jointPositionsByNames[parentJointName];
	//	btVector3 childPosition = jointPositionsByNames[childJointName];


	//	float segmentLength = parentPosition.distance(childPosition);

	//	//btCapsuleShape* shape = new btCapsuleShape(0.1, segmentLength);

	//	btTransform transform = jointTransformsByNames[parentJointName];
	//	transform.setOrigin(0.5 * (parentPosition + childPosition));

	//	btBoxShape* shape = new btBoxShape(segmentLength*btVector3(0.1, 0.1, 0.1));

	//	m_shapes.push_back(shape);

	//  m_compoundShape.addChildShape(transform, capsuleShape);

	//	btRigidBody* rigidBody = createRigidBody(btScalar(10.0), transform, shape);

	//	rigidBody->setDamping(btScalar(0.3), btScalar(0.85));
	//	rigidBody->setDeactivationTime(btScalar(2));
	//	rigidBody->setSleepingThresholds(btScalar(0), btScalar(0));
	//	//rigidBody->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	//	m_bodies.push_back(rigidBody);

	//	btPoint2PointConstraint* childToJointContraint = new btPoint2PointConstraint(*m_jointKinematicBody[parentJointName], *rigidBody, btVector3(0, 0, 0), transform.inverse()*parentPosition);

	//	m_jointConstraints.push_back(childToJointContraint);
	//	m_ownerWorld->addConstraint(childToJointContraint);

	//}
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
	float time = m_clock.getTimeSeconds();
	float deltaTime = time - m_lastTime;
	m_lastTime = time;

	unordered_map<string, btVector3> jointPositions;
	m_skeletalMotion->QuerySkeletalAnimation(frameIndex, 0, true, NULL, &jointPositions, NULL, NULL);

	for (auto joint : jointPositions)
	{
		if (m_jointKinematicBody[joint.first])
			m_jointKinematicBody[joint.first]->setActivationState(DISABLE_DEACTIVATION);

		if (m_jointKinematicMotionStates[joint.first])
			m_jointKinematicMotionStates[joint.first]->setKinematicPos(btTransform(btMatrix3x3::getIdentity(), joint.second));
	}

	// Compute COM Position, estimate Velocity and acceleration

	btCompoundShapeChild* children = m_compoundShape.getChildList();
	for (int i = 0; i < m_bodies.size(); i++)
	{
		children[i].m_transform = m_bodies[i]->getWorldTransform();
	}
	btTransform principalAxes;
	btVector3 inertia;
	this->m_compoundShape.calculatePrincipalAxisTransform(&(m_masses[0]), principalAxes, inertia);

	UpdateCOMBuffer(principalAxes.getOrigin(), m_COMPositions);

	UpdateCOMBuffer((0.8*(m_COMPositions[0] - m_COMPositions[1]) + 0.2*(m_COMPositions[2] - m_COMPositions[3])) / deltaTime, m_COMVelocities);

	UpdateCOMBuffer((m_COMPositions[0] - 2 * m_COMPositions[1] + m_COMPositions[2]) / (deltaTime*deltaTime), m_COMAccelerations);

	btVector3 totalAngularMomentum(0, 0, 0);

	m_debugBodiesMomentum.clear();
	m_debugBodyCOMs.clear();

	// Now let us compute the total Angular Momentum;
	float totalMass = GetTotalMass();
	for (int i = 0; i < m_bodies.size(); i++)
	{
		btScalar mass = m_masses[i] / totalMass;
		btVector3 bodyInertiaLocal;
		children[i].m_childShape->calculateLocalInertia(mass, bodyInertiaLocal);

		btTransform bodyToWorldRotation = btTransform::getIdentity();
		bodyToWorldRotation.setRotation(m_bodies[i]->getWorldTransform().getRotation());

		btVector3 bodyAngularVelocityLocal = bodyToWorldRotation.inverse() * m_bodies[i]->getAngularVelocity();

		btVector3 bodyAngularMomentumWorld = (bodyToWorldRotation * (bodyInertiaLocal * bodyAngularVelocityLocal));
		
		m_debugBodiesMomentum.push_back(bodyAngularMomentumWorld);

		btVector3 bodyCOM = m_bodies[i]->getCenterOfMassPosition();

		m_debugBodyCOMs.push_back(bodyCOM);

		btVector3 bodyCOMFromTotalCOM = bodyCOM - GetCOMPosition();
		btVector3 bodyVelFromTotalVel = m_bodies[i]->getLinearVelocity() - GetCOMVelocity();

		totalAngularMomentum += mass * (bodyCOMFromTotalCOM.cross(bodyVelFromTotalVel));
	}

	UpdateCOMBuffer(totalAngularMomentum, m_angularMomentums);
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


MultiBodyArticulatedRagdoll::MultiBodyArticulatedRagdoll(
	btMultiBodyDynamicsWorld* ownerWorld,
	SkeletalMotion* skeletalMotion,
	int skeletonIndex,
	const btVector3& positionOffset,
	btScalar scale)
{
	m_ownerWorld = ownerWorld;
	m_skeletalMotion = skeletalMotion;
	unordered_map<string, btVector3> jointPositionsByNames;
	unordered_map<string, btVector3> jointPositionsByNamesWithOffset;
	unordered_map<string, btTransform> jointTransformsByNames;
	m_skeletalMotion->QuerySkeletalAnimation(0, 0, false, NULL, &jointPositionsByNames, NULL, &jointTransformsByNames);
	m_skeletalMotion->QuerySkeletalAnimation(0, 0, true, NULL, &jointPositionsByNamesWithOffset, NULL, NULL);
	
	vector<pair<string, string>> bonesByJointNames;
	unordered_map<string, SkeletonJoint*> jointsByNames;

	m_skeletalMotion->GetRoot(0)->QuerySkeleton(&jointsByNames, &bonesByJointNames);

	int currentLink = 0;
	for (auto bone : bonesByJointNames)
	{
		m_linkIndicesByName[bone.first + bone.second] = currentLink++;
	}

	//init the base	
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape *pTempBox = new btBoxShape(btVector3(5,5,5));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	int numLinks = jointPositionsByNames.size();

	bool canSleep = false;
	bool fixedBase = false;

	m_multiBody = new btMultiBody(jointPositionsByNames.size(), baseMass, baseInertiaDiag, fixedBase, canSleep);
	
	m_multiBody->setBasePos(jointPositionsByNamesWithOffset[m_skeletalMotion->GetRoot(0)->GetName()]);

	for (auto bone : bonesByJointNames)
	{

		string startJointName = bone.first;
		string endJointName = bone.second;

		// Get parent bone:
		pair<string, string> parentBone = pair<string, string>("World", m_skeletalMotion->GetRoot(0)->GetName());
		for (auto currentBone : bonesByJointNames)
		{
			if (currentBone.second == startJointName)
				parentBone = currentBone;
		}

		if (!parentBone.first.compare("World"))
		{
			btVector3 currentPivotToCurrentCOM = 0.5*jointsByNames[endJointName]->GetLocalOffset();

			btTransform parentToChildTransform = m_skeletalMotion->GetLocalTransformByName(startJointName, 0);

			int linkIndex = m_linkIndicesByName[bone.first + bone.second];
			int parentLinkIndex = -1;

			m_multiBody->setupSpherical(linkIndex, 0, btVector3(0,0,0), parentLinkIndex, parentToChildTransform.getRotation().inverse(), jointPositionsByNames[startJointName], currentPivotToCurrentCOM);
		}
		else
		{
			btVector3 parentCOMToCurrentPivot = 0.5*jointsByNames[startJointName]->GetLocalOffset();
			btVector3 currentPivotToCurrentCOM = 0.5*jointsByNames[endJointName]->GetLocalOffset();

			btTransform parentToChildTransform = m_skeletalMotion->GetLocalTransformByName(startJointName, 0);

			int linkIndex = m_linkIndicesByName[bone.first + bone.second];
			int parentLinkIndex = m_linkIndicesByName[parentBone.first + parentBone.second];

			m_multiBody->setupSpherical(linkIndex, 0, btVector3(0, 0, 0), parentLinkIndex, parentToChildTransform.getRotation().inverse(), parentCOMToCurrentPivot, currentPivotToCurrentCOM);
		}
	}

	m_multiBody->finalizeMultiDof();
	ownerWorld->addMultiBody(m_multiBody);
	
	for (auto bone : bonesByJointNames)
	{
		string parentJointName = bone.first;
		string childJointName = bone.second;

		btVector3 parentPosition = btVector3(0, 0, 0);
		btVector3 childPosition = jointsByNames[childJointName]->GetLocalOffset();


		//// Comput look at rotation to align the capsule 
		//btQuaternion localOrn = btQuaternion::getIdentity();

		//btVector3 diff = childPosition - parentPosition;
		//btScalar lenSqr = diff.length2();
		//btScalar height = 0.f;

		//if (lenSqr > 1.192092896e-07F)
		//{
		//	height = btSqrt(lenSqr);
		//	btVector3 ax = diff / height;

		//	btVector3 zAxis(0, 0, 1);
		//	localOrn = shortestArcQuat(zAxis, ax);
		//}
		//btCapsuleShapeZ* shape = new btCapsuleShapeZ(0.1*height, 0.6*height);
		//
		btTransform localTransform = btTransform::getIdentity();//btTransform(localOrn, btVector3(0, 0, 0));
		float length = (childPosition - parentPosition).length();
		//btBoxShape* shape = new btBoxShape(btVector3(0.1*length, 0.1*length, 0.1*length));

		btSphereShape* shape = new btSphereShape(0.1);

		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_multiBody, m_linkIndicesByName[bone.first + bone.second]);

		col->setCollisionShape(shape);

		col->setWorldTransform(localTransform);

		ownerWorld->addCollisionObject(col, 2, 1 + 2);
		col->setFriction(0.1f);
		m_multiBody->getLink(m_linkIndicesByName[bone.first + bone.second]).m_collider = col;
		m_multiBody->getLink(m_linkIndicesByName[bone.first + bone.second]).m_collider->setWorldTransform(localTransform);
	}
}

void MultiBodyArticulatedRagdoll::UpdateJointPositions(int frameIndex)
{
	unordered_map<string, btTransform> jointTranforms;
	m_skeletalMotion->QuerySkeletalAnimation(frameIndex, 0, true, NULL, NULL, NULL, &jointTranforms);
	m_multiBody->setBaseWorldTransform(jointTranforms[m_skeletalMotion->GetRoot(0)->GetName()]);
	
	vector<pair<string, string>> bonesByName;
	m_skeletalMotion->GetRoot(0)->QuerySkeleton(NULL, &bonesByName);
	for (auto bone : bonesByName)
	{
		btQuaternion jointRotation = m_skeletalMotion->GetLocalTransformByName(bone.first, frameIndex).getRotation();
		m_multiBody->setJointPosMultiDof(m_linkIndicesByName[bone.first + bone.second], jointRotation);
	}
}

void MultiBodyArticulatedRagdoll::GetConstraintsJointPositions(std::vector<btVector3> &resultVector){}

void MultiBodyArticulatedRagdoll::KillRagdoll(){}

MultiBodyArticulatedRagdoll::~MultiBodyArticulatedRagdoll()
{
	if (m_multiBody)
	{
		delete m_multiBody;
	}
}
