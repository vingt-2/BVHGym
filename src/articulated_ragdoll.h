/*
	BVHGYM: Loads and plays skeletal animation in a physically simulated environment>
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
	virtual btVector3 GetCOMAcceleration() = 0;
	virtual btVector3 GetCOMAngularMomentum() = 0;

	virtual btVector3 GetSmoothCOMVelocity() = 0;
	virtual btVector3 GetSmoothCOMAcceleration() = 0;
	virtual btVector3 GetSmoothCOMAngularMomentum() = 0;

	virtual float		GetTotalMass() = 0;


	// For debug, remove later
	std::vector<btVector3>					m_debugBodiesMomentum;
	std::vector<btVector3>					m_debugBodyCOMs;

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

	virtual btVector3 GetCOMPosition() { return m_COMPositions[0]; };

	virtual btVector3 GetCOMVelocity() { return m_COMVelocities[0]; };
	virtual btVector3 GetCOMAcceleration() { return m_COMAccelerations[0]; };
	virtual btVector3 GetCOMAngularMomentum(){ return m_angularMomentums[0]; };
	
	virtual btVector3 GetSmoothCOMVelocity();
	virtual btVector3 GetSmoothCOMAcceleration();
	virtual btVector3 GetSmoothCOMAngularMomentum();


	virtual float		GetTotalMass()
	{
		float mass = 0;
		for (int i = 0; i < m_masses.size(); i++)
		{
			mass += m_masses[i];
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

#define POSITION_SMOOTHING_WINDOW_SIZE 1
#define VELOCITY_SMOOTHING_WINDOW_SIZE 16
#define ACCELERATION_SMOOTHING_WINDOW_SIZE 32

	std::vector<btVector3> m_COMPositions;
	std::vector<btVector3> m_COMVelocities;
	std::vector<btVector3> m_angularMomentums;
	std::vector<btVector3> m_COMAccelerations;

	btClock m_clock;
	float m_lastTime;
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