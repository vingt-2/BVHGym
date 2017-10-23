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


//	Altered sources of the BasicExample to get a GUI up and running quickly

#include "bvh_gym.h"
#include <iostream>

#define MAX(a,b) a > b ? a : b
#define MIN(a,b) a < b ? a : b

void BVHGym::ResetCamera()
{
	float dist = 4;
	float pitch = -35;
	float yaw = 52;
	float targetPos[3] = { 0, 0, 0 };
	m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
}

BVHGym::BVHGym(struct GUIHelperInterface* helper) : CommonMultiBodyBase(helper)
{
	m_bDrawSkeleton = false;
	m_bDrawCOMState = true;
	m_bShouldTerminate = false;

	m_animationPlayer = new SkeletalAnimationPlayer();
};

BVHGym::~BVHGym()
{
	delete m_skeletalMotion;
	delete m_animationPlayer;
}

void BVHGym::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///Create Ground
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-100,0));

	createRigidBody(0.0f,groundTransform,groundShape, btVector4(0,0,1,1));

	// Create Ragdoll
	m_articulatedRagdoll = new RagdollWithKinematicBodiesConstraints(m_dynamicsWorld, m_skeletalMotion, 0, btVector3(), 1);
	//m_articulatedRagdoll = new MultiBodyArticulatedRagdoll(m_dynamicsWorld, m_skeletalMotion, 0, btVector3(), 1);
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);	
}

void BVHGym::processCommandLineArgs(int argc, char* argv[])
{
	bool bAnimationLoaded = SetBVHAnimation(SkeletalMotion::BVHImport("test.bvh"));

	if (!bAnimationLoaded)
	{
		m_bShouldTerminate = true;
	}
}

// This is the Frame Update point of entry, we update the animation player, the physics, and the debug drawings in here.
void BVHGym::renderScene()
{
	// Remove previously setup debug draw calls
	m_dynamicsWorld->getDebugDrawer()->flushLines();

	// Update Animation Player
	m_animationPlayer->UpdatePlayer();

	// Query which animation frame we shall be playing
	int animationFrame = m_animationPlayer->GetCurrentAnimationFrame();

	// Update our articulated physics object to the specified animation frame.
	m_articulatedRagdoll->UpdateJointPositions(animationFrame);

	CommonMultiBodyBase::renderScene();

	// Fill out the center of mass positions buffer
	if (m_comPositions.size())
	{
		if(m_comPositions[m_comPositions.size() - 1] != m_articulatedRagdoll->GetCOMPosition())
			m_comPositions.push_back(m_articulatedRagdoll->GetCOMPosition());
	}
	else
		m_comPositions.push_back(m_articulatedRagdoll->GetCOMPosition());

	if (m_comPositions.size() > 1)
	{
		m_comVelocities.push_back((m_comPositions[m_comPositions.size() - 1] - m_comPositions[m_comPositions.size() - 2]) * m_skeletalMotion->GetSamplingRate());
		btVector3 totalAngularMomentum = m_articulatedRagdoll->GetCOMAngularMomentum() - m_articulatedRagdoll->GetTotalMass()*m_comPositions[m_comPositions.size()].cross(m_comPositions[m_comPositions.size()]);
		m_angularMomentum.push_back(totalAngularMomentum);
	}

	// Should we draw the skeleton, if yes draw segments as red lines and joints as red ballz
	if (m_bDrawSkeleton)
	{
		std::vector < std::pair<btVector3, btVector3>> segments;
		m_skeletalMotion->QuerySkeletalAnimation(animationFrame, 0, true, NULL, NULL, &segments, NULL);
		for (auto verticePair : segments)
		{
			m_dynamicsWorld->getDebugDrawer()->drawLine(verticePair.first, verticePair.second, { 1, 0, 0 });
		}
		
		std::vector<btVector3> positions;
		m_skeletalMotion->QuerySkeletalAnimation(animationFrame, 0, true, &positions, NULL, NULL, NULL);
		for (auto position : positions)
		{
			m_dynamicsWorld->getDebugDrawer()->drawSphere(position, 0.3, { 1, 0, 0 });
		}
	}

	// Should we draw the center of mass state (Position trail, velocity and angular momentum).
	if (m_bDrawCOMState)
	{
		int trailSize = 200;
		if (m_comPositions.size() > 10)
		{
			for (int i = MAX(m_comPositions.size() - trailSize - 1, 0); i < m_comPositions.size() - 1; i++)
			{
				m_dynamicsWorld->getDebugDrawer()->drawLine(m_comPositions[i], m_comPositions[i + 1], { 1, 1, 1 });
			}

			btVector3 smoothVelocityEstimate = 0.6*m_comVelocities[m_comVelocities.size() - 1] + 0.25*m_comVelocities[m_comVelocities.size() - 2] + 0.15*m_comVelocities[m_comVelocities.size() - 3];
			btVector3 smoothAngularMomentumEstimate;
			for (int i = 0; i < 10; i++)
			{
				smoothAngularMomentumEstimate += m_angularMomentum[m_angularMomentum.size() - i] / 10.0;
			}


			btVector3 velocityDrawStart = m_comPositions[m_comPositions.size() - 1];
			velocityDrawStart.setY(0);

			m_dynamicsWorld->getDebugDrawer()->drawLine(velocityDrawStart, velocityDrawStart + smoothVelocityEstimate, { 1, 0, 0 });

			m_dynamicsWorld->getDebugDrawer()->drawLine(velocityDrawStart, velocityDrawStart + smoothAngularMomentumEstimate, { 0, 1, 0 });

			// Draw Center of mass on actual position
			m_dynamicsWorld->getDebugDrawer()->drawSphere(m_comPositions[m_comPositions.size() - 1], 0.3, { 1, 1, 1 });
			// and also projected on y=0
			m_dynamicsWorld->getDebugDrawer()->drawSphere(velocityDrawStart, 0.3, { 1, 0, 0 });
		}
	}

	ClearKeyPressed();
}


CommonExampleInterface*   BVHGymCreateFunc(CommonExampleOptions& options)
{
	return new BVHGym(options.m_guiHelper);
}


B3_STANDALONE_EXAMPLE(BVHGymCreateFunc)



