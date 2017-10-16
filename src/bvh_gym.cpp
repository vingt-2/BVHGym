/*
	Altered sources of the BasicExample to get a GUI up and running quickly
*/

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

void BVHGym::initPhysics()
{
	m_animationPlayer = new SkeletalAnimationPlayer();

	SetBVHAnimation(SkeletalMotion::BVHImport("111_02.bvh"));// 91_61.bvh")); 

	//m_skeletalMotion->SetScale(0.1f);

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


void BVHGym::renderScene()
{
	m_dynamicsWorld->getDebugDrawer()->flushLines();

	m_animationPlayer->UpdatePlayer();

	int animationFrame = m_animationPlayer->GetCurrentAnimationFrame();

	m_articulatedRagdoll->UpdateJointPositions(animationFrame);

	CommonMultiBodyBase::renderScene();

	if (m_comPositions.size())
	{
		if(m_comPositions[m_comPositions.size() - 1] != m_articulatedRagdoll->GetCOMPosition())
			m_comPositions.push_back(m_articulatedRagdoll->GetCOMPosition());
	}
	else
		m_comPositions.push_back(m_articulatedRagdoll->GetCOMPosition());

	if (m_comPositions.size() > 1)
	{
		m_comVelocities.push_back(m_comPositions[m_comPositions.size() - 1] - m_comPositions[m_comPositions.size() - 2]);
		btVector3 totalAngularMomentum = m_articulatedRagdoll->GetCOMAngularMomentum() - m_articulatedRagdoll->GetTotalMass()*m_comPositions[m_comPositions.size()].cross(m_comPositions[m_comPositions.size()]);
		m_angularMomentum.push_back(totalAngularMomentum);
	}

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

			m_dynamicsWorld->getDebugDrawer()->drawLine(velocityDrawStart, velocityDrawStart + 10 * smoothVelocityEstimate, { 1, 0, 0 });

			m_dynamicsWorld->getDebugDrawer()->drawLine(velocityDrawStart, velocityDrawStart + smoothAngularMomentumEstimate, { 0, 1, 0 });

			// Draw Center of mass on actual position and also projected on y=0
			m_dynamicsWorld->getDebugDrawer()->drawSphere(m_comPositions[m_comPositions.size() - 1], 0.3, { 1, 1, 1 });
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



