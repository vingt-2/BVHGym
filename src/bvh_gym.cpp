/*
	Altered sources of the BasicExample to get a GUI up and running quickly
*/

#include "bvh_gym.h"
#include <iostream>

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

	SetBVHAnimation(SkeletalMotion::BVHImport("test.bvh"));

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
	groundTransform.setOrigin(btVector3(0,-50,0));

	{
		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}


	{
		int size = 16;

		float sizeX = 1.f;
		float sizeY = 1.f;

		//int rc=0;

		btScalar scale(3.5);
		btVector3 pos(0.0f, sizeY, 0.0f);
		
		m_articulatedRagdoll = new RagdollWithKinematicBodiesConstraints(m_dynamicsWorld, m_skeletalMotion, 0, btVector3(), 1);
	}

	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
}


void BVHGym::renderScene()
{
	m_dynamicsWorld->getDebugDrawer()->flushLines();

	CommonRigidBodyBase::renderScene();

	m_animationPlayer->UpdatePlayer();

	int animationFrame = m_animationPlayer->GetCurrentAnimationFrame();

	//std::cout << "Frame " << animationFrame << " out of " << m_skeletalMotion->GetFrameCount() << ".\n";

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

	//std::vector<btVector3> constraintPivots;
	//m_articulatedRagdoll->GetConstraintsJointPositions(constraintPivots);
	//for (auto position : constraintPivots)
	//{
	//	m_dynamicsWorld->getDebugDrawer()->drawSphere(position, 0.3, { 0, 1, 0 });
	//}

	m_articulatedRagdoll->UpdateJointPositions(animationFrame);

	ClearKeyPressed();
}


CommonExampleInterface*   BVHGymCreateFunc(CommonExampleOptions& options)
{
	return new BVHGym(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BVHGymCreateFunc)



