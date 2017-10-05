#include "btBulletDynamicsCommon.h"
#include "animation_clip.h"
#include <vector>

class ArticulatedRagDoll
{
public:

	ArticulatedRagDoll( btDynamicsWorld* ownerWorld,
						SkeletalMotion* skeletalMotion,
						int skeletonIndex,
						const btVector3& positionOffset, 
						btScalar scale);

	~ArticulatedRagDoll();

private:
	btDynamicsWorld* m_ownerWorld;
	std::vector<btCollisionShape*> m_shapes;
	std::vector<btRigidBody*> m_bodies;
	std::vector<btTypedConstraint*> m_joints;

	btRigidBody* createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);
};