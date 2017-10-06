#include <fstream>
#include <iostream>
#include <string.h>
#include "animation.h"

using namespace std;

void PrintJointRecursive(SkeletonJoint* joint, int depth)
{
	string out = "";
	for (int i = 0; i < depth; i++)
		out += "_";
	cout << out + joint->GetName() + "\n";

	for (auto child : joint->GetChildrenPointers())
		PrintJointRecursive(child, depth + 1);
}

void SkeletonJoint::PrintJoint()
{
	PrintJointRecursive(this, 0);
}

void GetSkeletalSegmentsRecurse(vector<pair<btVector3, btVector3>>& vertices,
	SkeletonJoint* joint,
	btVector3& jointPositionW,
	btTransform& cumulativeTransform,
	unordered_map<string, vector<btTransform>>* jointTransforms,
	int frameIndex,
	btVector3& rootTrajectory)
{
	for (auto child : joint->GetChildrenPointers())
	{
		btVector3 childPositionL = child->GetLocalOffset();
		btVector3 childPositionW = cumulativeTransform * btVector4(childPositionL[0], childPositionL[1], childPositionL[2], 1);

		childPositionW += rootTrajectory;

		vertices.push_back(pair<btVector3, btVector3>(jointPositionW, childPositionW));

		if (child->GetChildrenPointers().size())
		{
			btTransform nextCumulativeTransform = cumulativeTransform * jointTransforms->at(child->GetName())[frameIndex];
			GetSkeletalSegmentsRecurse(vertices, child, childPositionW, nextCumulativeTransform, jointTransforms, frameIndex, rootTrajectory);
		}
	}
}

void SkeletalMotion::GetSkeletalSegments(vector<pair<btVector3, btVector3>> &result, int skeletonIndex, int frameIndex, bool addRootTrajectory)
{
	SkeletonJoint* root = m_skeletonRoots[skeletonIndex];

	btTransform transform = m_jointTransforms[root->GetName()][frameIndex];

	btVector3 rootTrajOffset = addRootTrajectory ? m_rootTrajectories[frameIndex][skeletonIndex] : btVector3(0, 0, 0);

	btVector3 rootPositionW = root->GetLocalOffset() + rootTrajOffset;

	GetSkeletalSegmentsRecurse(result, root, rootPositionW, transform, &m_jointTransforms, frameIndex, rootTrajOffset);
}

void GetJointPositionsRecurse(
	vector<btVector3> &vertices,
	SkeletonJoint* joint,
	btTransform& cumulativeTransform,
	unordered_map<string, vector<btTransform>>* jointTransforms,
	int frameIndex,
	btVector3& rootTrajectory)
{
	for (auto child : joint->GetChildrenPointers())
	{
		btVector3 childPositionL = child->GetLocalOffset();
		btVector3 childPositionW = cumulativeTransform * btVector4(childPositionL[0], childPositionL[1], childPositionL[2], 1);

		childPositionW += rootTrajectory;

		vertices.push_back(childPositionW);

		if (child->GetChildrenPointers().size())
		{
			btTransform nextCumulativeTransform = cumulativeTransform * jointTransforms->at(child->GetName())[frameIndex];
			GetJointPositionsRecurse(vertices, child, nextCumulativeTransform, jointTransforms, frameIndex, rootTrajectory);
		}
	}
}

void SkeletalMotion::GetJointPositions(vector<btVector3> &result, int skeletonIndex, int frameIndex, bool addRootTrajectory)
{
	SkeletonJoint* root = m_skeletonRoots[skeletonIndex];

	btTransform transform = m_jointTransforms[root->GetName()][frameIndex];

	btVector3 rootTrajOffset = addRootTrajectory ? m_rootTrajectories[frameIndex][skeletonIndex] : btVector3(0, 0, 0);

	result.push_back(root->GetLocalOffset() + rootTrajOffset);

	GetJointPositionsRecurse(result, root, transform, &m_jointTransforms, frameIndex, rootTrajOffset);
}

void GetJointPositionsRecurseByNameRecurse(
	unordered_map<string, btVector3> &result,
	SkeletonJoint* joint,
	btTransform& cumulativeTransform,
	unordered_map<string, vector<btTransform>>* jointTransforms,
	int frameIndex,
	btVector3& rootTrajectory)
{
	for (auto child : joint->GetChildrenPointers())
	{
		btVector3 childPositionL = child->GetLocalOffset();
		btVector3 childPositionW = cumulativeTransform * btVector4(childPositionL[0], childPositionL[1], childPositionL[2], 1);

		childPositionW += rootTrajectory;

		result[child->GetName()] = childPositionW;

		if (child->GetChildrenPointers().size())
		{
			btTransform nextCumulativeTransform = cumulativeTransform * jointTransforms->at(child->GetName())[frameIndex];
			GetJointPositionsRecurseByNameRecurse(result, child, nextCumulativeTransform, jointTransforms, frameIndex, rootTrajectory);
		}
	}

}

void  SkeletalMotion::GetJointPositionsByName(unordered_map<string, btVector3> &result, int skeletonIndex, int frameIndex, bool addRootTrajectory)
{
	SkeletonJoint* root = m_skeletonRoots[skeletonIndex];

	btTransform transform = m_jointTransforms[root->GetName()][frameIndex];

	btVector3 rootTrajOffset = addRootTrajectory ? m_rootTrajectories[frameIndex][skeletonIndex] : btVector3(0, 0, 0);

	result[root->GetName()] = root->GetLocalOffset() + rootTrajOffset;

	GetJointPositionsRecurseByNameRecurse(result, root, transform, &m_jointTransforms, frameIndex, rootTrajOffset);
}