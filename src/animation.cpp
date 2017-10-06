#include <fstream>
#include <iostream>
#include <string.h>
#include <stack>
#include "animation.h"

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

void QuerySkeletalAnimationRecursive
(
/*Defines the recursion parameters */
SkeletonJoint* joint,
btTransform& cumulativeTransform,
unordered_map<string, vector<btTransform>>& jointTransforms,
/*Defines the query inputs*/
int frameIndex,
int skeletonIndex,
/*Defines the query outputs*/
vector<btVector3>* jointPositions,
unordered_map<string, btVector3>* jointPositionsByName,
vector<pair<btVector3, btVector3>>* segmentPositions,
unordered_map<string, btTransform>* cumulativeTransformsByName
)
{
	btVector3 jointPositionL = joint->GetLocalOffset();
	btVector3 jointPositionW = cumulativeTransform * btVector4(jointPositionL[0], jointPositionL[1], jointPositionL[2], 1);

	if (cumulativeTransformsByName)
	{
		if (cumulativeTransformsByName->find(joint->GetName()) == cumulativeTransformsByName->end())
			cumulativeTransformsByName->emplace(joint->GetName(), cumulativeTransform);
	}

	if (jointPositions)
		jointPositions->push_back(jointPositionW);

	if (jointPositionsByName)
	{
		if (jointPositionsByName->find(joint->GetName()) == jointPositionsByName->end())
			jointPositionsByName->emplace(joint->GetName(), jointPositionW);
	}

	btTransform nextCumulativeTransform;
	if (joint->GetChildrenPointers().size()) // Leaf joints do not have transforms, let's not try looking for them
		nextCumulativeTransform = cumulativeTransform * jointTransforms[joint->GetName()][frameIndex];

	for (auto child : joint->GetChildrenPointers())
	{
		if (segmentPositions)
		{
			btVector3 childPositionL = child->GetLocalOffset();
			btVector3 childPositionW = nextCumulativeTransform * btVector4(childPositionL[0], childPositionL[1], childPositionL[2], 1);

			segmentPositions->push_back(pair<btVector3, btVector3>(nextCumulativeTransform.getOrigin(), childPositionW));
		}

		QuerySkeletalAnimationRecursive(
			child,
			nextCumulativeTransform,
			jointTransforms,
			frameIndex,
			skeletonIndex,
			jointPositions,
			jointPositionsByName,
			segmentPositions,
			cumulativeTransformsByName);
		
	}
}

void SkeletalMotion::QuerySkeletalAnimation
(
/*Defines the query inputs*/
int frameIndex,
int skeletonIndex,
bool addRootOffset,
/*Defines the query outputs*/
vector<btVector3>* jointPositions,
unordered_map<string, btVector3>* jointPositionsByName,
vector<pair<btVector3, btVector3>>* segmentPositions,
unordered_map<string, btTransform>* cumulativeTransformsByName
)
{
	if (!jointPositions && !jointPositionsByName && !segmentPositions && !cumulativeTransformsByName)
		return;

	SkeletonJoint* root = m_skeletonRoots[skeletonIndex];

	btTransform rootTransform = btTransform::getIdentity();

	if (addRootOffset)
		rootTransform.setOrigin(m_rootTrajectories[frameIndex][skeletonIndex]);

	QuerySkeletalAnimationRecursive
	(
		root, 
		rootTransform, 
		m_jointTransforms,
		frameIndex, 
		skeletonIndex,
		jointPositions,
		jointPositionsByName, 
		segmentPositions,
		cumulativeTransformsByName
	);
}