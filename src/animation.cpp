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

	for (auto child : joint->GetDirectChildren())
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
float skeletonScale,
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
	btVector3 jointPositionW = cumulativeTransform * btVector4(jointPositionL[0], jointPositionL[1], jointPositionL[2], 1) * skeletonScale;

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
	if (joint->GetDirectChildren().size()) // Leaf joints do not have transforms, let's not try looking for them
		nextCumulativeTransform = cumulativeTransform * jointTransforms[joint->GetName()][frameIndex];

	for (auto child : joint->GetDirectChildren())
	{
		if (segmentPositions)
		{
			btVector3 childPositionL = child->GetLocalOffset();
			btVector3 childPositionW = nextCumulativeTransform * btVector4(childPositionL[0], childPositionL[1], childPositionL[2], 1) * skeletonScale;

			segmentPositions->push_back(pair<btVector3, btVector3>(nextCumulativeTransform.getOrigin() * skeletonScale, childPositionW));
		}

		QuerySkeletalAnimationRecursive(
			child,
			nextCumulativeTransform,
			jointTransforms,
			skeletonScale,
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

	// Build world pose of the skeleton by traversing the skeleton recursively and fetching joint rotation at the right frame.
	// fills out provided containers while traversing

	QuerySkeletalAnimationRecursive
	(
		root, 
		rootTransform, 
		m_jointTransforms,
		m_skeletonScale,
		frameIndex, 
		skeletonIndex,
		jointPositions,
		jointPositionsByName, 
		segmentPositions,
		cumulativeTransformsByName
	);
}

void SkeletonJoint::QuerySkeleton(unordered_map<string, SkeletonJoint*>* jointPointersByNames, vector<pair<string, string>>* bonesByJointNames)
{
	// Traverse skeleton recursively and fill out provided containers.

	if (jointPointersByNames)
	{
		if (jointPointersByNames->find(m_name) == jointPointersByNames->end())
			jointPointersByNames->emplace(m_name, this);
	}

	for (auto child : m_childJoints)
	{
		if (bonesByJointNames)
		{
			bonesByJointNames->push_back(pair<string, string>(m_name, child->GetName()));
		}

		child->QuerySkeleton(jointPointersByNames, bonesByJointNames);
	}
}

void SkeletalAnimationPlayer::UpdatePlayer()
{
	float time = m_clock.getTimeSeconds();
	float dt = time - m_lastTime;
	if (dt > 0.1)
		dt = 0.1;
	m_lastTime = time;
	if(m_playerState == IS_PLAYING)
	{
		m_currentAnimationFrame += dt * m_motionSpeed * m_skeletalMotion->GetSamplingRate();
		if (m_currentAnimationFrame >= m_skeletalMotion->GetFrameCount())
		{
			m_currentAnimationFrame = m_skeletalMotion->GetFrameCount();
			std::cout << "Animation Ended. Press Enter to start over.\n";
			m_playerState = IS_WAITING_START;
		}
	}
}

void SkeletalMotion::SetNormalizedScale()
{
	vector<btVector3> jointPositions;
	QuerySkeletalAnimation(0, 0, false, &jointPositions, NULL, NULL, NULL);

	float maxLength = 0;
	for (btVector3 position : jointPositions)
	{
		if (maxLength < position.length())
			maxLength = position.length();
	}

	m_skeletonScale = 1.0f / maxLength;
}

SkeletalMotion::~SkeletalMotion()
{

}