#include <fstream>
#include <iostream>
#include <string.h>
#include "animation_clip.h"

#define _HAS_ITERATOR_DEBUGGING 0

#ifndef M_PI
#define M_PI       btScalar(3.14159265358979323846)
#endif

using namespace std;

void InvalidBVH()
{
	cout << "There were invalid values encountered in your BVH file.\n";
}

#define INVALID_BVH {InvalidBVH(); return NULL;}

void ReadFrameRecursive(vector<string>& tokens, SkeletalMotion* motion, SkeletonJoint* joint, int &currentToken);
void PrintJointRecursive(SkeletonJoint* joint, int depth);
btMatrix3x3 GetRotationMatrix(int axis, btScalar angle);

int ChannelOrderToInt(string str)
{
	if (!str.compare("Xrotation"))
	{
		return 0;
	}
	else if (!str.compare("Yrotation"))
	{
		return 1;
	}
	else if (!str.compare("Zrotation"))
	{
		return 2;
	}
	else if (!str.compare("Xposition"))
	{
		return 0;
	}
	else if (!str.compare("Yposition"))
	{
		return 1;
	}
	else if (!str.compare("Zposition"))
	{
		return 2;
	}
	else
	{
		return -1;
	}
}

void tokenize(vector<string>& tokens, string str)
{
	string token = "";
	for (int i = 0; i < str.length(); i++)
	{
		if (i == str.length() - 1)
		{
			char a = str[i];
		}

		if (str[i] == '\0' || str[i] == '\n' || str[i] == ' ' || str[i] == '\r' || str[i] == '\t' || str[i] < 0)
		{
			if (token.length() > 0)
				tokens.push_back(token);

			token = "";
		}
		else
		{
			token += str[i];
		}
	}
}

SkeletonJoint* ParseJoint(vector<string> &tokens, int startToken, int* endToken, vector<int>* rootOrdering)
{
	string jointName = tokens[startToken + 1];

	SkeletonJoint* joint = new SkeletonJoint();
	joint->m_name = jointName;

	if (tokens[startToken + 2].compare("{"))
	{
		INVALID_BVH
	}

	if (tokens[startToken + 3].compare("OFFSET"))
	{
		INVALID_BVH
	}

	joint->m_localOffset = btVector3
	(
		btScalar(atof(tokens[startToken + 4].c_str())),
		btScalar(atof(tokens[startToken + 5].c_str())),
		btScalar(atof(tokens[startToken + 6].c_str()))
	);

	if (tokens[startToken + 7].compare("CHANNELS"))
	{
		INVALID_BVH
	}

	int currentToken;
	if (!tokens[startToken + 8].compare("3"))
	{
		currentToken = startToken + 12;
		joint->m_channelsOrdering[0] = ChannelOrderToInt(tokens[startToken + 9]);
		joint->m_channelsOrdering[1] = ChannelOrderToInt(tokens[startToken + 10]);
		joint->m_channelsOrdering[2] = ChannelOrderToInt(tokens[startToken + 11]);

		if (joint->m_channelsOrdering[0] == -1 ||
			joint->m_channelsOrdering[1] == -1 ||
			joint->m_channelsOrdering[2] == -1)
		{
			INVALID_BVH
		}
	}
	else if (!tokens[startToken + 8].compare("6"))
	{
		currentToken = startToken + 15;

		if (!rootOrdering)
			INVALID_BVH

			rootOrdering->push_back(ChannelOrderToInt(tokens[startToken + 9]));
		rootOrdering->push_back(ChannelOrderToInt(tokens[startToken + 10]));
		rootOrdering->push_back(ChannelOrderToInt(tokens[startToken + 11]));

		joint->m_channelsOrdering[0] = ChannelOrderToInt(tokens[startToken + 12]);
		joint->m_channelsOrdering[1] = ChannelOrderToInt(tokens[startToken + 13]);
		joint->m_channelsOrdering[2] = ChannelOrderToInt(tokens[startToken + 14]);
	}
	else
	{
		INVALID_BVH
	}

	// Now to read the child joints ...
	while (tokens[currentToken].compare("}"))
	{
		if (!tokens[currentToken].compare("JOINT"))
		{
			int currentEndToken = -1;
			SkeletonJoint* childJoint = ParseJoint(tokens, currentToken, &currentEndToken, NULL);
			if (childJoint)
			{
				joint->m_childJoints.push_back(childJoint);
				currentToken = currentEndToken + 1;
			}
		}
		else if (!tokens[currentToken].compare("End") && !tokens[currentToken + 1].compare("Site"))
		{
			if (!tokens[currentToken + 2].compare("{") && !tokens[currentToken + 3].compare("OFFSET") && !tokens[currentToken + 7].compare("}"))
			{
				SkeletonJoint* endJoint = new SkeletonJoint();
				endJoint->m_name = joint->m_name + "_end";
				endJoint->m_localOffset = btVector3
				(
					btScalar(atof(tokens[startToken + 4].c_str())),
					btScalar(atof(tokens[startToken + 5].c_str())),
					btScalar(atof(tokens[startToken + 6].c_str()))
				);
				joint->m_childJoints.push_back(endJoint);
				currentToken += 8;
			}
			else
			{
				INVALID_BVH
					currentToken++;
			}
		}
		else
		{
			cout << "Doing nothing for token " << tokens[currentToken] << "\n";
			currentToken++;
		}
	}
	*endToken = currentToken;

	return joint;
}

SkeletalMotion* SkeletalMotion::BVHImport(string bvhFilePath)
{
	SkeletalMotion* skeletalMotion = new SkeletalMotion();

	ifstream bvhFile(bvhFilePath, std::ifstream::binary);

	vector<string> tokens;
	if (bvhFile.is_open())
	{
		// get length of file:
		bvhFile.seekg(0, bvhFile.end);
		int length = bvhFile.tellg();
		bvhFile.seekg(0, bvhFile.beg);

		char * buffer = new char[length];

		// read data as a block:
		bvhFile.read(buffer, length);

		if (!bvhFile)
		{
			bvhFile.close();

			INVALID_BVH
		}
		bvhFile.close();

		tokenize(tokens, string(buffer));

		delete[] buffer;
	}


	if (!tokens.size())
	{
		InvalidBVH();
		return NULL;
	}

	int currentToken = 0;
	vector<vector<int>> rootOrderings;
	while (tokens[currentToken] != "MOTION")
	{

		if (!currentToken && tokens[currentToken].compare("HIERARCHY"))
		{
			InvalidBVH();
			return NULL;
		}

		if (!tokens[currentToken].compare("ROOT"))
		{
			vector<int> rootOrdering;
			int endToken = -1;
			SkeletonJoint* rootJoint = ParseJoint(tokens, currentToken, &endToken, &rootOrdering);

			rootOrderings.push_back(rootOrdering);

			if (rootJoint)
				skeletalMotion->m_skeletonRoots.push_back(rootJoint);

			currentToken = endToken;
		}

		currentToken++;
	}


	for (auto roots : skeletalMotion->m_skeletonRoots)
	{
		roots->PrintJoint();
	}

	if (tokens[currentToken + 1].compare("Frames:") || tokens[currentToken + 3].compare("Frame") | tokens[currentToken + 4].compare("Time:"))
		INVALID_BVH

		int numberOfFrames = atoi(tokens[currentToken + 2].c_str());

	skeletalMotion->m_frameTime = atof(tokens[currentToken + 5].c_str());

	currentToken += 6;


	for (int frame = 0; frame < numberOfFrames; frame++)
	{
		vector<btVector3> rootPositions;
		for (int rootIndex = 0; rootIndex < skeletalMotion->m_skeletonRoots.size(); rootIndex++)
		{
			SkeletonJoint* root = skeletalMotion->m_skeletonRoots[rootIndex];

			btVector3 rootPosition;
			rootPosition[rootOrderings[rootIndex][0]] = atof(tokens[currentToken + 0].c_str());
			rootPosition[rootOrderings[rootIndex][1]] = atof(tokens[currentToken + 1].c_str());
			rootPosition[rootOrderings[rootIndex][2]] = atof(tokens[currentToken + 2].c_str());

			rootPositions.push_back(rootPosition);

			currentToken += 3;

			ReadFrameRecursive(tokens, skeletalMotion, root, currentToken);
		}
		skeletalMotion->m_rootTrajectories.push_back(rootPositions);
	}
	if (currentToken != tokens.size())
		INVALID_BVH

	return skeletalMotion;
}

void ReadFrameRecursive(vector<string>& tokens, SkeletalMotion* motion, SkeletonJoint* joint, int &currentToken)
{
	if (!joint->m_childJoints.size())
		return;

	btMatrix3x3 rotation = btMatrix3x3::getIdentity();
	for (int r = 0; r < 3; r++)
		rotation *= GetRotationMatrix(joint->m_channelsOrdering[r], atof(tokens[currentToken + r].c_str()));

	btTransform jointTransform = btTransform(rotation, joint->m_localOffset);

	unordered_map<string, vector<btTransform>>::const_iterator mapIterator = motion->m_jointTransforms.find(joint->m_name);
	if (mapIterator == motion->m_jointTransforms.end())
	{
		motion->m_jointTransforms[joint->m_name] = vector<btTransform>({ jointTransform });
	}
	else
	{
		motion->m_jointTransforms[joint->m_name].push_back(jointTransform);
	}

	currentToken += 3;

	for (auto child : joint->m_childJoints)
	{
		ReadFrameRecursive(tokens, motion, child, currentToken);
	}
}

void PrintJointRecursive(SkeletonJoint* joint, int depth)
{
	string out = "";
	for (int i = 0; i < depth; i++)
		out += "_";
	cout << out + joint->m_name + "\n";

	for (auto child : joint->m_childJoints)
		PrintJointRecursive(child, depth + 1);
}

void SkeletonJoint::PrintJoint()
{
	PrintJointRecursive(this, 0);
}

void GetSkeletalSegmentsRecurse(vector<pair<btVector3, btVector3>>* vertices,
	SkeletonJoint* joint,
	btVector3& jointPositionW,
	btTransform& cumulativeTransform,
	unordered_map<string, vector<btTransform>>* jointTransforms,
	int frameIndex,
	btVector3& rootTrajectory)
{
	for (auto child : joint->m_childJoints)
	{
		btVector3 childPositionL = child->m_localOffset;
		btVector3 childPositionW = cumulativeTransform * btVector4(childPositionL[0], childPositionL[1], childPositionL[2], 1);

		childPositionW += rootTrajectory;

		vertices->push_back(pair<btVector3, btVector3>(jointPositionW, childPositionW));

		if (child->m_childJoints.size())
		{
			btTransform nextCumulativeTransform = cumulativeTransform * jointTransforms->at(child->m_name)[frameIndex];
			GetSkeletalSegmentsRecurse(vertices, child, childPositionW, nextCumulativeTransform, jointTransforms, frameIndex, rootTrajectory);
		}
	}
}

vector<pair<btVector3, btVector3>>* SkeletalMotion::GetSkeletalSegments(int skeletonIndex, int frameIndex, bool addRootTrajectory)
{
	SkeletonJoint* root = m_skeletonRoots[skeletonIndex];

	btTransform transform = m_jointTransforms[root->m_name][frameIndex];

	vector<pair<btVector3, btVector3>>* vertices = new vector<pair<btVector3, btVector3>>();

	btVector3 rootTrajOffset = addRootTrajectory ? m_rootTrajectories[frameIndex][skeletonIndex] : btVector3(0, 0, 0);

	btVector3 rootPositionW = root->m_localOffset + rootTrajOffset;

	GetSkeletalSegmentsRecurse(vertices, root, rootPositionW, transform, &m_jointTransforms, frameIndex, rootTrajOffset);

	return vertices;
}

btMatrix3x3 GetRotationMatrix(int axis, btScalar angle)
{
	angle *= M_PI / 180.0;

	if (axis == 0)
	{
		return btMatrix3x3
		(
			1, 0, 0,
			0, cos(angle), -sin(angle),
			0, sin(angle), cos(angle)
		);
	}
	else if (axis == 1)
	{
		return btMatrix3x3
		(
			cos(angle), 0, sin(angle),
			0, 1, 0,
			-sin(angle), 0, cos(angle)
		);
	}
	else if (axis == 2)
	{
		return btMatrix3x3
		(
			cos(angle), -sin(angle), 0,
			sin(angle), cos(angle), 0,
			-0, 0, 1
		);
	}
}