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
*/

#include <fstream>
#include <iostream>
#include <string.h>
#include "animation.h"

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

// Helper that gets Which rotation matrix we should output for each axis
btMatrix3x3 GetRotationMatrix(int axis, btScalar angle);
// Helper that turns a bunch of string parameters from a bvh to int values
int ChannelOrderToInt(string str);

// Turns a big string file into a big bunch of tokens (splits with special characters)
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

// See BVHImport for explanation
SkeletonJoint* ParseJoint(vector<string> &tokens, int startToken, int* endToken, unordered_map<string, vector<int>> &jointChannelsOrderings)
{
	if (tokens[startToken + 2].compare("{"))
	{
		INVALID_BVH
	}

	if (tokens[startToken + 3].compare("OFFSET"))
	{
		INVALID_BVH
	}

	string jointName = tokens[startToken + 1];
	btVector3 jointLocalOffset = btVector3
	(
		btScalar(atof(tokens[startToken + 4].c_str())),
		btScalar(atof(tokens[startToken + 5].c_str())),
		btScalar(atof(tokens[startToken + 6].c_str()))
	);

	vector<SkeletonJoint*> jointChildren;

	if (tokens[startToken + 7].compare("CHANNELS"))
	{
		INVALID_BVH
	}

	int currentToken;
	if (!tokens[startToken + 8].compare("3"))
	{
		currentToken = startToken + 12;
		
		jointChannelsOrderings[jointName] = vector<int>({ ChannelOrderToInt(tokens[startToken + 9]),
			ChannelOrderToInt(tokens[startToken + 10]),
			ChannelOrderToInt(tokens[startToken + 11]) });

	}
	else if (!tokens[startToken + 8].compare("6"))
	{
		currentToken = startToken + 15;

		jointChannelsOrderings[jointName] = vector<int>({ ChannelOrderToInt(tokens[startToken + 9]),
			ChannelOrderToInt(tokens[startToken + 10]),
			ChannelOrderToInt(tokens[startToken + 11]),
			ChannelOrderToInt(tokens[startToken + 12]),
			ChannelOrderToInt(tokens[startToken + 13]),
			ChannelOrderToInt(tokens[startToken + 14])});
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
			SkeletonJoint* childJoint = ParseJoint(tokens, currentToken, &currentEndToken, jointChannelsOrderings);
			if (childJoint)
			{
				jointChildren.push_back(childJoint);
				currentToken = currentEndToken + 1;
			}
		}
		else if (!tokens[currentToken].compare("End") && !tokens[currentToken + 1].compare("Site"))
		{
			if (!tokens[currentToken + 2].compare("{") && !tokens[currentToken + 3].compare("OFFSET") && !tokens[currentToken + 7].compare("}"))
			{
				string endJointName = jointName + "_end";
				btVector3 endJointLocalOffset = btVector3
				(
					btScalar(atof(tokens[startToken + 4].c_str())),
					btScalar(atof(tokens[startToken + 5].c_str())),
					btScalar(atof(tokens[startToken + 6].c_str()))
				);
				
				SkeletonJoint* endJoint = new SkeletonJoint(endJointName, vector<SkeletonJoint*>(), endJointLocalOffset);

				jointChildren.push_back(endJoint);
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

	return new SkeletonJoint(jointName,jointChildren,jointLocalOffset);
}

// See BVHImport for explanation
void ReadFrameRecursive(vector<string>& tokens,
	unordered_map<string, vector<btTransform>>& jointTransforms,
	SkeletonJoint* joint,
	int &currentToken,
	unordered_map<string, vector<int>> &jointsChannelOrderings,
	bool bIsRoot)
{
	if (!joint->GetDirectChildren().size())
		return;

	btMatrix3x3 rotation = btMatrix3x3::getIdentity();

	for (int r = 0; r < 3; r++)
		rotation *= GetRotationMatrix(jointsChannelOrderings[joint->GetName()][bIsRoot ? r + 3 : r], atof(tokens[currentToken + r].c_str()));

	btTransform jointTransform = btTransform(rotation, joint->GetLocalOffset());

	unordered_map<string, vector<btTransform>>::const_iterator mapIterator = jointTransforms.find(joint->GetName());
	if (mapIterator == jointTransforms.end())
	{
		jointTransforms[joint->GetName()] = vector<btTransform>({ jointTransform });
	}
	else
	{
		jointTransforms[joint->GetName()].push_back(jointTransform);
	}

	currentToken += 3;

	for (auto child : joint->GetDirectChildren())
	{
		ReadFrameRecursive(tokens, jointTransforms, child, currentToken, jointsChannelOrderings, false);
	}
}

/*
	BVH Imports:

	1) Opens the file.
	2) Calls a recursive joint parser to parse the tree structure
	3) for each frame, calls a recursive frame data reader that recusrively populates 
		the tree's joints data and advances in the frames data block.
	4) Profit. Returns a null pointer if there were any issue parsing the data.
*/
SkeletalMotion* SkeletalMotion::BVHImport(string bvhFilePath)
{
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

			std::cout << "Could not open " << bvhFilePath << "\n";
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
	
	vector<SkeletonJoint*>		skeletalRoots;
	vector<vector<btVector3>>	rootTrajectories;
	unordered_map<string, vector<btTransform>>	jointTransforms;

	unordered_map<string, vector<int>>			jointChannelsOrderings;
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
			SkeletonJoint* rootJoint = ParseJoint(tokens, currentToken, &endToken, jointChannelsOrderings);

			if (rootJoint)
				skeletalRoots.push_back(rootJoint);

			currentToken = endToken;
		}

		currentToken++;
	}

	for (auto roots : skeletalRoots)
	{
		roots->PrintJoint();
	}

	if (tokens[currentToken + 1].compare("Frames:") || tokens[currentToken + 3].compare("Frame") | tokens[currentToken + 4].compare("Time:"))
		INVALID_BVH

	int frameCount = atoi(tokens[currentToken + 2].c_str());
	float frameTime = atof(tokens[currentToken + 5].c_str());

	currentToken += 6;

	for (int frame = 0; frame < frameCount; frame++)
	{
		vector<btVector3> rootPositions;
		for (int rootIndex = 0; rootIndex < skeletalRoots.size(); rootIndex++)
		{
			SkeletonJoint* root = skeletalRoots[rootIndex];

			btVector3 rootPosition;
			rootPosition[jointChannelsOrderings[root->GetName()][0]] = atof(tokens[currentToken + 0].c_str());
			rootPosition[jointChannelsOrderings[root->GetName()][1]] = atof(tokens[currentToken + 1].c_str());
			rootPosition[jointChannelsOrderings[root->GetName()][2]] = atof(tokens[currentToken + 2].c_str());

			rootPositions.push_back(rootPosition);

			currentToken += 3;

			ReadFrameRecursive(tokens, jointTransforms, root, currentToken, jointChannelsOrderings, true);
		}
		rootTrajectories.push_back(rootPositions);
	}
	if (currentToken != tokens.size())
		INVALID_BVH

	SkeletalMotion* result = new SkeletalMotion(bvhFilePath, rootTrajectories, jointTransforms, skeletalRoots, 1.0 / frameTime, frameCount);
	
	/*if (bNormalizedOffsets)
	{
		for (int i = 0; i < result->m_skeletonRoots.size(); i++)
		{
			unordered_map<string, SkeletonJoint*> joints;
			result->m_skeletonRoots[i]->QuerySkeleton(&joints,NULL);
			float maxOffsetLen = 0;
			for (auto joint : joints)
			{
				btScalar length = joint.second->GetLocalOffset().length();
				if (maxOffsetLen < length)
					maxOffsetLen = length;
			}
			for (auto joint : joints)
			{
				joint.second->ApplyOffsetNormalization(maxOffsetLen);
			}

		}
	}*/

	return result;
}

// Gets Which rotation matrix we should output for each axis
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
	else
	{
		std::cout << "Warning: Invalid rotation axis provided in bvh file...\n";
		return btMatrix3x3::getIdentity();
	}
}

// Helper that turns a bunch of string parameters from a bvh to int values
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