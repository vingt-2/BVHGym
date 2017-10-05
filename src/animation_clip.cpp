#include <fstream>
#include <iostream>
#include <string.h>
#include "animation_clip.h"

#define _HAS_ITERATOR_DEBUGGING 0

using namespace std;

void InvalidBVH()
{
	cout << "There were invalid values encountered in your BVH file.\n";
}

#define INVALID_BVH {InvalidBVH(); return NULL;}

void ReadFrameRecursive(vector<string>& tokens, SkeletalMotion* motion, SkeletonJoint* joint, int currentToken);
void PrintJointRecursive(SkeletonJoint* joint, int depth);

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
		else if (!tokens[currentToken].compare("End") && !tokens[currentToken+1].compare("Site"))
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

SkeletalMotion* BVHImport(string bvhFilePath)
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
	
	currentToken += 5;


	for (int frame = 0; frame < numberOfFrames; frame++)
	{
		for (int rootIndex = 0; rootIndex < skeletalMotion->m_skeletonRoots.size(); rootIndex++)
		{
			SkeletonJoint* root = skeletalMotion->m_skeletonRoots[rootIndex];
			
			btVector3 rootPosition;
			rootPosition[rootOrderings[rootIndex][0]] = atof(tokens[currentToken+0].c_str());
			rootPosition[rootOrderings[rootIndex][1]] = atof(tokens[currentToken+1].c_str());
			rootPosition[rootOrderings[rootIndex][2]] = atof(tokens[currentToken+2].c_str());

			currentToken += 3;

			ReadFrameRecursive(tokens, skeletalMotion, root, currentToken);
		}
	}

	return skeletalMotion;
}

btTransform GetTransformFromEuler(btVector3 &eulerAngles, btVector3 &translation)
{

	float phi = eulerAngles[0];
	float theta = eulerAngles[1];
	float psy = eulerAngles[2];

	btMatrix3x3 rotation
	(
		cos(theta)*cos(psy), -cos(theta)*sin(psy), sin(theta),
		cos(phi)*sin(psy) + sin(phi)*sin(theta)*cos(psy), cos(phi)*cos(psy) - sin(phi)*sin(theta)*sin(psy), -sin(phi)*cos(theta),
		sin(phi)*sin(psy) - cos(phi)*sin(theta)*cos(psy), sin(phi)*cos(psy) + cos(phi)*sin(theta)*sin(psy), cos(phi)*cos(theta)
	);

	return btTransform(rotation, translation);
}

void ReadFrameRecursive(vector<string>& tokens, SkeletalMotion* motion, SkeletonJoint* joint, int currentToken)
{
	btVector3 eulerAngles;
	eulerAngles[joint->m_channelsOrdering[0]] = atof(tokens[currentToken + 0].c_str());
	eulerAngles[joint->m_channelsOrdering[1]] = atof(tokens[currentToken + 1].c_str());
	eulerAngles[joint->m_channelsOrdering[2]] = atof(tokens[currentToken + 2].c_str());

	btTransform jointTransform = GetTransformFromEuler(eulerAngles, joint->m_localOffset);

	unordered_map<string, vector<btTransform>>::const_iterator mapIterator = motion->m_jointTransforms.find(joint->m_name);
	if (mapIterator == motion->m_jointTransforms.end())
	{
		motion->m_jointTransforms[joint->m_name] = vector<btTransform>({ jointTransform });
	}
	else
	{
		motion->m_jointTransforms[joint->m_name].push_back(jointTransform);
	}

	for (auto child : joint->m_childJoints)
	{
		ReadFrameRecursive(tokens, motion, child, currentToken + 3);
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