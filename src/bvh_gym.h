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


///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include <stdio.h>
#include <iostream>
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces\CommonMultiBodyBase.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

#include "articulated_ragdoll.h"

class CommonExampleInterface*    BVHGymCreateFunc(struct CommonExampleOptions& options);

struct BVHGym : public CommonMultiBodyBase
{
	BVHGym(struct GUIHelperInterface* helper);

	virtual ~BVHGym();
	virtual void initPhysics();
	virtual void renderScene();
	virtual void processCommandLineArgs(int argc, char** argv);

	void ResetCamera();

	bool SetBVHAnimation(SkeletalMotion* motion)
	{
		if (motion == NULL)
		{
			return false;
		}
		m_animationPlayer->SetSkeletalMotion(motion);
		m_skeletalMotion = motion;
		return true;
	}

	bool keyboardCallback(int keycode, int state)
	{
		if (keycode == 65295)
			m_keycode = 1;
		else if (keycode == 65296)
			m_keycode = 2;
		else if (keycode == 65309)
			m_keycode = 0;
		else
			m_keycode = -1;

		if (m_animationPlayer)
			m_animationPlayer->InputKeyPressed(m_keycode);

		return true;
	}

	int GetKeyPressed() { return m_keycode; }

	void ClearKeyPressed() { m_keycode = -1; }

	bool IsTerminating() { return m_bShouldTerminate; }

private:
	SkeletalAnimationPlayer* m_animationPlayer;
	SkeletalMotion* m_skeletalMotion;
	ArticulatedRagdoll* m_articulatedRagdoll;

	std::vector<btVector3> m_comPositions;

	bool m_bDrawSkeleton;
	bool m_bDrawCOMState;

	int m_keycode = -1;

	bool m_bShouldTerminate;
	
private:
	btClock m_clock;
};