/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <stdio.h>
#include <iostream>
#include <vector>

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "Utils/b3Clock.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

#include "bvh_gym.h"

CommonExampleInterface*    BVHGym;
int gSharedMemoryKey = -1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = BVHGym->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = BVHGym->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

b3KeyboardCallback prevkeyboardButtonCallback = 0;
static void OnKeyboardDown(int keycode, int state) {
	bool handled = false;

	handled = BVHGym->keyboardCallback(keycode, state);
	if (!handled)
	{
		if (prevkeyboardButtonCallback)
			prevkeyboardButtonCallback(keycode, state);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};
int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Articulated Ragdoll", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, true);


	CommonExampleOptions options(&gui);


	BVHGym = BVHGymCreateFunc(options);
	BVHGym->processCommandLineArgs(argc, argv);

	BVHGym->initPhysics();
	BVHGym->resetCamera();

	app->m_window->setKeyboardCallback((b3KeyboardCallback)OnKeyboardDown);

	b3Clock clock;
	float time = 0;
	do
	{
		time += clock.getTimeInSeconds();
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		if (dtSec < 0.1)
			dtSec = 0.1;

		BVHGym->stepSimulation(dtSec);
		clock.reset();

		BVHGym->renderScene();

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	BVHGym->exitPhysics();
	delete BVHGym;
	delete app;
	return 0;
}

