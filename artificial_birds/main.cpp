#include "ArtificialBirds.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer gDebugDrawer;

int main(int argc, char** argv) {
	ArtificialBirdsDemoApp* artificialBirdsApp = new ArtificialBirdsDemoApp();
	artificialBirdsApp->initPhysics();
	artificialBirdsApp->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
	artificialBirdsApp->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
	glutmain(argc, argv, 800, 600,
		"Artificial Birds (http://github.com/kastur/artificial_birds)",
		artificialBirdsApp);
	
	delete artificialBirdsApp;
	return 0;
}
