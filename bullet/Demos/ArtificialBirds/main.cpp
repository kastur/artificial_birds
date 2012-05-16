#include "ArtificialBirds.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer gDebugDrawer;

int main(int argc, char** argv) {
	ArtificialBirds* artificialBirds = new ArtificialBirds();
	artificialBirds->initPhysics();
	artificialBirds->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
	artificialBirds->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
	glutmain(argc, argv, 1024, 768,
		"Artificial Birds (http://github.com/kastur/artificial_birds)",
		artificialBirds);
	
	delete artificialBirds;
	return 0;
}
