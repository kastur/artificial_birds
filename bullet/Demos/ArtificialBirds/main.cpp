#include "ArtificialBirds.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer gDebugDrawer;

int main(int argc, char** argv) {
	ArtificialBirds* artificialBirds = new ArtificialBirds();
	artificialBirds->initPhysics();
	artificialBirds->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
	glutmain(argc, argv, 1024, 768,
		"Artificial Birds (http://github.com/kasturi/artificial_birds)",
		artificialBirds);
	delete artificialBirds;
	return 0;
}
