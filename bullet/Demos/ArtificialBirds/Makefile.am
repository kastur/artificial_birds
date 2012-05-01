noinst_PROGRAMS=ArtificialBirds

ArtificialBirds_SOURCES=ArtificialBirds.cpp ArtificialBirds.h main.cpp
ArtificialBirds_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
ArtificialBirds_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
