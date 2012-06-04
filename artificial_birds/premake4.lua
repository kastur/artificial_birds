project "ArtificialBirds"
kind "ConsoleApp"

libdirs {
	"../bullet/Glut",
}

links {
	"LinearMath",
	"BulletCollision",
	"BulletDynamics", 
	"OpenGLSupport",
	"opengl32",
}

configuration "x64"
links {
	"glut64",
	"glew64s"
}

configuration "x32"
links {
	"glut32",
	"glew32s"
}

configuration {}

includedirs {
	"../bullet/src",
	"../bullet/Glut",
	"../bullet/Demos/OpenGL"
}

files { "**.cpp", "**.h" }
