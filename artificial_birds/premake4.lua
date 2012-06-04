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
	"libprotobuf"
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

configuration "Debug"
libdirs {
	"../protobuf_win/Debug"
}

configuration "Release"
libdirs {
	"../protobuf_win/Release"
}

configuration {}

includedirs {
	"../bullet/src",
	"../bullet/Glut",
	"../bullet/Demos/OpenGL",
	"../proto/",
	"../protobuf_win/src"
}

files { "**.cpp", "**.h", "../proto/**.cc", "../proto/**.h" }

