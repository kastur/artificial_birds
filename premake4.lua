solution "ArtificalBirds"

configurations {"Release", "Debug"}

configuration "Release"
	flags { "Optimize", "StaticRuntime", "NoMinimalRebuild", "NoRTTI", "NoExceptions", "FloatFast"}
	
configuration "Debug"
	flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoRTTI", "NoExceptions", "NoEditAndContinue" ,"FloatFast"}
		
platforms {"x32", "x64"}

configuration "x64"		
	targetsuffix "_64"
configuration {"x64", "debug"}
	targetsuffix "_x64_debug"
configuration {"x64", "release"}
	targetsuffix "_x64"
configuration {"x32", "debug"}
	targetsuffix "_debug"

configuration {"Windows"}
defines { "_CRT_SECURE_NO_WARNINGS","_CRT_SECURE_NO_DEPRECATE"}

targetdir "bin"

-- Disable exception handling on MSVC 2008 and higher. MSVC 2005 without service pack has some linker issue (ConvexDecompositionDemo uses STL through HACD library)	
defines { "_HAS_EXCEPTIONS=0" }


-- Multithreaded compiling
buildoptions { "/MP"  }

language "C++"
	
location("./vs2010")
include "artificial_birds"
include "bullet/src/LinearMath"	
include "bullet/src/BulletCollision"	
include "bullet/src/BulletDynamics"	
include "bullet/src/BulletSoftBody"
include "bullet/Demos/OpenGL"	


-- Test cases
dofile "proto/wingbeat_pattern-test.lua"