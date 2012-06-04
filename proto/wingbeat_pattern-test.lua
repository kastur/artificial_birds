project "ProtoTests"
kind "ConsoleApp"
language "C++"

links {
	"libprotobuf",
	"gtest"
}

configuration "Debug"
libdirs {
	"../protobuf_win/Debug",
	"../gtest_win/Debug"
}

configuration "Release"
libdirs {
	"../protobuf_win/Release",
	"../gtest_win/Release"
}

configuration {}

includedirs {
	"../protobuf_win/src",
	"../gtest_win/include"
}

files { "wingbeat_pattern-test.cpp", "wingbeat_pattern.pb.cc", "wingbeat_pattern.pb.h" }
