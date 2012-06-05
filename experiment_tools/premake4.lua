project "ExperimentTools"
kind "ConsoleApp"

libdirs {
	"../dslin",
}

links {
	"libprotobuf",
	"discpp"
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
	"../dslin",
	"../proto/",
	"../protobuf_win/src"
}

files { "**.cpp", "**.h", "../proto/**.cc", "../proto/**.h" }

