#include <cmath>
#include <iostream>
#include <string>
#include "wingbeat_pattern.pb.h"
#include "gtest/gtest.h";

TEST(WingbeatPatternTest, ExampleUsage) {
	proto::WingbeatPattern pattern;
	
	for (int ii = 0; ii <= 5; ++ii) {
		float wing_angle = sin((3.14f / 5.f) *  ii);
		float feather_angle = cos((3.14f / 5.f) *  ii);

		proto::JointAngleInfo* anglesElement = pattern.add_angles();
		anglesElement->set_wing(wing_angle);
		anglesElement->set_feather(feather_angle);
	}

	const std::string& patternAsString = pattern.DebugString();

	std::cout << "Here's the wingbeat pattern: " << std::endl;
	std::cout << patternAsString << std::endl;

	// Accessing an index.
	EXPECT_FLOAT_EQ(
		sin((3.14f / 5.f) *  3),
		pattern.angles(3).wing());

	EXPECT_FLOAT_EQ(
		cos((3.14f / 5.f) *  5),
		pattern.angles(5).feather());

	// Getting the length of the pattern.
	EXPECT_EQ(6, pattern.angles().size());

	// We can parse from string to protobuf data structure.
	std::string serializedElem = pattern.angles(3).SerializeAsString();
	proto::JointAngleInfo parsedInfo;
	parsedInfo.ParseFromString(serializedElem);

	std::string expectedDebugString = pattern.angles(3).DebugString();
	std::string actualDebugString = parsedInfo.DebugString();
	EXPECT_STREQ(expectedDebugString.c_str(), actualDebugString.c_str());

}

int main(int argc, wchar_t** argv) {
	::testing::InitGoogleTest(&argc, argv);
	RUN_ALL_TESTS();
	system("PAUSE");
	return 0;
}