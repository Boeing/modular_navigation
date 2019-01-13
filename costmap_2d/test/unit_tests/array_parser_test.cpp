#include <gtest/gtest.h>

#include "costmap_2d/array_parser.h"

using namespace costmap_2d;

TEST(array_parser, basic_operation)
{
    std::string error;
    std::vector<std::vector<float>> vvf;
    vvf = parseVVF("[[1, 2.2], [.3, -4e4]]", error);
    EXPECT_EQ(2, vvf.size());
    EXPECT_EQ(2, vvf[0].size());
    EXPECT_EQ(2, vvf[1].size());
    EXPECT_EQ(1.0f, vvf[0][0]);
    EXPECT_EQ(2.2f, vvf[0][1]);
    EXPECT_EQ(0.3f, vvf[1][0]);
    EXPECT_EQ(-40000.0f, vvf[1][1]);
    EXPECT_EQ("", error);
}

TEST(array_parser, missing_open)
{
    std::string error;
    parseVVF("[1, 2.2], [.3, -4e4]]", error);
    EXPECT_FALSE(error == "");
}

TEST(array_parser, missing_close)
{
    std::string error;
    parseVVF("[[1, 2.2], [.3, -4e4]", error);
    EXPECT_FALSE(error == "");
}

TEST(array_parser, wrong_depth)
{
    std::string error;
    parseVVF("[1, 2.2], [.3, -4e4]", error);
    EXPECT_FALSE(error == "");
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
