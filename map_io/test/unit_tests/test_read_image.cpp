#include <gtest/gtest.h>
#include <map_io/image_loader.h>
#include <stdexcept>

const unsigned int g_valid_image_width = 10;
const unsigned int g_valid_image_height = 10;
// Note that the image content is given in row-major order, with the
// lower-left pixel first.  This is different from a graphics coordinate
// system, which starts with the upper-left pixel.  The loadMapFromFile
// call converts from the latter to the former when it loads the image, and
// we want to compare against the result of that conversion.
const char g_valid_image_content[] = {
    0,   0,   0,   0,   0, 0,   0,   0,   0, 0, 100, 100, 100, 100, 0, 0,   100, 100, 100, 0, 100, 100, 100, 100, 0,
    0,   100, 100, 100, 0, 100, 0,   0,   0, 0, 0,   0,   0,   0,   0, 100, 0,   0,   0,   0, 0,   0,   0,   0,   0,
    100, 0,   0,   0,   0, 0,   100, 100, 0, 0, 100, 0,   0,   0,   0, 0,   100, 100, 0,   0, 100, 0,   0,   0,   0,
    0,   100, 100, 0,   0, 100, 0,   0,   0, 0, 0,   100, 100, 0,   0, 100, 0,   0,   0,   0, 0,   0,   0,   0,   0,
};

const char* g_valid_png_file = "testmap.png";
const char* g_valid_bmp_file = "testmap.bmp";

const double g_valid_image_res = 0.1;

TEST(TestImageLoader, loadValidPNG)
{

    const double origin[3] = {0.0, 0.0, 0.0};
    const nav_msgs::OccupancyGrid map =
        map_io::loadMapFromFile(g_valid_png_file, g_valid_image_res, false, 0.65, 0.1, origin[0], origin[1], origin[2]);
    EXPECT_FLOAT_EQ(map.info.resolution, g_valid_image_res);
    EXPECT_EQ(map.info.width, g_valid_image_width);
    EXPECT_EQ(map.info.height, g_valid_image_height);
    for (unsigned int i = 0; i < map.info.width * map.info.height; i++)
        EXPECT_EQ(g_valid_image_content[i], map.data[i]);
}

TEST(TestImageLoader, loadValidBMP)
{
    const double origin[3] = {0.0, 0.0, 0.0};
    const nav_msgs::OccupancyGrid map =
        map_io::loadMapFromFile(g_valid_bmp_file, g_valid_image_res, false, 0.65, 0.1, origin[0], origin[1], origin[2]);

    EXPECT_FLOAT_EQ(map.info.resolution, g_valid_image_res);
    EXPECT_EQ(map.info.width, g_valid_image_width);
    EXPECT_EQ(map.info.height, g_valid_image_height);
    for (unsigned int i = 0; i < map.info.width * map.info.height; i++)
        EXPECT_EQ(g_valid_image_content[i], map.data[i]);
}

TEST(TestImageLoader, loadInvalidFile)
{
    try
    {
        const double origin[3] = {0.0, 0.0, 0.0};
        const nav_msgs::OccupancyGrid map =
            map_io::loadMapFromFile("foo", 0.1, false, 0.65, 0.1, origin[0], origin[1], origin[2]);
    }
    catch (const std::runtime_error&)
    {
        SUCCEED();
        return;
    }
    catch (...)
    {
        FAIL() << "Uncaught exception";
    }
    ADD_FAILURE() << "Didn't throw exception as expected";
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
