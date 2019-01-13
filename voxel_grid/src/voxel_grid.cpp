#include <ros/console.h>
#include <sys/time.h>
#include <voxel_grid/voxel_grid.h>

namespace voxel_grid
{
VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z) : costmap_(nullptr)
{
    size_x_ = size_x;
    size_y_ = size_y;
    size_z_ = size_z;

    if (size_z_ > 16)
    {
        ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z_);
        size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0) >> 16;
    uint32_t* col = data_;
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i)
    {
        *col = unknown_col;
        ++col;
    }
}

// cppcheck-suppress unusedFunction
void VoxelGrid::resize(unsigned int size_x, unsigned int size_y, unsigned int size_z)
{
    // if we're not actually changing the size, we can just reset things
    if (size_x == size_x_ && size_y == size_y_ && size_z == size_z_)
    {
        reset();
        return;
    }

    delete[] data_;
    size_x_ = size_x;
    size_y_ = size_y;
    size_z_ = size_z;

    if (size_z_ > 16)
    {
        ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z);
        size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0) >> 16;
    uint32_t* col = data_;
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i)
    {
        *col = unknown_col;
        ++col;
    }
}

VoxelGrid::~VoxelGrid()
{
    delete[] data_;
}

void VoxelGrid::reset()
{
    uint32_t unknown_col = ~((uint32_t)0) >> 16;
    uint32_t* col = data_;
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i)
    {
        *col = unknown_col;
        ++col;
    }
}

void VoxelGrid::markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length)
{
    if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ || z1 >= size_z_)
    {
        ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)",
                  x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
        return;
    }

    MarkVoxel mv(data_);
    raytraceLine(mv, x0, y0, z0, x1, y1, z1, max_length);
}

void VoxelGrid::clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1,
                               unsigned int max_length)
{
    if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ || z1 >= size_z_)
    {
        ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)",
                  x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
        return;
    }

    ClearVoxel cv(data_);
    raytraceLine(cv, x0, y0, z0, x1, y1, z1, max_length);
}

// cppcheck-suppress unusedFunction
void VoxelGrid::clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1,
                                    unsigned char* map_2d, unsigned int unknown_threshold, unsigned int mark_threshold,
                                    unsigned char free_cost, unsigned char unknown_cost, unsigned int max_length)
{
    costmap_ = map_2d;
    if (map_2d == nullptr)
    {
        clearVoxelLine(x0, y0, z0, x1, y1, z1, max_length);
        return;
    }

    if (x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1 >= size_x_ || y1 >= size_y_ || z1 >= size_z_)
    {
        ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)",
                  x0, y0, z0, x1, y1, z1, size_x_, size_y_, size_z_);
        return;
    }

    ClearVoxelInMap cvm(data_, costmap_, unknown_threshold, mark_threshold, free_cost, unknown_cost);
    raytraceLine(cvm, x0, y0, z0, x1, y1, z1, max_length);
}

VoxelStatus VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
{
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
        ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
        return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1 << z << 16) | (1 << z);
    uint32_t result = data_[y * size_x_ + x] & full_mask;
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if (bits < 2)
    {
        if (bits < 1)
            return FREE;

        return UNKNOWN;
    }

    return MARKED;
}

VoxelStatus VoxelGrid::getVoxelColumn(unsigned int x, unsigned int y, unsigned int unknown_threshold,
                                      unsigned int marked_threshold)
{
    if (x >= size_x_ || y >= size_y_)
    {
        ROS_DEBUG("Error, voxel out of bounds. (%d, %d)\n", x, y);
        return UNKNOWN;
    }

    uint32_t* col = &data_[y * size_x_ + x];

    unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
    unsigned int marked_bits = *col >> 16;

    // check if the number of marked bits qualifies the col as marked
    if (!bitsBelowThreshold(marked_bits, marked_threshold))
    {
        return MARKED;
    }

    // check if the number of unkown bits qualifies the col as unknown
    if (!bitsBelowThreshold(unknown_bits, unknown_threshold))
        return UNKNOWN;

    return FREE;
}

unsigned int VoxelGrid::sizeX()
{
    return size_x_;
}

unsigned int VoxelGrid::sizeY()
{
    return size_y_;
}

unsigned int VoxelGrid::sizeZ()
{
    return size_z_;
}

void VoxelGrid::printVoxelGrid()
{
    for (unsigned int z = 0; z < size_z_; z++)
    {
        printf("Layer z = %u:\n", z);
        for (unsigned int y = 0; y < size_y_; y++)
        {
            for (unsigned int x = 0; x < size_x_; x++)
            {
                printf((getVoxel(x, y, z)) == voxel_grid::MARKED ? "#" : " ");
            }
            printf("|\n");
        }
    }
}

void VoxelGrid::printColumnGrid()
{
    printf("Column view:\n");
    for (unsigned int y = 0; y < size_y_; y++)
    {
        for (unsigned int x = 0; x < size_x_; x++)
        {
            printf((getVoxelColumn(x, y, 16, 0) == voxel_grid::MARKED) ? "#" : " ");
        }
        printf("|\n");
    }
}
}
