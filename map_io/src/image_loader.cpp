#include <cstring>
#include <stdexcept>

#include <stdio.h>
#include <stdlib.h>

#include <SDL/SDL_image.h>
#include <tf2/LinearMath/Quaternion.h>

#include <map_io/image_loader.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace map_io
{

nav_msgs::OccupancyGrid loadMapFromFile(const std::string& file_name, const double res, const bool negate,
                                        const double occ_th, const double free_th, const double origin_x,
                                        const double origin_y, const double origin_yaw, const MapMode mode)
{
    SDL_Surface* img;

    nav_msgs::OccupancyGrid map;

    unsigned char* pixels;
    unsigned char* p;
    unsigned char value;
    int rowstride, n_channels, avg_channels;
    unsigned int i, j;
    int k;
    double occ;
    int alpha;
    int color_sum;
    double color_avg;

    // Load the image using SDL.  If we get NULL back, the image load failed.
    if (!(img = IMG_Load(file_name.c_str())))
    {
        std::string errmsg =
            std::string("failed to open image file \"") + file_name + std::string("\": ") + IMG_GetError();
        throw std::runtime_error(errmsg);
    }

    // Copy the image data into the map structure
    map.info.width = img->w;
    map.info.height = img->h;
    map.info.resolution = res;
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.position.z = 0.0;
    tf2::Quaternion qt;
    qt.setRPY(0, 0, origin_yaw);
    map.info.origin.orientation.x = qt.x();
    map.info.origin.orientation.y = qt.y();
    map.info.origin.orientation.z = qt.z();
    map.info.origin.orientation.w = qt.w();

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height);

    // Get values that we'll need to iterate through the pixels
    rowstride = img->pitch;
    n_channels = img->format->BytesPerPixel;

    // NOTE: Trinary mode still overrides here to preserve existing behavior.
    // Alpha will be averaged in with color channels when using trinary mode.
    if (mode == TRINARY || !img->format->Amask)
        avg_channels = n_channels;
    else
        avg_channels = n_channels - 1;

    // Copy pixel data into the map structure
    pixels = static_cast<unsigned char*>(img->pixels);
    for (j = 0; j < map.info.height; j++)
    {
        for (i = 0; i < map.info.width; i++)
        {
            // Compute mean of RGB for this pixel
            p = pixels + j * rowstride + i * n_channels;
            color_sum = 0;
            for (k = 0; k < avg_channels; k++)
                color_sum += *(p + (k));
            color_avg = color_sum / (double)avg_channels;

            if (n_channels == 1)
                alpha = 1;
            else
                alpha = *(p + n_channels - 1);

            if (negate)
                color_avg = 255 - color_avg;

            if (mode == RAW)
            {
                value = color_avg;
                map.data[MAP_IDX(map.info.width, i, map.info.height - j - 1)] = value;
                continue;
            }

            // If negate is true, we consider blacker pixels free, and whiter
            // pixels free.  Otherwise, it's vice versa.
            occ = (255 - color_avg) / 255.0;

            // Apply thresholds to RGB means to determine occupancy values for
            // map.  Note that we invert the graphics-ordering of the pixels to
            // produce a map with cell (0,0) in the lower-left corner.
            if (occ > occ_th)
                value = +100;
            else if (occ < free_th)
                value = 0;
            else if (mode == TRINARY || alpha < 1.0)
                value = -1;
            else
            {
                double ratio = (occ - free_th) / (occ_th - free_th);
                value = 99 * ratio;
            }

            map.data[MAP_IDX(map.info.width, i, map.info.height - j - 1)] = value;
        }
    }

    SDL_FreeSurface(img);

    return map;
}
}
