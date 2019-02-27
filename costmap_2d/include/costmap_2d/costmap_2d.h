#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <boost/thread.hpp>
#include <geometry_msgs/Point.h>
#include <queue>
#include <vector>

namespace costmap_2d
{

struct MapLocation
{
    unsigned int x;
    unsigned int y;
};

class Costmap2D
{
    friend class CostmapTester;  // Need this for gtest to work correctly
  public:
    Costmap2D(const unsigned int cells_size_x, const unsigned int cells_size_y, const double resolution,
              const double origin_x, const double origin_y, const unsigned char default_value = 0);

    Costmap2D(const Costmap2D& map);

    Costmap2D& operator=(const Costmap2D& map);

    bool copyCostmapWindow(const Costmap2D& map, const double win_origin_x, const double win_origin_y,
                           const double win_size_x, const double win_size_y);

    Costmap2D();
    virtual ~Costmap2D();

    unsigned char getCost(const unsigned int mx, const unsigned int my) const;

    void setCost(const unsigned int mx, const unsigned int my, const unsigned char cost);

    void mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy) const;

    bool worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my) const;

    void worldToMapNoBounds(const double wx, const double wy, int& mx, int& my) const;

    void worldToMapEnforceBounds(const double wx, const double wy, int& mx, int& my) const;

    inline unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }

    inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
        my = index / size_x_;
        mx = index - (my * size_x_);
    }

    unsigned char* getCharMap() const;

    unsigned int getSizeInCellsX() const;

    unsigned int getSizeInCellsY() const;

    double getSizeInMetersX() const;

    double getSizeInMetersY() const;

    double getOriginX() const;

    double getOriginY() const;

    double getResolution() const;

    void setDefaultValue(unsigned char c)
    {
        default_value_ = c;
    }

    unsigned char getDefaultValue()
    {
        return default_value_;
    }

    /**
     * @brief  Sets the cost of a convex polygon to a desired value
     * @param polygon The polygon to perform the operation on
     * @param cost_value The value to set costs to
     * @return True if the polygon was filled... false if it could not be filled
     */
    bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, const unsigned char cost_value);

    /**
     * @brief  Get the map cells that make up the outline of a polygon
     * @param polygon The polygon in map coordinates to rasterize
     * @param polygon_cells Will be set to the cells contained in the outline of the polygon
     */
    void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

    /**
     * @brief  Get the map cells that fill a convex polygon
     * @param polygon The polygon in map coordinates to rasterize
     * @param polygon_cells Will be set to the cells that fill the polygon
     */
    void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

    /**
     * @brief  Move the origin of the costmap to a new location.... keeping data when it can
     * @param  new_origin_x The x coordinate of the new origin
     * @param  new_origin_y The y coordinate of the new origin
     */
    virtual void updateOrigin(const double new_origin_x, const double new_origin_y);

    /**
     * @brief  Save the costmap out to a pgm file
     * @param file_name The name of the file to save
     */
    bool saveMap(std::string file_name);

    void resizeMap(const unsigned int size_x, const unsigned int size_y, const double resolution, const double origin_x,
                   const double origin_y);

    void resetMap(const unsigned int x0, const unsigned int y0, const unsigned int xn, const unsigned int yn);

    unsigned int cellDistance(double world_dist);

    // Provide a typedef to ease future code maintenance
    typedef boost::recursive_mutex mutex_t;
    mutex_t* getMutex()
    {
        return access_;
    }

  protected:
    /**
     * @brief  Copy a region of a source map into a destination map
     * @param  source_map The source map
     * @param sm_lower_left_x The lower left x point of the source map to start the copy
     * @param sm_lower_left_y The lower left y point of the source map to start the copy
     * @param sm_size_x The x size of the source map
     * @param  dest_map The destination map
     * @param dm_lower_left_x The lower left x point of the destination map to start the copy
     * @param dm_lower_left_y The lower left y point of the destination map to start the copy
     * @param dm_size_x The x size of the destination map
     * @param region_size_x The x size of the region to copy
     * @param region_size_y The y size of the region to copy
     */
    template <typename data_type>
    void copyMapRegion(data_type* source_map, const unsigned int sm_lower_left_x, const unsigned int sm_lower_left_y,
                       const unsigned int sm_size_x, data_type* dest_map, const unsigned int dm_lower_left_x,
                       const unsigned int dm_lower_left_y, const unsigned int dm_size_x,
                       const unsigned int region_size_x, const unsigned int region_size_y)
    {
        // we'll first need to compute the starting points for each map
        data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
        data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

        // now, we'll copy the source map into the destination map
        for (unsigned int i = 0; i < region_size_y; ++i)
        {
            memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
            sm_index += sm_size_x;
            dm_index += dm_size_x;
        }
    }

    /**
     * @brief  Deletes the costmap, static_map, and markers data structures
     */
    virtual void deleteMaps();

    /**
     * @brief  Resets the costmap and static_map to be unknown space
     */
    virtual void resetMaps();

    /**
     * @brief  Initializes the costmap, static_map, and markers data structures
     * @param size_x The x size to use for map initialization
     * @param size_y The y size to use for map initialization
     */
    virtual void initMaps(const unsigned int size_x, const unsigned int size_y);

    /**
     * @brief  Raytrace a line and apply some action at each step
     * @param  at The action to take... a functor
     * @param  x0 The starting x coordinate
     * @param  y0 The starting y coordinate
     * @param  x1 The ending x coordinate
     * @param  y1 The ending y coordinate
     * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
     */
    template <class ActionType>
    inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                             unsigned int max_length = UINT_MAX)
    {
        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = sign(dx);
        int offset_dy = sign(dy) * size_x_;

        unsigned int offset = y0 * size_x_ + x0;

        // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
        double dist = hypot(dx, dy);
        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

        // if x is dominant
        if (abs_dx >= abs_dy)
        {
            int error_y = abs_dx / 2;
            bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
            return;
        }

        // otherwise y is dominant
        int error_x = abs_dy / 2;
        bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
    }

  private:
    /**
     * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
     */
    template <class ActionType>
    inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                            int offset_b, unsigned int offset, unsigned int max_length)
    {
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i)
        {
            at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }

    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    mutex_t* access_;

  protected:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned char* costmap_;
    unsigned char default_value_;

    class MarkCell
    {
      public:
        MarkCell(unsigned char* costmap, unsigned char value) : costmap_(costmap), value_(value)
        {
        }
        inline void operator()(unsigned int offset)
        {
            costmap_[offset] = value_;
        }

      private:
        unsigned char* costmap_;
        unsigned char value_;
    };

    class PolygonOutlineCells
    {
      public:
        PolygonOutlineCells(const Costmap2D& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells)
            : costmap_(costmap), char_map_(char_map), cells_(cells)
        {
        }

        // just push the relevant cells back onto the list
        inline void operator()(unsigned int offset)
        {
            MapLocation loc;
            costmap_.indexToCells(offset, loc.x, loc.y);
            cells_.push_back(loc);
        }

      private:
        const Costmap2D& costmap_;
        const unsigned char* char_map_;
        std::vector<MapLocation>& cells_;
    };
};
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_H
