#ifndef GLOBAL_PLANNER_DIJKSTRA_H
#define GLOBAL_PLANNER_DIJKSTRA_H

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <global_planner/expander.h>
#include <global_planner/planner_core.h>

// inserting onto the priority blocks
#define push_cur(n)                                                                                                    \
    {                                                                                                                  \
        if (n >= 0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && currentEnd_ < PRIORITYBUFSIZE)    \
        {                                                                                                              \
            currentBuffer_[currentEnd_++] = n;                                                                         \
            pending_[n] = true;                                                                                        \
        }                                                                                                              \
    }
#define push_next(n)                                                                                                   \
    {                                                                                                                  \
        if (n >= 0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && nextEnd_ < PRIORITYBUFSIZE)       \
        {                                                                                                              \
            nextBuffer_[nextEnd_++] = n;                                                                               \
            pending_[n] = true;                                                                                        \
        }                                                                                                              \
    }
#define push_over(n)                                                                                                   \
    {                                                                                                                  \
        if (n >= 0 && n < ns_ && !pending_[n] && getCost(costs, n) < lethal_cost_ && overEnd_ < PRIORITYBUFSIZE)       \
        {                                                                                                              \
            overBuffer_[overEnd_++] = n;                                                                               \
            pending_[n] = true;                                                                                        \
        }                                                                                                              \
    }

namespace global_planner
{
class DijkstraExpansion : public Expander
{
  public:
    DijkstraExpansion(std::shared_ptr<PotentialCalculator> p_calc, int nx, int ny);
    ~DijkstraExpansion();
    bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                             int cycles, float* potential);

    /**
     * @brief  Sets or resets the size of the map
     * @param nx The x size of the map
     * @param ny The y size of the map
     */
    void setSize(int nx, int ny); /**< sets or resets the size of the map */

    void setNeutralCost(unsigned char neutral_cost)
    {
        neutral_cost_ = neutral_cost;
        priorityIncrement_ = 2 * neutral_cost_;
    }

    void setPreciseStart(bool precise)
    {
        precise_ = precise;
    }

  private:
    /**
     * @brief  Updates the cell at index n
     * @param costs The costmap
     * @param potential The potential array in which we are calculating
     * @param n The index to update
     */
    void updateCell(unsigned char* costs, float* potential, int n); /** updates the cell at index n */

    float getCost(unsigned char* costs, int n)
    {
        float c = costs[n];
        if (c < lethal_cost_ - 1 || (unknown_ && c == 255))
        {
            c = c * factor_ + neutral_cost_;
            if (c >= lethal_cost_)
                c = lethal_cost_ - 1;
            return c;
        }
        return lethal_cost_;
    }

    /** block priority buffers */
    int *buffer1_, *buffer2_, *buffer3_;             /**< storage buffers for priority blocks */
    int *currentBuffer_, *nextBuffer_, *overBuffer_; /**< priority buffer block ptrs */
    int currentEnd_, nextEnd_, overEnd_;             /**< end points of arrays */
    bool* pending_;                                  /**< pending_ cells during propagation */
    bool precise_;

    /** block priority thresholds */
    float threshold_;         /**< current threshold */
    float priorityIncrement_; /**< priority threshold increment */
};

}  // end namespace global_planner
#endif
