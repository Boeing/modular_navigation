#include <eband_local_planner/eband_local_planner.h>

#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace eband_local_planner
{

EBandPlanner::EBandPlanner(const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap,
                           const int num_optim_iterations, const double internal_force_gain,
                           const double external_force_gain, const double tiny_bubble_distance,
                           const double tiny_bubble_expansion, const double min_bubble_overlap,
                           const int equilibrium_max_recursion_depth, const double equilibrium_relative_overshoot,
                           const double significant_force, const double costmap_weight)
    : local_costmap_(local_costmap), num_optim_iterations_(num_optim_iterations),
      internal_force_gain_(internal_force_gain), external_force_gain_(external_force_gain),
      tiny_bubble_distance_(tiny_bubble_distance), tiny_bubble_expansion_(tiny_bubble_expansion),
      min_bubble_overlap_(min_bubble_overlap), max_recursion_depth_approx_equi_(equilibrium_max_recursion_depth),
      equilibrium_relative_overshoot_(equilibrium_relative_overshoot), significant_force_(significant_force),
      costmap_weight_(costmap_weight), visualization_(false), costmap_(local_costmap_->getCostmap())
{
}

EBandPlanner::~EBandPlanner()
{
}

void EBandPlanner::setVisualization(std::shared_ptr<EBandVisualization> eband_visual)
{
    eband_visual_ = eband_visual;
    visualization_ = true;
}

bool EBandPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    // check if plan valid (minimum 2 frames)
    if (global_plan.size() < 2)
    {
        ROS_ERROR(
            "Attempt to pass empty path to optimization. Valid path needs to have at least 2 Frames. This one has %lu.",
            global_plan.size());
        return false;
    }
    // copy plan to local member variable
    global_plan_ = global_plan;

    // check whether plan and costmap are in the same frame
    if (global_plan.front().header.frame_id != local_costmap_->getGlobalFrameID())
    {
        ROS_ERROR("Elastic Band expects plan for optimization in the %s frame, the plan was sent in the %s frame.",
                  local_costmap_->getGlobalFrameID().c_str(), global_plan.front().header.frame_id.c_str());
        return false;
    }

    try
    {
        elastic_band_ = convert(global_plan_, *costmap_, costmap_weight_);
    }
    catch (const std::exception& e)
    {
        ROS_WARN_STREAM("Conversion from plan to elastic band failed: " << e.what());
        return false;
    }

    // close gaps and remove redundant bubbles
    if (!refineBand(elastic_band_))
    {
        ROS_WARN("Band is broken. Could not close gaps in converted path. Path not set. Global re-planning needed");
        return false;
    }

    ROS_DEBUG("Refinement done - Band set.");
    return true;
}


bool EBandPlanner::getPlan(std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    // check if there is a band
    if (elastic_band_.empty())
    {
        ROS_WARN("Band is empty. There was no path successfully set so far.");
        return false;
    }

    global_plan = convert(elastic_band_);

    return true;
}

bool EBandPlanner::getBand(std::vector<Bubble>& elastic_band)
{
    elastic_band = elastic_band_;

    // check if there is a band
    if (elastic_band_.empty())
    {
        ROS_WARN("Band is empty.");
        return false;
    }

    return true;
}

bool EBandPlanner::addFrames(const std::vector<geometry_msgs::PoseStamped>& plan_to_add,
                             const AddAtPosition& add_frames_at)
{
    // check that there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
    if (elastic_band_.size() < 1)
    {
        ROS_WARN("Attempt to connect path to empty band. path not connected. Use SetPath instead");
        return false;
    }

    // check that plan which shall be added is not empty
    if (plan_to_add.empty())
    {
        ROS_WARN("Attempt to connect empty path to band. Nothing to do here.");
        return false;
    }

    // check whether plan and costmap are in the same frame
    if (plan_to_add.at(0).header.frame_id != local_costmap_->getGlobalFrameID())
    {
        ROS_ERROR(
            "Elastic Band expects robot pose for optimization in the %s frame, the pose was sent in the %s frame.",
            local_costmap_->getGlobalFrameID().c_str(), plan_to_add.at(0).header.frame_id.c_str());
        return false;
    }

    std::vector<Bubble> band_to_add;
    try
    {
        band_to_add = convert(plan_to_add, *costmap_, costmap_weight_);
    }
    catch (const std::exception& e)
    {
        ROS_WARN_STREAM("Conversion from plan to elastic band failed: " << e.what());
        return false;
    }

    // connect frames to existing band
    ROS_DEBUG("Checking for connections between current band and new bubbles");
    bool connected = false;
    int bubble_connect = -1;
    if (add_frames_at == add_front)
    {
        // add frames at the front of the current band
        // - for instance to connect band and current robot position
        for (int i = elastic_band_.size() - 1; i >= 0; i--)
        {
            // cycle over bubbles from End - connect to bubble furthest away but overlapping
            if (checkOverlap(band_to_add.back(), elastic_band_.at(i)))
            {
                bubble_connect = i;
                connected = true;
                break;
            }
        }
    }
    else
    {
        // add frames at the end of the current band
        // - for instance to connect new frames entering the moving window
        for (std::size_t i = 0; i < elastic_band_.size() - 1; i++)
        {
            // cycle over bubbles from Start - connect to bubble furthest away but overlapping
            if (checkOverlap(band_to_add.front(), elastic_band_.at(i)))
            {
                bubble_connect = i;
                connected = true;
                break;
            }
        }
    }

    // instantiate local copy of band
    std::vector<Bubble> tmp_band;
    std::vector<Bubble>::iterator tmp_iter1, tmp_iter2;

    // copy new frames to tmp_band
    tmp_band.assign(band_to_add.begin(), band_to_add.end());

    if (connected)
    {
        ROS_DEBUG("Connections found - composing new band by connecting new frames to bubble %d", bubble_connect);
        if (add_frames_at == add_front)
        {
            // compose new vector by appending elastic_band to new frames
            tmp_iter1 = elastic_band_.begin() + bubble_connect;
            ROS_ASSERT((tmp_iter1 >= elastic_band_.begin()) && (tmp_iter1 < elastic_band_.end()));
            tmp_band.insert(tmp_band.end(), tmp_iter1, elastic_band_.end());
        }
        else
        {
            // compose new vector by pre-appending elastic_band to new frames
            tmp_iter1 = elastic_band_.begin() + bubble_connect + 1;  // +1 - as insert only appends [start, end)
            ROS_ASSERT((tmp_iter1 > elastic_band_.begin()) && (tmp_iter1 <= elastic_band_.end()));
            tmp_band.insert(tmp_band.begin(), elastic_band_.begin(), tmp_iter1);
        }

        // done
        elastic_band_ = tmp_band;
        return true;
    }

    // otherwise, we need to do some more work - add complete band to tmp_band
    ROS_DEBUG("No direct connection found - Composing tmp band and trying to fill gap");
    if (add_frames_at == add_front)
    {
        // compose new vector by appending elastic_band to new frames
        tmp_band.insert(tmp_band.end(), elastic_band_.begin(), elastic_band_.end());
        // and get iterators to connecting bubbles
        tmp_iter1 = tmp_band.begin() + band_to_add.size() - 1;
        tmp_iter2 = tmp_iter1 + 1;
    }
    else
    {
        // compose new vector by pre-appending elastic_band to new frames
        tmp_band.insert(tmp_band.begin(), elastic_band_.begin(), elastic_band_.end());

        // and get iterators to connecting bubbles
        tmp_iter1 = tmp_band.begin() + elastic_band_.size() - 1;
        tmp_iter2 = tmp_iter1 + 1;
    }

    // just in case
    ROS_ASSERT(tmp_iter1 >= tmp_band.begin());
    ROS_ASSERT(tmp_iter2 < tmp_band.end());
    ROS_ASSERT(tmp_iter1 < tmp_iter2);
    if (!fillGap(tmp_band, tmp_iter1, tmp_iter2))
    {
        // we could not connect band and robot at its current position
        ROS_DEBUG("Could not connect robot pose to band - Failed to fill gap.");
        return false;
    }

    // otherwise - done
    elastic_band_ = tmp_band;

    return true;
}


bool EBandPlanner::optimizeBand()
{
    // check if there is a band
    if (elastic_band_.empty())
    {
        ROS_ERROR("Band is empty. Probably Band has not been set yet");
        return false;
    }

    // call optimization with member elastic_band_
    ROS_DEBUG_STREAM("Starting optimization of band of size: " << elastic_band_.size());
    if (!optimizeBand(elastic_band_))
    {
        ROS_DEBUG("Aborting Optimization. Changes discarded.");
        return false;
    }

    return true;
}


bool EBandPlanner::optimizeBand(std::vector<Bubble>& band)
{
    // check whether band and costmap are in the same frame
    if (band.front().center.header.frame_id != local_costmap_->getGlobalFrameID())
    {
        ROS_ERROR("Elastic Band expects plan for optimization in the %s frame, the plan was sent in the %s frame.",
                  local_costmap_->getGlobalFrameID().c_str(), band.front().center.header.frame_id.c_str());
        return false;
    }

    for (std::size_t i = 0; i < band.size(); i++)
    {
        // update Size of Bubbles in band by calculating Dist to nearest Obstacle [depends kinematic, environment]
        const double distance = obstacleDistance(band.at(i).center.pose, *costmap_, costmap_weight_);
        if (distance == 0.0)
        {
            // frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
            ROS_DEBUG("Optimization (Elastic Band) - Calculation of Distance failed. Frame %d of %lu in collision. "
                      "Plan invalid. Trying to refine band.",
                      i, band.size());
            // TODO if frame in collision try to repair band instead of aborting everything
            return false;
        }

        band.at(i).expansion = distance;
    }

    // close gaps and remove redundant bubbles
    if (!refineBand(band))
    {
        ROS_DEBUG("Elastic Band is broken. Could not close gaps in band. Global replanning needed.");
        return false;
    }

    // get a copy of current (valid) band
    std::vector<Bubble> tmp_band = band;

    // now optimize iteratively (for instance by miminizing the energy-function of the full system)
    for (int i = 0; i < num_optim_iterations_; i++)
    {
        ROS_DEBUG("Inside optimization: Cycle no %d", i);

        // calculate forces and apply changes
        if (!modifyBandArtificialForce(tmp_band))
        {
            ROS_DEBUG("Optimization failed while trying to modify Band.");
            // something went wrong -> discard changes and stop process
            return false;
        }

        // check whether band still valid - refine if neccesarry
        if (!refineBand(tmp_band))
        {
            ROS_DEBUG("Optimization failed while trying to refine modified band");
            // modified band is not valid anymore -> discard changes and stop process
            return false;
        }
    }

    // copy changes back to band
    band = tmp_band;
    return true;
}


// private methods

bool EBandPlanner::refineBand(std::vector<Bubble>& band)
{
    // check if band valid (minimum 2 bubbles)
    if (band.size() < 2)
    {
        ROS_WARN(
            "Attempt to convert empty band to plan. Valid band needs to have at least 2 Frames. This one has %lu .",
            band.size());
        return false;
    }

    // instantiate local variables
    bool success;
    std::vector<Bubble> tmp_band;
    std::vector<Bubble>::iterator start_iter, end_iter;

    // remove redundant Bubbles and fill gabs recursively
    tmp_band = band;
    start_iter = tmp_band.begin();
    end_iter = (tmp_band.end() - 1);  // -1 because .end() points "past the end"!

    success = removeAndFill(tmp_band, start_iter, end_iter);

    if (!success)
    {
        ROS_INFO("Band is broken. Could not close gaps.");
    }
    else
    {
        band = tmp_band;
    }

    return success;
}

bool EBandPlanner::removeAndFill(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,
                                 std::vector<Bubble>::iterator& end_iter)
{
    // instantiate local variables
    bool overlap;
    std::vector<Bubble>::iterator tmp_iter;

#ifdef DEBUG_EBAND_
    int debug_dist_start, debug_dist_iters;
    debug_dist_start = std::distance(band.begin(), start_iter);
    debug_dist_iters = std::distance(start_iter, end_iter);
    ROS_DEBUG("Refining Recursive - Check if Bubbles %d and %d overlapp. Total size of band %lu.", debug_dist_start,
              (debug_dist_start + debug_dist_iters), band.size());
#endif

    // check that iterators are still valid
    ROS_ASSERT(start_iter >= band.begin());
    ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT(start_iter < end_iter);

    // check whether start and end bubbles of this intervall overlap
    overlap = checkOverlap(*start_iter, *end_iter);

    if (overlap)
    {
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Bubbles overlapp, check for redundancies");
#endif

        // if there are bubbles between start and end of intervall remove them (they are redundant as start and end of
        // interval do overlap)
        if ((start_iter + 1) < end_iter)
        {
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles overlapp, removing Bubbles %d to %d.", (debug_dist_start + 1),
                      (debug_dist_start + debug_dist_iters - 1));
#endif
            // erase bubbles between start and end (but not start and end themself) and get new iterator to end (old one
            // is invalid)
            tmp_iter = band.erase((start_iter + 1), end_iter);

            // write back changed iterator pointing to the end of the intervall
            end_iter = tmp_iter;
        }

#ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Bubbles overlapp - DONE");
#endif

        // we are done here (leaf of this branch is reached)
        return true;
    }

    // if bubbles do not overlap -> check whether there are still bubbles between start and end
    if ((start_iter + 1) < end_iter)
    {
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Bubbles do not overlapp, go one recursion deeper");
#endif

        // split remaining sequence of bubbles
        // get distance between start and end iterator for this intervall
        int mid_int = std::distance(start_iter, end_iter);
        mid_int = mid_int / 2;  // division by integer implies floor (round down)

        // now get iterator pointing to the middle (roughly)
        tmp_iter = start_iter + mid_int;

        // and relative position of end_iter to tmp_iter
        int diff_int = std::distance(tmp_iter, end_iter);

        // after all this arithmetic - check that iterators are still valid
        ROS_ASSERT(start_iter >= band.begin());
        ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
        ROS_ASSERT(start_iter < end_iter);

        // and call removeAndFill recursively for the left intervall
        if (!removeAndFill(band, start_iter, tmp_iter))
        {
            // band is broken in this intervall and could not be fixed
            return false;
        }

        // careful at this point!!! if we filled in or removed bubbles end_iter is not valid anymore
        // but the relative position towards tmp_iter is still the same and tmp_iter was kept valid in the lower
        // recursion steps
        end_iter = tmp_iter + diff_int;

        // check that iterators are still valid - one more time
        ROS_ASSERT(start_iter >= band.begin());
        ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
        ROS_ASSERT((start_iter < tmp_iter) && (tmp_iter < end_iter));

        // TODO if the number of bubbles exceeds a limit exit early
        // not need to add more bubbles than we can process

        // o.k. we are done with left hand interval now do the same for the right hand intervall
        // but first get relative position of start and tmp iter
        diff_int = std::distance(start_iter, tmp_iter);
        if (!removeAndFill(band, tmp_iter, end_iter))
        {
            // band is broken in this intervall and could not be fixed
            return false;
        }

        // if we filled in bubbles vector might have been reallocated -> start_iter might be invalid
        start_iter = tmp_iter - diff_int;

        // check that iterators are still valid - almost done
        ROS_ASSERT(start_iter >= band.begin());
        ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
        ROS_ASSERT((start_iter < tmp_iter) && (tmp_iter < end_iter));

        // we reached the leaf but we are not yet done
        // -> we know that there are no redundant elements in the left intervall taken on its own
        // -> and we know the same holds for the right intervall
        // but the middle bubble itself might be redundant -> check it
        if (checkOverlap(*(tmp_iter - 1), *(tmp_iter + 1)))
        {
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Removing middle bubble");
#endif

            // again: get distance between (tmp_iter + 1) and end_iter, (+1 because we will erase tmp_iter itself)
            diff_int = std::distance((tmp_iter + 1), end_iter);

            // remove middle bubble and correct end_iter
            tmp_iter = band.erase(tmp_iter);
            end_iter = tmp_iter + diff_int;
        }

        // check that iterators are still valid - almost almost
        ROS_ASSERT(start_iter >= band.begin());
        ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
        ROS_ASSERT(start_iter < end_iter);

#ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Bubbles do not overlapp, go one recursion deeper DONE");
#endif

        // now we are done with this case
        return true;
    }

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Refining Recursive - Gap detected, fill recursive");
#endif

    // last possible case -> bubbles do not overlap AND there are nor bubbles in between -> try to fill gap recursively
    if (!fillGap(band, start_iter, end_iter))
    {
        // band is broken in this interval and could not be fixed (this should only be called on a leaf, so we put a
        // log_out here;)
        ROS_DEBUG("Failed to fill gap between bubble %d and %d.", distance(band.begin(), start_iter),
                  distance(band.begin(), end_iter));
        return false;
    }

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Refining Recursive - Gap detected, fill recursive DONE");
#endif

    // we could fill the gap (reached leaf of this branch)
    return true;
}


bool EBandPlanner::fillGap(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,
                           std::vector<Bubble>::iterator& end_iter)
{
    // insert bubbles in the middle between not-overlapping bubbles (e.g. (Dist > Size Bub1) && (Dist > Size Bub2) )
    // repeat until gaps are closed

    // instantiate local variables
    double distance = 0.0;
    Bubble interpolated_bubble;
    geometry_msgs::PoseStamped interpolated_center;
    std::vector<Bubble>::iterator tmp_iter;
    int start_num, end_num;

    // make sure this method was called for a valid element in the forces or bubbles vector
    ROS_ASSERT(start_iter >= band.begin());
    ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT(start_iter < end_iter);

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Fill recursive - interpolate");
#endif

    // interpolate between bubbles [depends kinematic]
    if (!interpolateBubbles(start_iter->center, end_iter->center, interpolated_center))
    {
        // interpolation failed (for whatever reason), so return with false
        start_num = std::distance(band.begin(), start_iter);
        end_num = std::distance(band.begin(), end_iter);
        ROS_DEBUG("Interpolation failed while trying to fill gap between bubble %d and %d.", start_num, end_num);
        return false;
    }

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Fill recursive - calc expansion of interpolated bubble");
#endif

    // calc Size of Bubbles by calculating Dist to nearest Obstacle [depends kinematic, environment]
    distance = obstacleDistance(interpolated_center.pose, *costmap_, costmap_weight_);

    if (distance <= 0)  // tiny_bubble_distance_)
    {
        // band broken! frame must not be immediately in collision -> otherwise calculation of gradient will later be
        // invalid
        start_num = std::distance(band.begin(), start_iter);
        end_num = std::distance(band.begin(), end_iter);
        ROS_WARN("Interpolated Bubble in Collision - failed to fill gap between bubble %d and %d.", start_num, end_num);

        return false;
        /*
        // TODO sample normally along line but focus in center?
        ROS_INFO_STREAM("Collision start: " << start_iter->center.pose.position.x << " "
                                            << start_iter->center.pose.position.y << " "
                                            << start_iter->center.pose.position.z);
        ROS_INFO_STREAM("Collision end: " << start_iter->center.pose.position.x << " "
                                          << end_iter->center.pose.position.y << " "
                                          << end_iter->center.pose.position.z);

        ROS_INFO_STREAM("Start distance: " << calcObstacleKinematicDistance(start_iter->center.pose));
        ROS_INFO_STREAM("End distance: " << calcObstacleKinematicDistance(end_iter->center.pose));

        Bubble start = *start_iter;
        Bubble end = *end_iter;
        start.expansion = 0.02;
        end.expansion = 0.02;
        eband_visual_->publishBubble("bubble_hypo", 8888, eband_visual_->red, start);
        eband_visual_->publishBubble("bubble_hypo", 9999, eband_visual_->red, end);

        std::cin.get();

        // TODO this means only that there is an obstacle on the direct interconnection between the bubbles
        // think about repair or rescue strategie
        if (!movePoseOutOfCollision(interpolated_center, 1.0))
        {
            ROS_WARN("Tried to move interpolated pose out of collision but exhausted search radius");
            return false;
        }
        */
    }

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Fill recursive - inserting interpolated bubble at (%f, %f), with expansion %f",
              interpolated_center.pose.position.x, interpolated_center.pose.position.y, distance);
#endif

    // insert bubble and assign center and expansion
    interpolated_bubble.center = interpolated_center;
    interpolated_bubble.expansion = distance;

    // insert bubble (vector.insert() inserts elements before given iterator) and get iterator pointing to it
    tmp_iter = band.insert(end_iter, interpolated_bubble);

    // insert is a little bit more tricky than erase, as it may require reallocation of the vector -> start and end iter
    // could be invalid
    start_iter = tmp_iter - 1;
    end_iter = tmp_iter + 1;

    // check that iterators are still valid - just in case :)
    ROS_ASSERT(start_iter >= band.begin());
    ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT((start_iter < tmp_iter) && (tmp_iter < end_iter));


#ifdef DEBUG_EBAND_
    ROS_DEBUG("Fill recursive - check overlap interpolated bubble and first bubble");
#endif

    // we have now two intervalls (left and right of inserted bubble) which need to be checked again and filled if
    // neccessary
    if (!checkOverlap(*start_iter, *tmp_iter))
    {
#ifdef DEBUG_EBAND
        ROS_DEBUG("Fill recursive - gap btw. interpolated and first bubble - fill recursive");
#endif

        // gap in left intervall -> try to fill
        if (!fillGap(band, start_iter, tmp_iter))
        {
            // band is broken in this intervall and could not be fixed
            return false;
        }
        // bubbles were inserted -> make sure to keep end_iter iterator valid
        end_iter = tmp_iter + 1;
    }

    // check that iterators are still valid - just in case :)
    ROS_ASSERT(start_iter >= band.begin());
    ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT((start_iter < tmp_iter) && (tmp_iter < end_iter));


#ifdef DEBUG_EBAND_
    ROS_DEBUG("Fill recursive - check overlap interpolated bubble and second bubble");
#endif

    if (!checkOverlap(*tmp_iter, *end_iter))
    {
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - gap btw. interpolated and second bubble - fill recursive");
#endif

        // get distance between start_iter and tmp_iter before filling right interval (in case of reallocation of
        // vector)
        int diff_int = std::distance(start_iter, tmp_iter);

        // gap in left interval -> try to fill
        if (!fillGap(band, tmp_iter, end_iter))
        {
            // band is broken in this interval and could not be fixed
            return false;
        }
        // bubbles were inserted -> make sure to keep start_iter iterator valid
        start_iter = tmp_iter - diff_int;
    }

    // check that iterators are still valid - just in case :)
    ROS_ASSERT(start_iter >= band.begin());
    ROS_ASSERT(end_iter < band.end());  // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT((start_iter < tmp_iter) && (tmp_iter < end_iter));


#ifdef DEBUG_EBAND_
    ROS_DEBUG("Fill recursive - gap closed");
#endif

    // bubbles overlap, iterators are kept valid -> done
    return true;
}

// optimization
bool EBandPlanner::modifyBandArtificialForce(std::vector<Bubble>& band)
{
    if (band.empty())
    {
        ROS_ERROR("Trying to modify an empty band.");
        return false;
    }

    if (band.size() <= 2)
    {
        // nothing to do here -> we can stop right away
        return true;
    }

    std::vector<geometry_msgs::WrenchStamped> internal_forces, external_forces, forces;
    geometry_msgs::WrenchStamped wrench;

#ifdef DEBUG_EBAND_
    // publish original band
    if (visualization_)
        eband_visual_->publishBand("bubbles", band);
#endif

    // init variables to calm down debug warnings
    wrench.header.stamp = ros::Time::now();
    wrench.header.frame_id = band[0].center.header.frame_id;
    wrench.wrench.force.x = 0.0;
    wrench.wrench.force.y = 0.0;
    wrench.wrench.force.z = 0.0;
    wrench.wrench.torque.x = 0.0;
    wrench.wrench.torque.y = 0.0;
    wrench.wrench.torque.z = 0.0;
    internal_forces.assign(band.size(), wrench);
    external_forces = internal_forces;
    forces = internal_forces;

    // TODO log timings of planner
    // instantiate variables for timing
    // ros::Time time_stamp1, time_stamp2;
    // ros::Duration duration;
    // time_stamp1 = ros::Time::now();

    // due to refinement band might change its size -> use while loop
    std::size_t i = 1;
    bool forward = true;  // cycle 1x forwards and 1x backwards through band
    while ((i > 0) && (i < band.size() - 1))
    {
        ROS_DEBUG("Modifying bubble %d.", i);

#ifdef DEBUG_EBAND_
        ROS_DEBUG("Calculating internal force for bubble %d.", i);
#endif

        if (!calcInternalForces(i, band, band.at(i), internal_forces.at(i)))
        {
            // calculation of internal forces failed - stopping optimization
            ROS_DEBUG("Calculation of internal forces failed");
            return false;
        }

#ifdef DEBUG_EBAND_
        if (visualization_)
            // publish internal forces
            eband_visual_->publishForce("internal_forces", i, eband_visual_->blue, internal_forces[i], band[i]);
        // Log out debug info about next step
        ROS_DEBUG("Calculating external force for bubble %d.", i);
#endif

        // if(!calcExternalForces(i, band, external_forces))
        if (!calcExternalForces(i, band.at(i), external_forces.at(i)))
        {
            // calculation of External Forces failed - stopping optimization
            ROS_DEBUG("Calculation of external forces failed");
            return false;
        }

#ifdef DEBUG_EBAND_
        if (visualization_)
            // publish external forces
            eband_visual_->publishForce("external_forces", i, eband_visual_->red, external_forces[i], band[i]);
        // Log out debug info about next step
        ROS_DEBUG("Superposing internal and external forces");
#endif

        // sum up external and internal forces over all bubbles
        forces.at(i).wrench.force.x = internal_forces.at(i).wrench.force.x + external_forces.at(i).wrench.force.x;
        forces.at(i).wrench.force.y = internal_forces.at(i).wrench.force.y + external_forces.at(i).wrench.force.y;
        forces.at(i).wrench.force.z = internal_forces.at(i).wrench.force.z + external_forces.at(i).wrench.force.z;

        forces.at(i).wrench.torque.x = internal_forces.at(i).wrench.torque.x + external_forces.at(i).wrench.torque.x;
        forces.at(i).wrench.torque.y = internal_forces.at(i).wrench.torque.y + external_forces.at(i).wrench.torque.y;
        forces.at(i).wrench.torque.z = internal_forces.at(i).wrench.torque.z + external_forces.at(i).wrench.torque.z;

#ifdef DEBUG_EBAND_
        ROS_DEBUG("Superpose forces: (x, y, theta) = (%f, %f, %f)", forces.at(i).wrench.force.x,
                  forces.at(i).wrench.force.y, forces.at(i).wrench.torque.z);
        ROS_DEBUG("Supressing tangential forces");
#endif

        if (!suppressTangentialForces(i, band, forces.at(i)))
        {
            // suppression of tangential forces failed
            ROS_DEBUG("Supression of tangential forces failed");
            return false;
        }

#ifdef DEBUG_EBAND_
        if (visualization_)
            // publish resulting forces
            eband_visual_->publishForce("resulting_forces", i, eband_visual_->green, forces[i], band[i]);
#endif

        ROS_DEBUG("Applying forces to modify band");
        if (!applyForces(i, band, forces))
        {
            // band invalid
            ROS_DEBUG("Band is invalid - Stopping Modification");
            return false;
        }

#ifdef DEBUG_EBAND_
        if (visualization_)
        {
            // publish band with changed bubble at resulting position
            eband_visual_->publishBand("bubbles", band);
            ros::Duration(0.01).sleep();
        }
#endif

        // next bubble
        if (forward)
        {
            i++;
            if (i == band.size() - 1)
            {
                // reached end of band - start backwards cycle until at start again - then stop
                forward = false;
                i--;
                ROS_DEBUG("Optimization Elastic Band - Forward cycle done, starting backward cycle");
            }
        }
        else
        {
            i--;
        }
    }

    return true;
}


bool EBandPlanner::applyForces(int bubble_num, std::vector<Bubble>& band,
                               std::vector<geometry_msgs::WrenchStamped> forces)
{
    // cycle over all bubbles except first and last (these are fixed)
    if (band.size() <= 2)
    {
        // nothing to do here -> we can stop right away - no forces calculated
        return true;
    }

    geometry_msgs::Twist bubble_jump;
    Bubble new_bubble = band.at(bubble_num);

    // move bubble
    geometry_msgs::Pose2D bubble_pose2D = convert(band.at(bubble_num).center.pose);

    // move according to bubble_new = bubble_old + alpha*force -> we choose alpha to be the current expansion of the
    // modified bubble
    bubble_jump.linear.x = band.at(bubble_num).expansion * forces.at(bubble_num).wrench.force.x;
    bubble_jump.linear.y = band.at(bubble_num).expansion * forces.at(bubble_num).wrench.force.y;
    bubble_jump.linear.z = 0.0;
    bubble_jump.angular.x = 0.0;
    bubble_jump.angular.y = 0.0;
    bubble_jump.angular.z =
        band.at(bubble_num).expansion / getCircumscribedRadius(*local_costmap_) * forces.at(bubble_num).wrench.torque.z;
    bubble_jump.angular.z = normalize_angle(bubble_jump.angular.z);

    // apply changes to calc tmp bubble position
    geometry_msgs::Pose2D new_bubble_pose2D;
    new_bubble_pose2D.x = bubble_pose2D.x + bubble_jump.linear.x;
    new_bubble_pose2D.y = bubble_pose2D.y + bubble_jump.linear.y;
    new_bubble_pose2D.theta = bubble_pose2D.theta + bubble_jump.angular.z;
    new_bubble_pose2D.theta = normalize_angle(new_bubble_pose2D.theta);

    // apply changes to local copy
    new_bubble.center.pose = convert(new_bubble_pose2D);

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Try moving bubble %d at (%f, %f, %f) by (%f, %f, %f).", bubble_num, bubble_pose2D.x, bubble_pose2D.y,
              bubble_pose2D.theta, bubble_jump.linear.x, bubble_jump.linear.y, bubble_jump.angular.z);
#endif

    // check validity of moved bubble

    // recalc expansion of bubble -> calc Size of Bubbles by calculating Dist to nearest Obstacle
    const double distance = obstacleDistance(new_bubble.center.pose, *costmap_, costmap_weight_);

    if (distance <= tiny_bubble_expansion_)
    {
        // frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
        ROS_DEBUG("Calculation of Distance failed. Frame %d of %lu in collision. Plan invalid. Discarding Changes",
                  bubble_num, band.size());

#ifdef DEBUG_EBAND_
        if (visualization_)
            eband_visual_->publishBubble("bubble_hypo", bubble_num, eband_visual_->red, new_bubble);
#endif

        // this bubble must not be changed, but band is still valid -> continue with other bubbles
        return true;
    }

    // so far o.k. -> assign distance to new bubble
    new_bubble.expansion = distance;

    // check whether step was reasonable
    geometry_msgs::WrenchStamped new_bubble_force = forces.at(bubble_num);

    // check whether we get a valid force calculation here
    if (!getForcesAt(bubble_num, band, new_bubble, new_bubble_force))
    {
        // error during calculation of forces for the new position - discard changes
        ROS_DEBUG("Cannot calculate forces on bubble %d at new position - discarding changes", bubble_num);
        return true;
    }

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Check for zero-crossings in force on bubble %d", bubble_num);
#endif

    // project force-vectors onto each other
    double checksum_zero = (new_bubble_force.wrench.force.x * forces.at(bubble_num).wrench.force.x) +
                           (new_bubble_force.wrench.force.y * forces.at(bubble_num).wrench.force.y) +
                           (new_bubble_force.wrench.torque.z * forces.at(bubble_num).wrench.torque.z);

    // if sign changes and ...
    if (checksum_zero < 0.0)
    {
        ROS_DEBUG("Detected zero-crossings in force on bubble %d. Checking total change in force.", bubble_num);
        // check the absolute values of the two vectors
        double abs_new_force = sqrt((new_bubble_force.wrench.force.x * new_bubble_force.wrench.force.x) +
                                    (new_bubble_force.wrench.force.y * new_bubble_force.wrench.force.y) +
                                    (new_bubble_force.wrench.torque.z * new_bubble_force.wrench.torque.z));
        double abs_old_force = sqrt((forces.at(bubble_num).wrench.force.x * forces.at(bubble_num).wrench.force.x) +
                                    (forces.at(bubble_num).wrench.force.x * forces.at(bubble_num).wrench.force.x) +
                                    (forces.at(bubble_num).wrench.torque.z * forces.at(bubble_num).wrench.torque.z));

        // force still has a significant high value (> ~75% of old force by default)
        if ((abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) && (abs_new_force > significant_force_))
        {
            ROS_DEBUG(
                "Detected significante change in force (%f to %f) on bubble %d. Entering Recursive Approximation.",
                abs_old_force, abs_new_force, bubble_num);
            // o.k. now we really have to take a closer look -> start recursive approximation to equilibrium-point
            int curr_recursion_depth = 0;
            geometry_msgs::Twist new_step_width;
            Bubble curr_bubble = band.at(bubble_num);
            geometry_msgs::WrenchStamped curr_bubble_force = forces.at(bubble_num);

            // half step size
            new_step_width.linear.x = 0.5 * bubble_jump.linear.x;
            new_step_width.linear.y = 0.5 * bubble_jump.linear.y;
            new_step_width.linear.z = 0.5 * bubble_jump.linear.z;
            new_step_width.angular.x = 0.5 * bubble_jump.angular.x;
            new_step_width.angular.y = 0.5 * bubble_jump.angular.y;
            new_step_width.angular.z = 0.5 * bubble_jump.angular.z;

            // one step deeper into the recursion
            if (moveApproximateEquilibrium(bubble_num, band, curr_bubble, curr_bubble_force, new_step_width,
                                           curr_recursion_depth))
            {
                // done with recursion - change bubble and hand it back
                new_bubble = curr_bubble;

#ifdef DEBUG_EBAND_
                const geometry_msgs::Pose2D curr_bubble_pose2D = convert(curr_bubble.center.pose);
                ROS_DEBUG("Instead - Try moving bubble %d at (%f, %f, %f) by (%f, %f, %f) to (%f, %f, %f).", bubble_num,
                          bubble_pose2D.x, bubble_pose2D.y, bubble_pose2D.theta, new_step_width.linear.x,
                          new_step_width.linear.y, new_step_width.angular.z, curr_bubble_pose2D.x, curr_bubble_pose2D.y,
                          curr_bubble_pose2D.theta);
#endif
            }
        }
    }

    // check validity of resulting band (given the moved bubble)

    // TODO use this routine not only to check whether gap can be filled but also to fill gap (if possible)
    // get local copy of band, set new position of moved bubble and init iterators
    std::vector<Bubble> tmp_band = band;
    std::vector<Bubble>::iterator start_iter, end_iter;
    tmp_band.at(bubble_num) = new_bubble;
    start_iter = tmp_band.begin();

    // check left connection (bubble and bubble-1)
    start_iter = start_iter + bubble_num - 1;
    end_iter = start_iter + 1;

    // check Overlap - if bubbles do not overlap try to fill gap
    if (!checkOverlap(*start_iter, *end_iter))
    {
        if (!fillGap(tmp_band, start_iter, end_iter))
        {
            ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
            // this bubble must not be changed, but band is still valid -> continue with other bubbles
            return true;
        }
    }

    // get fresh copy of band, set new position of bubble again and reinit iterators
    tmp_band = band;
    tmp_band.at(bubble_num) = new_bubble;
    start_iter = tmp_band.begin();

    // check right connection (bubble and bubble +1)
    start_iter = start_iter + bubble_num;
    end_iter = start_iter + 1;

    // check Overlap - if bubbles do not overlap try to fill gap
    if (!checkOverlap(*start_iter, *end_iter))
    {
        if (!fillGap(tmp_band, start_iter, end_iter))
        {
            ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
            // this bubble must not be changed, but band is still valid -> continue with other bubbles
            return true;
        }
    }

    band.at(bubble_num) = new_bubble;

    return true;
}


bool EBandPlanner::moveApproximateEquilibrium(const int& bubble_num, const std::vector<Bubble>& band,
                                              Bubble& curr_bubble,
                                              const geometry_msgs::WrenchStamped& curr_bubble_force,
                                              geometry_msgs::Twist& curr_step_width, const int& curr_recursion_depth)
{
    Bubble new_bubble = curr_bubble;
    geometry_msgs::WrenchStamped new_bubble_force = curr_bubble_force;

    // move bubble
    const geometry_msgs::Pose2D curr_bubble_pose2D = convert(curr_bubble.center.pose);
    geometry_msgs::Pose2D new_bubble_pose2D = convert(new_bubble.center.pose);

    // apply changes to calculate tmp bubble position
    new_bubble_pose2D.x = curr_bubble_pose2D.x + curr_step_width.linear.x;
    new_bubble_pose2D.y = curr_bubble_pose2D.y + curr_step_width.linear.y;
    new_bubble_pose2D.theta = curr_bubble_pose2D.theta + curr_step_width.angular.z;
    new_bubble_pose2D.theta = normalize_angle(new_bubble_pose2D.theta);

    // apply changes to local copy
    new_bubble.center.pose = convert(new_bubble_pose2D);

    // check validity of moved bubble

    // recalc expansion of bubble -> calc Size of Bubbles by calculating Dist to nearest Obstacle
    const double distance = obstacleDistance(new_bubble.center.pose, *costmap_, costmap_weight_);

    // we wont be able to calculate forces later on
    if (distance == 0.0)
        return false;

    // so far o.k. -> assign distance to new bubble
    new_bubble.expansion = distance;

    // check whether we get a valid force calculation here
    if (!getForcesAt(bubble_num, band, new_bubble, new_bubble_force))
        return false;

    // great - lets store this - this is better then what we had so far
    curr_bubble = new_bubble;

    // if everything is fine and we reached our maximum recursion depth
    if (curr_recursion_depth >= max_recursion_depth_approx_equi_)
        // break recursion at this point
        return true;

// now - let's check for zero-crossing
#ifdef DEBUG_EBAND_
    ROS_DEBUG("Check for zero-crossings in force on bubble %d - Recursion %d", bubble_num, curr_recursion_depth);
#endif

    int new_recursion_depth;
    geometry_msgs::Twist new_step_width;

    // check zero-crossing by projecting force-vectors onto each other
    double checksum_zero = (new_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x) +
                           (new_bubble_force.wrench.force.y * curr_bubble_force.wrench.force.y) +
                           (new_bubble_force.wrench.torque.z * curr_bubble_force.wrench.torque.z);

    if (checksum_zero < 0.0)
    {
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Detected zero-crossings in force on bubble %d - Recursion %d. Checking total change in force.",
                  bubble_num, curr_recursion_depth);
#endif

        // check the absolute values of the two vectors
        double abs_new_force = sqrt((new_bubble_force.wrench.force.x * new_bubble_force.wrench.force.x) +
                                    (new_bubble_force.wrench.force.y * new_bubble_force.wrench.force.y) +
                                    (new_bubble_force.wrench.torque.z * new_bubble_force.wrench.torque.z));
        double abs_old_force = sqrt((curr_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x) +
                                    (curr_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x) +
                                    (curr_bubble_force.wrench.torque.z * curr_bubble_force.wrench.torque.z));

        if ((abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) && (abs_new_force > significant_force_))
        {
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Detected significant change in force (%f to %f) on bubble %d - Recursion %d. Going one "
                      "Recursion deeper.",
                      abs_old_force, abs_new_force, bubble_num, curr_recursion_depth);
#endif

            // o.k. now we really have to take a closer look -> start recursive approximation to equilibrium-point
            new_recursion_depth = curr_recursion_depth + 1;

            // half step size - backward direction
            new_step_width.linear.x = -0.5 * curr_step_width.linear.x;
            new_step_width.linear.y = -0.5 * curr_step_width.linear.y;
            new_step_width.linear.z = -0.5 * curr_step_width.linear.z;
            new_step_width.angular.x = -0.5 * curr_step_width.angular.x;
            new_step_width.angular.y = -0.5 * curr_step_width.angular.y;
            new_step_width.angular.z = -0.5 * curr_step_width.angular.z;

            // one step deeper into the recursion
            if (moveApproximateEquilibrium(bubble_num, band, new_bubble, new_bubble_force, new_step_width,
                                           new_recursion_depth))
                // done with recursion - change bubble and hand it back
                curr_bubble = new_bubble;

            // otherwise - could not get a better value - return without change (curr_bubble as assigned above)
        }

        // otherwise - this is good enough for us - return with this value (curr_bubble as assigned above)
    }
    else
    {
#ifdef DEBUG_EBAND_
        ROS_DEBUG("No zero-crossings in force on bubble %d - Recursion %d. Continue walk in same direction. Going one "
                  "recursion deeper.",
                  bubble_num, curr_recursion_depth);
#endif

        // continue walk in same direction
        new_recursion_depth = curr_recursion_depth + 1;
        // half step size - backward direction
        new_step_width.linear.x = 0.5 * curr_step_width.linear.x;
        new_step_width.linear.y = 0.5 * curr_step_width.linear.y;
        new_step_width.linear.z = 0.5 * curr_step_width.linear.z;
        new_step_width.angular.x = 0.5 * curr_step_width.angular.x;
        new_step_width.angular.y = 0.5 * curr_step_width.angular.y;
        new_step_width.angular.z = 0.5 * curr_step_width.angular.z;

        // one step deeper into the recursion
        if (moveApproximateEquilibrium(bubble_num, band, new_bubble, new_bubble_force, new_step_width,
                                       new_recursion_depth))
            // done with recursion - change bubble and hand it back
            curr_bubble = new_bubble;

        // otherwise - could not get a better value - return without change (curr_bubble as asigned above)
    }

    return true;
}


bool EBandPlanner::getForcesAt(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble,
                               geometry_msgs::WrenchStamped& forces)
{
    geometry_msgs::WrenchStamped internal_force, external_force;

    if (!calcInternalForces(bubble_num, band, curr_bubble, internal_force))
    {
        // calculation of internal forces failed - stopping optimization
        ROS_DEBUG("Calculation of internal forces failed");
        return false;
    }

    if (!calcExternalForces(bubble_num, curr_bubble, external_force))
    {
        // calculation of External Forces failed - stopping optimization
        ROS_DEBUG("Calculation of external forces failed");
        return false;
    }

    // sum up external and internal forces over all bubbles
    forces.wrench.force.x = internal_force.wrench.force.x + external_force.wrench.force.x;
    forces.wrench.force.y = internal_force.wrench.force.y + external_force.wrench.force.y;
    forces.wrench.force.z = internal_force.wrench.force.z + external_force.wrench.force.z;

    forces.wrench.torque.x = internal_force.wrench.torque.x + external_force.wrench.torque.x;
    forces.wrench.torque.y = internal_force.wrench.torque.y + external_force.wrench.torque.y;
    forces.wrench.torque.z = internal_force.wrench.torque.z + external_force.wrench.torque.z;

    if (!suppressTangentialForces(bubble_num, band, forces))
    {
        // suppression of tangential forces failed
        ROS_DEBUG("Supression of tangential forces failed");
        return false;
    }

    return true;
}


bool EBandPlanner::calcInternalForces(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble,
                                      geometry_msgs::WrenchStamped& forces)
{
    // cycle over all bubbles except first and last (these are fixed)
    if (band.size() <= 2)
    {
        // nothing to do here -> we can stop right away - no forces calculated
        return true;
    }

    ROS_ASSERT(bubble_num > 0);
    ROS_ASSERT(bubble_num < int(band.size()) - 1);

    const Bubble& prev_bubble = band[bubble_num - 1];
    const Bubble& next_bubble = band[bubble_num + 1];

    const double dx_1 = prev_bubble.center.pose.position.x - curr_bubble.center.pose.position.x;
    const double dy_1 = prev_bubble.center.pose.position.y - curr_bubble.center.pose.position.y;
    double distance_1 = std::sqrt((dx_1 * dx_1) + (dy_1 * dy_1));

    const double dx_2 = next_bubble.center.pose.position.x - curr_bubble.center.pose.position.x;
    const double dy_2 = next_bubble.center.pose.position.y - curr_bubble.center.pose.position.y;
    double distance_2 = std::sqrt((dx_2 * dx_2) + (dy_2 * dy_2));

    const double angular_z_1 =
        rotationZ(curr_bubble.center.pose, prev_bubble.center.pose) * getCircumscribedRadius(*local_costmap_);
    const double angular_z_2 =
        rotationZ(curr_bubble.center.pose, next_bubble.center.pose) * getCircumscribedRadius(*local_costmap_);

    // make sure to avoid division by  (almost) zero during force calculation (avoid numerical problems)
    // -> if difference/distance is (close to) zero then the force in this direction should be zero as well
    if (distance_1 <= tiny_bubble_distance_)
        distance_1 = 1000000.0;
    if (distance_2 <= tiny_bubble_distance_)
        distance_2 = 1000000.0;

    // forces model an elastic band and are normed (distance) to render forces for small and large bubbles the same
    geometry_msgs::Wrench wrench;

    wrench.force.x = internal_force_gain_ * (dx_1 / distance_1 + dx_2 / distance_2);
    wrench.force.y = internal_force_gain_ * (dy_1 / distance_1 + dy_2 / distance_2);
    wrench.torque.z = internal_force_gain_ * (angular_z_1 / distance_1 + angular_z_2 / distance_2);

    // store wrench in vector
    forces.wrench = wrench;

    return true;
}

bool EBandPlanner::calcExternalForces(int, Bubble curr_bubble, geometry_msgs::WrenchStamped& forces)
{
    // init tmp variables
    geometry_msgs::Pose edge;
    geometry_msgs::Wrench wrench;

    // calculate delta-poses (on upper edge of bubble) for x-direction
    edge = curr_bubble.center.pose;
    edge.position.x = edge.position.x + curr_bubble.expansion;

    // get expansion on bubble at this point
    double distance1 = obstacleDistance(edge, *costmap_, costmap_weight_);

    // calculate delta-poses (on lower edge of bubble) for x-direction
    edge.position.x = edge.position.x - 2.0 * curr_bubble.expansion;

    // get expansion on bubble at this point
    double distance2 = obstacleDistance(edge, *costmap_, costmap_weight_);

    // calculate difference-quotient (approx. of derivative) in x-direction
    if (curr_bubble.expansion <= tiny_bubble_expansion_)
    {
        // avoid division by (almost) zero to avoid numerical problems
        wrench.force.x = -external_force_gain_ * (distance2 - distance1) / (2.0 * tiny_bubble_expansion_);
        // actually we should never end up here - band should have been considered as broken
        ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably "
                  "ill configured");
    }
    else
        wrench.force.x = -external_force_gain_ * (distance2 - distance1) / (2.0 * curr_bubble.expansion);

    // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponding
    // term

    // calculate delta-poses (on upper edge of bubble) for y-direction
    edge = curr_bubble.center.pose;
    edge.position.y = edge.position.y + curr_bubble.expansion;

    // get expansion on bubble at this point
    distance1 = obstacleDistance(edge, *costmap_, costmap_weight_);

    // calculate delta-poses (on lower edge of bubble) for x-direction
    edge.position.y = edge.position.y - 2.0 * curr_bubble.expansion;

    // get expansion on bubble at this point
    distance2 = obstacleDistance(edge, *costmap_, costmap_weight_);

    // calculate difference-quotient (approx. of derivative) in x-direction
    if (curr_bubble.expansion <= tiny_bubble_expansion_)
    {
        // avoid division by (almost) zero to avoid numerical problems
        wrench.force.y = -external_force_gain_ * (distance2 - distance1) / (2.0 * tiny_bubble_expansion_);
        // actually we should never end up here - band should have been considered as broken
        ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably "
                  "ill configured");
    }
    else
        wrench.force.y = -external_force_gain_ * (distance2 - distance1) / (2.0 * curr_bubble.expansion);
    // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponsing
    // term

    // no force in z-direction
    wrench.force.z = 0.0;

    // no torque around x and y axis
    wrench.torque.x = 0.0;
    wrench.torque.y = 0.0;

    // calculate delta-poses (on upper edge of bubble) for x-direction
    geometry_msgs::Pose2D edge_pose2D = convert(curr_bubble.center.pose);
    edge_pose2D.theta = edge_pose2D.theta + (curr_bubble.expansion / getCircumscribedRadius(*local_costmap_));
    edge_pose2D.theta = normalize_angle(edge_pose2D.theta);

    // get expansion on bubble at this point
    distance1 = obstacleDistance(edge, *costmap_, costmap_weight_);

    // calculate delta-poses (on lower edge of bubble) for x-direction
    edge_pose2D = convert(edge);
    edge_pose2D.theta = edge_pose2D.theta - 2.0 * (curr_bubble.expansion / getCircumscribedRadius(*local_costmap_));
    edge_pose2D.theta = normalize_angle(edge_pose2D.theta);

    // get expansion on bubble at this point
    distance2 = obstacleDistance(edge, *costmap_, costmap_weight_);

    // calculate difference-quotient (approx. of derivative) in x-direction
    if (curr_bubble.expansion <= tiny_bubble_expansion_)
    {
        // avoid division by (almost) zero to avoid numerical problems
        wrench.torque.z = -external_force_gain_ * (distance2 - distance1) / (2.0 * tiny_bubble_expansion_);
        // actually we should never end up here - band should have been considered as broken
        ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably "
                  "ill configured");
    }
    else
        wrench.torque.z = -external_force_gain_ * (distance2 - distance1) / (2.0 * curr_bubble.expansion);
    // TODO above equations skip term to make forces continuous at end of influence region - test to add
    // corresponsing term

    // assign wrench to forces vector
    forces.wrench = wrench;

    return true;
}


bool EBandPlanner::suppressTangentialForces(int bubble_num, std::vector<Bubble> band,
                                            geometry_msgs::WrenchStamped& forces)
{
    // cycle over all bubbles except first and last (these are fixed)
    if (band.size() <= 2)
    {
        // nothing to do here -> we can stop right away - no forces calculated
        return true;
    }

    // make sure this method was called for a valid element in the forces or bubbles vector
    ROS_ASSERT(bubble_num > 0);
    ROS_ASSERT(bubble_num < int(band.size()) - 1);

    const Bubble& prev_bubble = band[bubble_num - 1];
    const Bubble& next_bubble = band[bubble_num + 1];

    const double dx = prev_bubble.center.pose.position.x - next_bubble.center.pose.position.x;
    const double dy = prev_bubble.center.pose.position.y - next_bubble.center.pose.position.y;
    const double angular_z =
        rotationZ(next_bubble.center.pose, prev_bubble.center.pose) * getCircumscribedRadius(*local_costmap_);

    // "project wrench" in middle bubble onto connecting vector
    // scalar wrench*difference
    const double scalar_fd =
        forces.wrench.force.x * dx + forces.wrench.force.y * dy + forces.wrench.torque.z * angular_z;

    // abs of difference-vector: scalar difference*difference
    const double scalar_dd = dx * dx + dy * dy + angular_z * angular_z;

    // avoid division by (almost) zero -> check if bubbles have (almost) same center-pose
    if (scalar_dd <= tiny_bubble_distance_)
    {
        // there are redundant bubbles, this should normally not hapen -> probably error in band refinement
        ROS_DEBUG("Calculating tangential forces for redundant bubbles. Bubble should have been removed. Local Planner "
                  "probably ill configured");
    }

    // calculate orthogonal components
    forces.wrench.force.x = forces.wrench.force.x - scalar_fd / scalar_dd * dx;
    forces.wrench.force.y = forces.wrench.force.y - scalar_fd / scalar_dd * dy;
    forces.wrench.force.z = 0;
    forces.wrench.torque.x = 0;
    forces.wrench.torque.y = 0;
    forces.wrench.torque.z = forces.wrench.torque.z - scalar_fd / scalar_dd * angular_z;

    return true;
}

// problem (geometry) dependant functions
bool EBandPlanner::interpolateBubbles(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,
                                      geometry_msgs::PoseStamped& interpolated_center)
{
    // copy header
    interpolated_center.header = start_center.header;

    // interpolate angles
    // TODO make this in a better way
    // - for instance use "slerp" to interpolate directly between quaternions
    // - or work with pose2D right from the beginnning
    // convert quaternions to euler angles - at this point this no longer works in 3D !!
    geometry_msgs::Pose2D start_pose2D = convert(start_center.pose);
    geometry_msgs::Pose2D end_pose2D = convert(end_center.pose);

    // calc mean of theta angle
    const double delta_theta = normalize_angle(end_pose2D.theta - start_pose2D.theta) / 2.0;

    geometry_msgs::Pose2D interpolated_pose2D;
    interpolated_pose2D.theta = start_pose2D.theta + delta_theta;
    interpolated_pose2D.theta = normalize_angle(interpolated_pose2D.theta);

    // convert back to quaternion
    interpolated_pose2D.x = 0.0;
    interpolated_pose2D.y = 0.0;
    interpolated_center.pose = convert(interpolated_pose2D);

    // interpolate positions
    interpolated_center.pose.position.x = (end_center.pose.position.x + start_center.pose.position.x) / 2.0;
    interpolated_center.pose.position.y = (end_center.pose.position.y + start_center.pose.position.y) / 2.0;
    interpolated_center.pose.position.z = (end_center.pose.position.z + start_center.pose.position.z) / 2.0;

    // TODO ideally this would take into account kinematics of the robot and for instance use splines

    return true;
}

bool EBandPlanner::checkOverlap(Bubble bubble1, Bubble bubble2)
{
    // calc (kinematic) Distance between bubbles
    const double distance = distance2D(bubble1.center.pose, bubble2.center.pose);

    // compare with size of the two bubbles
    if (distance >= min_bubble_overlap_ * (bubble1.expansion + bubble2.expansion))
        return false;

    // TODO this does not account for kinematic properties -> improve

    // everything fine - bubbles overlap
    return true;
}
}
