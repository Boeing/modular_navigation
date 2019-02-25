#include <eband_local_planner/eband_local_planner.h>

#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace eband_local_planner
{

namespace
{

bool checkOverlap(const Bubble& bubble1, const Bubble& bubble2, const double min_overlap)
{
    const double distance = distance2D(bubble1.center, bubble2.center);
    return (distance < min_overlap * (bubble1.expansion + bubble2.expansion));
}

geometry_msgs::Pose interpolate(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
                                const double fraction = 0.5)
{
    geometry_msgs::Pose interpolated;

    const geometry_msgs::Pose2D start_pose2D = convert(start);
    const geometry_msgs::Pose2D end_pose2D = convert(end);

    const double delta_theta = normalize_angle(end_pose2D.theta - start_pose2D.theta);

    tf2::Quaternion qt;
    qt.setRPY(0, 0, normalize_angle(start_pose2D.theta + fraction * delta_theta));

    interpolated.orientation.x = qt.x();
    interpolated.orientation.y = qt.y();
    interpolated.orientation.z = qt.z();
    interpolated.orientation.w = qt.w();

    interpolated.position.x = start.position.x + fraction * (end.position.x - start.position.x);
    interpolated.position.y = start.position.y + fraction * (end.position.y - start.position.y);
    interpolated.position.z = start.position.z + fraction * (end.position.z - start.position.z);

    return interpolated;
}

geometry_msgs::Wrench add(const geometry_msgs::Wrench& first, const geometry_msgs::Wrench& second)
{
    geometry_msgs::Wrench sum;
    sum.force.x = first.force.x + second.force.x;
    sum.force.y = first.force.y + second.force.y;
    sum.force.z = first.force.z + second.force.z;
    sum.torque.x = first.torque.x + second.torque.x;
    sum.torque.y = first.torque.y + second.torque.y;
    sum.torque.z = first.torque.z + second.torque.z;
    return sum;
}
}

EBandOptimiser::EBandOptimiser(const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap,
                               const int num_optim_iterations, const double internal_force_gain,
                               const double external_force_gain, const double tiny_bubble_distance,
                               const double tiny_bubble_expansion, const double min_bubble_overlap,
                               const int equilibrium_max_recursion_depth, const double equilibrium_relative_overshoot,
                               const double significant_force, const double costmap_weight,
                               const double costmap_inflation_radius)
    : local_costmap_(local_costmap), num_optim_iterations_(num_optim_iterations),
      internal_force_gain_(internal_force_gain), external_force_gain_(external_force_gain),
      tiny_bubble_distance_(tiny_bubble_distance), tiny_bubble_expansion_(tiny_bubble_expansion),
      min_bubble_overlap_(min_bubble_overlap), max_recursion_depth_approx_equi_(equilibrium_max_recursion_depth),
      equilibrium_relative_overshoot_(equilibrium_relative_overshoot), significant_force_(significant_force),
      costmap_weight_(costmap_weight), costmap_inflation_radius_(costmap_inflation_radius), visualization_(false),
      costmap_(local_costmap_->getCostmap()), robot_radius_(getCircumscribedRadius(*local_costmap_))
{
}

EBandOptimiser::~EBandOptimiser()
{
}

void EBandOptimiser::setVisualization(std::shared_ptr<EBandVisualization> eband_visual)
{
    eband_visual_ = eband_visual;
    visualization_ = true;
}

void EBandOptimiser::setPlan(const std::vector<geometry_msgs::Pose>& global_plan)
{
    try
    {
        elastic_band_ = convert(global_plan, *costmap_, costmap_weight_, costmap_inflation_radius_);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Conversion from plan to elastic band failed: " + std::string(e.what()));
    }

    refineBand(elastic_band_);
}

void EBandOptimiser::pruneTillPose(const geometry_msgs::Pose& robot_pose)
{
    Bubble robot_bubble = {robot_pose,
                           obstacleDistance(robot_pose, *costmap_, costmap_weight_, costmap_inflation_radius_)};
    for (int i = static_cast<int>(elastic_band_.size()) - 1; i >= 0; i--)
    {
        if (checkOverlap(robot_bubble, elastic_band_[static_cast<std::size_t>(i)], min_bubble_overlap_))
        {
            elastic_band_.erase(elastic_band_.begin(), elastic_band_.begin() + i);
            elastic_band_.insert(elastic_band_.begin(), robot_bubble);
            return;
        }
    }
    elastic_band_.insert(elastic_band_.begin(), robot_bubble);
    return;
}

void EBandOptimiser::append(const std::vector<geometry_msgs::Pose>& plan)
{
    try
    {
        const std::vector<Bubble> band_to_add = convert(plan, *costmap_, costmap_weight_, costmap_inflation_radius_);
        elastic_band_.insert(elastic_band_.end(), band_to_add.begin(), band_to_add.end());
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Conversion from plan to elastic band failed: " + std::string(e.what()));
    }
}

void EBandOptimiser::optimize()
{
    optimizeBand(elastic_band_);
}

void EBandOptimiser::optimizeBand(std::vector<Bubble>& band) const
{
    updateDistances(band);
    refineBand(band);

    for (int i = 0; i < num_optim_iterations_; i++)
    {
        ROS_DEBUG_STREAM("Optimization step: " << i);
        modifyBandArtificialForce(band);
        refineBand(band);
    }
}

void EBandOptimiser::updateDistances(std::vector<Bubble>& band) const
{
    for (std::size_t i = 0; i < band.size(); i++)
    {
        const double distance =
            obstacleDistance(band.at(i).center, *costmap_, costmap_weight_, costmap_inflation_radius_);
        if (distance == 0.0)
        {
            throw std::runtime_error("Frame " + std::to_string(i) + " of " + std::to_string(band.size()) +
                                     " in collision");
        }
        band.at(i).expansion = distance;
    }
}

void EBandOptimiser::refineBand(std::vector<Bubble>& band) const
{
    const std::size_t max_size = 50;

    std::vector<Bubble>::iterator iter = band.begin();
    while (std::distance(band.begin(), iter) < static_cast<int>(max_size))
    {
        const auto next = iter + 1;

        if (next == band.end())
            break;

        const double distance_to_next_bubble = distance2D(iter->center, next->center);
        const bool overlaping = (distance_to_next_bubble < min_bubble_overlap_ * (iter->expansion + next->expansion));

        if (!overlaping)
        {
            // check if a path is possible
            if (validPath(iter->center, next->center, *costmap_, costmap_weight_, tiny_bubble_distance_))
            {
                Bubble new_bubble = *iter;
                const double fraction = iter->expansion * min_bubble_overlap_ / distance_to_next_bubble;
                new_bubble.center = interpolate(iter->center, next->center, fraction);
                new_bubble.expansion =
                    obstacleDistance(new_bubble.center, *costmap_, costmap_weight_, costmap_inflation_radius_);

                // insert to band
                iter = band.insert(next, new_bubble);
            }
            else
            {
                // we can't connect this band... oh shit
                // ok now we need to a-star or something boss
                // for now lets fail *shrug*
                throw std::runtime_error("Failed to connect band at: " +
                                         std::to_string(std::distance(band.begin(), iter)));
            }
        }
        else
        {
            const auto next_next = next + 1;

            if (next_next == band.end())
                break;

            // check if a shortcut path is possible to the next next bubble
            if (validPath(iter->center, next_next->center, *costmap_, costmap_weight_, tiny_bubble_distance_))
            {
                // remove the next bubble
                iter = band.erase(next);
                --iter;
            }
            else
            {
                // we can't shortcut so we just have to deal
                ++iter;
            }
        }
    }
}

void EBandOptimiser::modifyBandArtificialForce(std::vector<Bubble>& band) const
{
    if (band.empty())
    {
        ROS_ERROR("Trying to modify an empty band.");
        return;
    }

    if (band.size() <= 2)
    {
        // nothing to do here -> we can stop right away
        return;
    }

    std::size_t i = 1;
    bool forward = true;
    // cycle 1x forwards and 1x backwards through band
    while ((i > 0) && (i < band.size() - 1))
    {
        const Bubble& prev_bubble = band.at(i - 1);
        const Bubble& curr_bubble = band.at(i);
        const Bubble& next_bubble = band.at(i + 1);

        const geometry_msgs::Wrench wrench = force(prev_bubble, curr_bubble, next_bubble);
        band.at(i) = applyForce(wrench, prev_bubble, curr_bubble, next_bubble);

        // next bubble
        if (forward)
        {
            i++;
            if (i == band.size() - 1)
            {
                // reached end of band - start backwards cycle until at start again - then stop
                forward = false;
                i--;
            }
        }
        else
        {
            i--;
        }
    }
}


Bubble EBandOptimiser::applyForce(const geometry_msgs::Wrench& wrench, const Bubble& prev_bubble,
                                  const Bubble& curr_bubble, const Bubble& next_bubble) const
{
    Bubble new_bubble = moveToEquilibrium(wrench, prev_bubble, curr_bubble, next_bubble);

    // Check there is a straight line path to the previous bubble
    if (!validPath(prev_bubble.center, new_bubble.center, *costmap_, costmap_weight_, tiny_bubble_distance_))
    {
        ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
        return curr_bubble;
    }

    // Check there is a straight line path to the next bubble
    if (!validPath(new_bubble.center, next_bubble.center, *costmap_, costmap_weight_, tiny_bubble_distance_))
    {
        ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
        return curr_bubble;
    }

    return new_bubble;
}

Bubble EBandOptimiser::moveBubble(const geometry_msgs::Wrench& wrench, const Bubble& curr_bubble,
                                  const double step_size) const
{
    geometry_msgs::Twist bubble_jump;
    bubble_jump.linear.x = step_size * wrench.force.x;
    bubble_jump.linear.y = step_size * wrench.force.y;
    bubble_jump.linear.z = 0.0;
    bubble_jump.angular.x = 0.0;
    bubble_jump.angular.y = 0.0;
    bubble_jump.angular.z = normalize_angle(step_size / robot_radius_ * wrench.torque.z);

    geometry_msgs::Pose2D new_bubble_pose2D;
    geometry_msgs::Pose2D bubble_pose2D = convert(curr_bubble.center);
    new_bubble_pose2D.x = bubble_pose2D.x + bubble_jump.linear.x;
    new_bubble_pose2D.y = bubble_pose2D.y + bubble_jump.linear.y;
    new_bubble_pose2D.theta = normalize_angle(bubble_pose2D.theta + bubble_jump.angular.z);

    Bubble new_bubble = curr_bubble;
    new_bubble.center = convert(new_bubble_pose2D);
    return new_bubble;
}

Bubble EBandOptimiser::moveToEquilibrium(const geometry_msgs::Wrench& wrench, const Bubble& prev_bubble,
                                         const Bubble& curr_bubble, const Bubble& next_bubble) const
{
    double step_size = curr_bubble.expansion;

    Bubble equilib_bubble = curr_bubble;

    for (int iteration_num = 0; iteration_num < max_recursion_depth_approx_equi_; iteration_num++)
    {
        Bubble new_bubble = moveBubble(wrench, curr_bubble, step_size);

        const double distance =
            obstacleDistance(new_bubble.center, *costmap_, costmap_weight_, costmap_inflation_radius_);
        if (distance < tiny_bubble_expansion_)
        {
            ROS_DEBUG("Calculation of Distance failed. Bubble moved into collision");
            return equilib_bubble;
        }
        new_bubble.expansion = distance;

        equilib_bubble = new_bubble;

        const geometry_msgs::Wrench new_wrench = force(prev_bubble, new_bubble, next_bubble);

        const double checksum_zero = (new_wrench.force.x * wrench.force.x) + (new_wrench.force.y * wrench.force.y) +
                                     (new_wrench.torque.z * wrench.torque.z);

        if (checksum_zero < 0.0)
        {
            const double abs_new_force =
                std::sqrt((new_wrench.force.x * new_wrench.force.x) + (new_wrench.force.y * new_wrench.force.y) +
                          (new_wrench.torque.z * new_wrench.torque.z));
            const double abs_old_force =
                std::sqrt((wrench.force.x * wrench.force.x) + (wrench.force.y * wrench.force.y) +
                          (wrench.torque.z * wrench.torque.z));

            if ((abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) &&
                (abs_new_force > significant_force_))
            {
                step_size *= 0.5;
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    return equilib_bubble;
}

geometry_msgs::Wrench EBandOptimiser::force(const Bubble& prev_bubble, const Bubble& curr_bubble,
                                            const Bubble& next_bubble) const
{
    const geometry_msgs::Wrench internal_force = internalForce(prev_bubble, curr_bubble, next_bubble);
    const geometry_msgs::Wrench external_force = externalForce(curr_bubble);
    const geometry_msgs::Wrench wrench =
        tangentialForce(add(internal_force, external_force), prev_bubble, curr_bubble, next_bubble);
    return wrench;
}

geometry_msgs::Wrench EBandOptimiser::internalForce(const Bubble& prev_bubble, const Bubble& curr_bubble,
                                                    const Bubble& next_bubble) const
{
    const double dx_1 = prev_bubble.center.position.x - curr_bubble.center.position.x;
    const double dy_1 = prev_bubble.center.position.y - curr_bubble.center.position.y;
    double distance_1 = std::sqrt((dx_1 * dx_1) + (dy_1 * dy_1));

    const double dx_2 = next_bubble.center.position.x - curr_bubble.center.position.x;
    const double dy_2 = next_bubble.center.position.y - curr_bubble.center.position.y;
    double distance_2 = std::sqrt((dx_2 * dx_2) + (dy_2 * dy_2));

    const double angular_z_1 = rotationZ(curr_bubble.center, prev_bubble.center) * robot_radius_;
    const double angular_z_2 = rotationZ(curr_bubble.center, next_bubble.center) * robot_radius_;

    // make sure to avoid division by  (almost) zero during force calculation (avoid numerical problems)
    // -> if difference/distance is (close to) zero then the force in this direction should be zero as well
    if (distance_1 <= tiny_bubble_distance_)
        distance_1 = 1000000.0;
    if (distance_2 <= tiny_bubble_distance_)
        distance_2 = 1000000.0;

    geometry_msgs::Wrench wrench;

    wrench.force.x = internal_force_gain_ * (dx_1 / distance_1 + dx_2 / distance_2);
    wrench.force.y = internal_force_gain_ * (dy_1 / distance_1 + dy_2 / distance_2);
    wrench.force.z = 0.0;

    wrench.torque.x = 0.0;
    wrench.torque.y = 0.0;
    wrench.torque.z = internal_force_gain_ * (angular_z_1 / distance_1 + angular_z_2 / distance_2);

    return wrench;
}

geometry_msgs::Wrench EBandOptimiser::externalForce(const Bubble& curr_bubble) const
{
    // calculate delta-poses (on upper edge of bubble) for x-direction
    geometry_msgs::Pose e1 = curr_bubble.center;
    e1.position.x += curr_bubble.expansion;
    const double distance1 = obstacleDistance(e1, *costmap_, costmap_weight_, costmap_inflation_radius_);

    // calculate delta-poses (on lower edge of bubble) for x-direction
    geometry_msgs::Pose e2 = curr_bubble.center;
    e2.position.x -= curr_bubble.expansion;
    const double distance2 = obstacleDistance(e2, *costmap_, costmap_weight_, costmap_inflation_radius_);

    // calculate delta-poses (on upper edge of bubble) for y-direction
    geometry_msgs::Pose e3 = curr_bubble.center;
    e3.position.y += curr_bubble.expansion;
    const double distance3 = obstacleDistance(e3, *costmap_, costmap_weight_, costmap_inflation_radius_);

    // calculate delta-poses (on lower edge of bubble) for y-direction
    geometry_msgs::Pose e4 = curr_bubble.center;
    e4.position.y -= curr_bubble.expansion;
    const double distance4 = obstacleDistance(e4, *costmap_, costmap_weight_, costmap_inflation_radius_);

    geometry_msgs::Wrench wrench;

    wrench.force.x = -external_force_gain_ * (distance2 - distance1) / (2.0 * curr_bubble.expansion);
    wrench.force.y = -external_force_gain_ * (distance4 - distance3) / (2.0 * curr_bubble.expansion);
    wrench.force.z = 0.0;

    wrench.torque.x = 0.0;
    wrench.torque.y = 0.0;
    wrench.torque.z = 0.0;

    return wrench;
}

geometry_msgs::Wrench EBandOptimiser::tangentialForce(const geometry_msgs::Wrench& wrench, const Bubble& prev_bubble,
                                                      const Bubble&, const Bubble& next_bubble) const
{
    const double dx = prev_bubble.center.position.x - next_bubble.center.position.x;
    const double dy = prev_bubble.center.position.y - next_bubble.center.position.y;
    const double angular_z = rotationZ(prev_bubble.center, next_bubble.center) * robot_radius_;

    // "project wrench" in middle bubble onto connecting vector
    // scalar wrench * difference
    const double scalar_fd = wrench.force.x * dx + wrench.force.y * dy + wrench.torque.z * angular_z;

    // abs of difference-vector: scalar difference * difference
    const double scalar_dd = dx * dx + dy * dy + angular_z * angular_z;

    // calculate orthogonal components
    geometry_msgs::Wrench ret = wrench;
    ret.force.x -= scalar_fd / scalar_dd * dx;
    ret.force.y -= scalar_fd / scalar_dd * dy;
    ret.force.z = 0;
    ret.torque.x = 0;
    ret.torque.y = 0;
    ret.torque.z -= scalar_fd / scalar_dd * angular_z;

    return ret;
}
}
