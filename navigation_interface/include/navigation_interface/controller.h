#ifndef NAVIGATION_INTERFACE_CONTROLLER_INTERFACE_H
#define NAVIGATION_INTERFACE_CONTROLLER_INTERFACE_H

#include <navigation_interface/types/control.h>
#include <navigation_interface/types/trajectory.h>

#include <gridmap/map_data.h>

#include <xmlrpcpp/XmlRpc.h>

#include <boost/optional.hpp>

#include <memory>

namespace navigation_interface
{

class Controller
{
  public:

    enum class Outcome
    {
        FAILED,
        SUCCESSFUL,
        COMPLETE
    };

    struct Result
    {
        Outcome outcome;

        std::size_t target_i;

        Eigen::VectorXd command;
    };

    virtual bool setTrajectory(const Trajectory& trajectory) = 0;
    virtual void clearTrajectory() = 0;

    virtual boost::optional<std::string> trajectoryId() const = 0;
    virtual boost::optional<Trajectory> trajectory() const = 0;

    virtual Result control(const ros::SteadyTime& time, const KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom) = 0;

    virtual void initialize(const XmlRpc::XmlRpcValue& parameters,
                            const std::shared_ptr<const gridmap::MapData>& map_data) = 0;

    virtual ~Controller()
    {
    }

  protected:
    Controller()
    {
    }
};

};

#endif
