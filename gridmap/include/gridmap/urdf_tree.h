#ifndef GRIDMAP_URDF_TREE
#define GRIDMAP_URDF_TREE

#include <Eigen/Geometry>

#include <urdf/model.h>

namespace gridmap
{

class URDFTree
{
  public:
    explicit URDFTree(const urdf::ModelInterface& urdf) : urdf_(urdf)
    {
    }

    Eigen::Isometry3d getTransform(const std::string& child, const std::string& parent)
    {
        urdf::LinkConstSharedPtr link = urdf_.getLink(child);

        if (!link)
            throw std::runtime_error("Missing link '" + child + "' from static transforms in URDF");

        const urdf::LinkConstSharedPtr parent_link = urdf_.getLink(parent);

        if (!parent_link)
            throw std::runtime_error("Missing link '" + parent + "' from static transforms in URDF");

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        while (link != parent_link)
        {
            if (!link->getParent())
                throw std::runtime_error("Link '" + link->name + "' has no parent");

            if (link->parent_joint->type != urdf::Joint::FIXED)
                throw std::runtime_error("Link '" + link->name + "' is not connected to parent with a FIXED joint");

            const urdf::Pose& pose = link->parent_joint->parent_to_joint_origin_transform;
            const Eigen::Isometry3d tr =
                Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
                Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);

            transform = tr * transform;

            link = link->getParent();
        }

        return transform;
    }

  private:
    const urdf::ModelInterface urdf_;
};

}  // namespace gridmap

#endif
