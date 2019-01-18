#ifndef __POINTS_TRANSFORMER_H__
#define __POINTS_TRANSFORMER_H__

#include <pcl/point_types.h>
class PointTranformer
{
  public:
    virtual ~PointTranformer() {}

    virtual void transPointToWorld(pcl::PointXYZI *const pi, pcl::PointXYZI *const po) = 0;
};

class PointTranformer3D : public PointTranformer
{
  public:
    Eigen::Matrix3d R_;
    Eigen::Vector3d T_;

    PointTranformer3D(Eigen::Vector3d &r, Eigen::Vector3d &t)
    {
        r[0] = atan2(sin(r[0]), cos(r[0]));
        r[1] = atan2(sin(r[1]), cos(r[1]));
        r[2] = atan2(sin(r[2]), cos(r[2]));

        R_ = Eigen::AngleAxisd(r[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(r[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(r[2], Eigen::Vector3d::UnitZ());
        T_ = t;
    }

    void transPointToWorld(pcl::PointXYZI *const pi, pcl::PointXYZI *const po)
    {
        Eigen::Vector3d v(pi->x, pi->y, pi->z);
        v = R_ * v + T_;

        po->x = v[0];
        po->y = v[1];
        po->z = v[2];

        //if (std::is_same<P, PointType>::value)
        {
            po->intensity = pi->intensity;
        }
    }
};
#endif