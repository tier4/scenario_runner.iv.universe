#include <scenario_api/scenario_calc_dist_utils.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/assign/list_of.hpp>

namespace scenario_api_utils
{
namespace bg = boost::geometry;

double calcDistFromPolygonToPointCloud(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> & pointcloud_ptr,
  const ScenarioAPIAutoware::Polygon & poly, const bool consider_height, const double top,
  const double bottom, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
{
  if (pointcloud_ptr->header.frame_id != "base_link") {
    RCLCPP_WARN_THROTTLE(
      logger.get_child("calcDistFromPolygonToPointCloud"), *clock,
      std::chrono::milliseconds{5000}.count(), "frame_id of point cloud must be base_link");
    return 0.0;  // return short distance
  } else {
    // convert ros msg to pointcloud
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::fromROSMsg(*pointcloud_ptr, pointcloud);

    // calculate distance from polygon to point
    // bg::distance(): https://faithandbrave.hateblo.jpjp/entry/20110803/1312299337
    double minimum_distance = 1000.0;
    for (const auto & p : pointcloud.points) {
      if ((top >= p.z && p.z >= bottom) || !consider_height) {
        const ScenarioAPIAutoware::Point point2d(p.x, p.y);
        const double distance = bg::distance(point2d, poly);
        if (distance < minimum_distance) minimum_distance = distance;
      }
    }
    return minimum_distance;
  }
}

ScenarioAPIAutoware::Polygon makeRelativePolygonFromSelf(
  const geometry_msgs::msg::Pose self_pose, const geometry_msgs::msg::Pose obj_pose,
  const geometry_msgs::msg::Vector3 obj_size)
{
  // get object position
  double obj_x = obj_pose.position.x;
  double obj_y = obj_pose.position.y;
  double obj_yaw = yawFromQuat(obj_pose.orientation);

  // get self position
  double self_x = self_pose.position.x;
  double self_y = self_pose.position.y;
  double self_yaw = yawFromQuat(self_pose.orientation);

  // calculate distance x, y *in self-centered coordinate*
  double dif_x = (obj_x - self_x) * std::cos(-self_yaw) - (obj_y - self_y) * std::sin(-self_yaw);
  double dif_y = (obj_x - self_x) * std::sin(-self_yaw) + (obj_y - self_y) * std::cos(-self_yaw);
  double dif_yaw = obj_yaw - self_yaw;

  // rename
  double h = obj_size.x;  // object height
  double w = obj_size.y;  // object width

  // create object polygon *in self-centered coordinate*
  ScenarioAPIAutoware::Polygon obj_poly;

  // base polygon
  bg::exterior_ring(obj_poly) = boost::assign::list_of<ScenarioAPIAutoware::Point>(
    h / 2.0, w / 2.0)(-h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);
  // rotation
  bg::strategy::transform::rotate_transformer<bg::radian, double, 2, 2> rotate(
    -dif_yaw);  // original:clockwise rotation
  ScenarioAPIAutoware::Polygon rotate_obj_poly;
  bg::transform(obj_poly, rotate_obj_poly, rotate);
  // translation f
  bg::strategy::transform::translate_transformer<double, 2, 2> translate(
    dif_x, dif_y);  // original:clockwise rotation
  ScenarioAPIAutoware::Polygon translate_obj_poly;
  bg::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

double calcDistOfPolygon(
  const ScenarioAPIAutoware::Polygon & poly, const ScenarioAPIAutoware::Polygon & poly2)
{
  return bg::distance(poly, poly2);
}

}  // namespace scenario_api_utils