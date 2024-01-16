#include <geometry_msgs/Point.h>
#include <icp_study/frame.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

void HSVtoRGB(int h, int s, int v, int& r, int& g, int& b) {
  int i2 = h / 43;
  int remainder = (h - (i2 * 43)) * 6;

  int p = (v * (255 - s)) >> 8;
  int q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  int t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (i2) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      r = v;
      g = p;
      b = q;
      break;
  }
}

void VisualizeFrame(ros::Publisher marker_pub, Frame& frame, int color) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  for (int i = 0; i < frame.GetSize(); i++) {
    double x = frame.GetOnePoint(i)(0);
    double y = frame.GetOnePoint(i)(1);
    // double height = height_grid.GetCells()[i].height;

    // bool disabled = height_grid.GetCells()[i].disabled;

    marker = visualization_msgs::Marker();
    // Set the frame ID and timestamp.
    marker.header.frame_id = "velo_link";
    marker.header.stamp = ros::Time(frame.GetTimestamp());
    // Set the namespace and id for this marker. This serves to create a unique ID Any marker sent with the same
    // namespace and id will overwrite the old one
    if (color == 0) {
      marker.ns = "frame_0";
    } else if (color == 1) {
      marker.ns = "frame_1";
    } else if (color == 2) {
      marker.ns = "frame_2";
    }
    marker.id = i;
    // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = -1.73;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = frame.GetResolution();
    marker.scale.y = frame.GetResolution();
    marker.scale.z = 0.001;

    if (color == 0) {
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
    } else if (color == 1) {
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
    } else if (color == 2) {
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
    }

    marker.color.a = 0.5;
    // marker.color.a = 0.5 * height;

    // // if disabled, make it yellow
    // if (disabled) {
    //   marker.color.r = 1;
    //   marker.color.g = 1;
    //   marker.color.b = 0;
    //   marker.color.a = 0.5;
    // }

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }

  marker_pub.publish(marker_array);
}

void VisualizeLineBetweenMatchingPoints(ros::Publisher marker_pub, Frame F1, Frame F2) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  Eigen::MatrixXd points1 = F1.GetPoints();
  Eigen::MatrixXd points2 = F2.GetPoints();

  for (int i = 0; i < F1.GetSize(); i++) {
    double x1 = points1(0, i);
    double y1 = points1(1, i);
    // double height1 = cells1[i].height;

    double x2 = points2(0, i);
    double y2 = points2(1, i);
    // double height2 = cells2[i].height;

    // bool disabled = cells1[i].disabled;

    marker = visualization_msgs::Marker();
    // Set the frame ID and timestamp.
    marker.header.frame_id = "velo_link";
    marker.header.stamp = ros::Time(F1.GetTimestamp());
    // Set the namespace and id for this marker. This serves to create a unique ID Any marker sent with the same
    // namespace and id will overwrite the old one
    marker.ns = "line_between_matching_points";
    marker.id = i;
    // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker. only w component is used for LINE_STRIP markers
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker. only x component is used for LINE_STRIP markers
    marker.scale.x = 0.05;

    int h = 0;
    int s = 255;
    int v = 255;
    int r = 0, g = 0, b = 0;
    HSVtoRGB(h, s, v, r, g, b);

    marker.color.r = r / 255.0;
    marker.color.g = g / 255.0;
    marker.color.b = b / 255.0;
    marker.color.a = 0.5;
    // marker.color.a = 0.5 * (height1 + height2);

    // if disabled, make it yellow
    // if (disabled) {
    //   marker.color.r = 1;
    //   marker.color.g = 1;
    //   marker.color.b = 0;
    //   marker.color.a = 0.5;
    // }

    marker.lifetime = ros::Duration();

    geometry_msgs::Point p1, p2;
    p1.x = x1;
    p1.y = y1;
    p1.z = -1.73;
    p2.x = x2;
    p2.y = y2;
    p2.z = -1.73;
    marker.points.push_back(p1);
    marker.points.push_back(p2);

    marker_array.markers.push_back(marker);

    marker_pub.publish(marker_array);
  }
}

void VisualizeCentroid(ros::Publisher marker_pub, Eigen::Vector2d centroid, time_t timestamp, int color) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  marker = visualization_msgs::Marker();
  // Set the frame ID and timestamp.
  marker.header.frame_id = "velo_link";
  marker.header.stamp = ros::Time(timestamp);
  // Set the namespace and id for this marker. This serves to create a unique ID Any marker sent with the same
  // namespace and id will overwrite the old one
  if (color == 0) {
    marker.ns = "centroid_0";
  } else if (color == 1) {
    marker.ns = "centroid_1";
  } else if (color == 2) {
    marker.ns = "centroid_2";
  }
  marker.id = color;
  // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;
  // Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = centroid(0);
  marker.pose.position.y = centroid(1);
  marker.pose.position.z = -1.73;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  if (color == 0) {
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
  } else if (color == 1) {
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
  } else if (color == 2) {
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
  }

  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();

  marker_array.markers.push_back(marker);

  marker_pub.publish(marker_array);
}