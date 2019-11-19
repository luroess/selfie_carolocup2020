/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_avoiding_obstacles/road_obstacle_detector.h>

Road_obstacle_detector::Road_obstacle_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , timer_duration_(0.3)
    , is_time_calculated_for_overtake_(false)
    , received_road_markings_(false)
    , maximum_distance_to_obstacle_(0.5)
    , found_obstacles_in_a_row_(0)
{
  pnh_.param<bool>("visualization", visualization_, false);
  pnh_.param<float>("maximum_length_of_obstacle", maximum_length_of_obstacle_, 0.8);
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &Road_obstacle_detector::obstacle_callback, this);
  markings_sub_ = nh_.subscribe("/road_markings", 1, &Road_obstacle_detector::road_markings_callback, this);
  setpoint_pub_ = nh_.advertise<std_msgs::Float32>("/setpoint", 1);
  pnh_.param<float>("point_min_x", point_min_x_, 0.3);
  pnh_.param<float>("point_max_x", point_max_x_, 1.1);
  pnh_.param<float>("point_min_y", point_min_y_, -1.3);
  pnh_.param<float>("point_max_y", point_max_y_, 1.3);

  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/avoiding_obstacles", 1);
  }
  status_ = CLEAR;
  ROS_INFO("road_obstacle_detector initialized ");
}

Road_obstacle_detector::~Road_obstacle_detector() {}

void Road_obstacle_detector::obstacle_callback(const selfie_msgs::PolygonArray &msg)
{
  switch (status_)
  {
  case CLEAR:
    filter_boxes(msg);
    if (!filtered_boxes_.empty())
      if (nearest_box_in_front_of_car_->bottom_left.x <= maximum_distance_to_obstacle_)
      {
        if (found_obstacles_in_a_row_ >= 3)
        {

          change_lane(LEFT);
          status_ = OVERTAKING;
          speed_sub_ = nh_.subscribe("/speed", 1, &Road_obstacle_detector::calculate_overtake_time, this);
          found_obstacles_in_a_row_ = 0;
        } else if (found_obstacles_in_a_row_ == 0)
        {
          middle_of_last_obstacle_ =
              Point((nearest_box_in_front_of_car_->bottom_left.x + nearest_box_in_front_of_car_->bottom_right.x) / 2,
                    (nearest_box_in_front_of_car_->bottom_left.y + nearest_box_in_front_of_car_->bottom_right.y) / 2);
        } else
        {
          double xDiff, yDiff;
          xDiff = (nearest_box_in_front_of_car_->bottom_left.x + nearest_box_in_front_of_car_->bottom_right.x) / 2 -
                  middle_of_last_obstacle_.x;
          xDiff = abs(xDiff);
          yDiff = (nearest_box_in_front_of_car_->bottom_left.y + nearest_box_in_front_of_car_->bottom_right.y) / 2 -
                  middle_of_last_obstacle_.y;
          xDiff = abs(yDiff);
          double tolerance = 0.07;
          if (xDiff < tolerance && yDiff < tolerance)
          {
            found_obstacles_in_a_row_++;
            middle_of_last_obstacle_ =
                Point((nearest_box_in_front_of_car_->bottom_left.x + nearest_box_in_front_of_car_->bottom_right.x) / 2,
                      (nearest_box_in_front_of_car_->bottom_left.y + nearest_box_in_front_of_car_->bottom_right.y) / 2);
          }
        }
      }
    break;
  case OVERTAKING:
    if (time_left_ <= 0 && is_time_calculated_for_overtake_)
    {
      change_lane(RIGHT);
      status_ = CLEAR;
      is_time_calculated_for_overtake_ = false;
      timer_.stop();
    }
    break;
  default:
    ROS_ERROR("Wrong avoiding_obstacle action status");
  }
}

void Road_obstacle_detector::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  geometry_msgs::Polygon polygon;
  for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
  {
    polygon = msg.polygons[box_nr];
    int box_ok = 0;
    for (int a = 0; a < 4; ++a)
    {
      Point p(polygon.points[a]);

      if (is_on_right_lane(p) && p.check_position(point_min_x_, point_max_x_, point_min_y_, point_max_y_))
      {
        box_ok++;
        break;
      }
    }
    if (box_ok >= 2)
    {
      Box temp_box(polygon);
      filtered_boxes_.insert(filtered_boxes_.begin(), temp_box);
      if (temp_box.bottom_left.x > 0)
      {
        nearest_box_in_front_of_car_ = filtered_boxes_.begin();
      }
    }
  }
  if (visualization_)
  {
    visualizeBoxes();
  }
}

void Road_obstacle_detector::road_markings_callback(const selfie_msgs::RoadMarkings &msg)
{
  int size = msg.left_line.size();
  if (size != 3 && size != 4)
    ROS_ERROR("Invalid number of args in RoadMarkings");
  for (int i = 0; i < size; i++)
  {
    left_line_[i] = msg.left_line[i];
    center_line_[i] = msg.center_line[i];
    right_line_[i] = msg.right_line[i];
  }
  if (size == 3)
  {
    left_line_[3] = 0;
    center_line_[3] = 0;
    right_line_[3] = 0;
  }
  received_road_markings_ = true;
}

bool Road_obstacle_detector::is_on_right_lane(const Point &point)
{
  if (received_road_markings_ == false)
    return false;

  float right_value = right_line_[0] + point.x * right_line_[1] + point.x * point.x * right_line_[2] +
                      point.x * point.x * point.x * right_line_[3];
  float center_value = center_line_[0] + point.x * center_line_[1] + point.x * point.x * center_line_[2] +
                       point.x * point.x * point.x * center_line_[3];

  if (point.y > right_value && point.y < center_value)
    return true;
}

void Road_obstacle_detector::calculate_overtake_time(const std_msgs::Float32 &msg)
{
  time_left_ = (maximum_length_of_obstacle_ + maximum_distance_to_obstacle_) / msg.data;
  timer_ = nh_.createTimer(ros::Duration(timer_duration_), &Road_obstacle_detector::calculate_time, this);
  is_time_calculated_for_overtake_ = true;
  speed_sub_.shutdown();
}

void Road_obstacle_detector::change_lane(float lane)
{
  setpoint_value_.data = lane;
  setpoint_pub_.publish(setpoint_value_);
  if (lane == LEFT)
    ROS_INFO("Lane changed to left");
  else
    ROS_INFO("Lane changed to right");
}

void Road_obstacle_detector::visualizeBoxes()
{
  Box().visualizeList(filtered_boxes_, visualizer_, "boxes_on_lane", 0.9, 0.9, 0.9);
  nearest_box_in_front_of_car_->visualize(visualizer_, "nearest_box", 1, 0.1, 0.1);
}

void Road_obstacle_detector::calculate_time(const ros::TimerEvent &time) { time_left_ -= timer_duration_; }
