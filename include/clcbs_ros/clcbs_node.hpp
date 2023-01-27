#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <CL-CBS/include/cl_cbs.hpp>

#include "clcbs_ros/GoalPoseArray.h"
#include "clcbs_ros/mushr_environment.hpp"

namespace clcbs_ros {

using clcbs_ros::Environment;
using libMultiRobotPlanning::CL_CBS;

class CLCBSNode {
public:
  CLCBSNode(ros::NodeHandle &_nh)
    : nh(_nh),
      m_planning(false),
      m_ini_obs(false),
      m_ini_goal(false) {
    size_t i;
    for (i = 0; nh.hasParam("/clcbs_ros/car" + std::to_string(i + 1) + "/name"); ++i) {
      std::string name, color;
      nh.getParam("/clcbs_ros/car" + std::to_string(i + 1) + "/name", name);
      nh.getParam("/clcbs_ros/car" + std::to_string(i + 1) + "/color", color);
      std::cout << name << " " << color << std::endl;
      m_car_name.push_back(name);
      m_car_color.push_back(color);
      m_sub_car_pose.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
        name + "/init_pose",
        10,
        boost::bind(&CLCBSNode::CarPoseCallback, this, _1, i)
      ));
      m_pub_plan.push_back(nh.advertise<geometry_msgs::PoseArray>(
        name + "/waypoints",
        10
      ));
      m_pub_marker.push_back(nh.advertise<visualization_msgs::Marker>(
        name + "/marker",
        10
      ));
    }
    m_max_agent = i;
    m_car_pose = std::vector<State>(m_max_agent);
    m_pub_border = nh.advertise<visualization_msgs::Marker>(
        "/clcbs_ros/border",
        10
      );
    m_sub_obs_pose = nh.subscribe("/clcbs_ros/obstacles",
            10,
            &CLCBSNode::ObsPoseCallback,
            this);
    m_sub_goal = nh.subscribe("/clcbs_ros/goals",
            10,
            &CLCBSNode::GoalCallback,
            this);
    ros::Duration(1).sleep();
  }

private:
  void CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, size_t i) {
    std::cout << "get car " << i << std::endl;
    m_assigned.insert(i);
    m_car_pose[i] = State(msg->pose.position.x, msg->pose.position.y, yawFromQuat(msg->pose.orientation));
    if (isReady()) {
        solve();
    }
  }

  void ObsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    std::cout << "get obs " << std::endl;
    m_ini_obs = true;
    for (auto &pose : msg->poses) {
      m_obs_pose.emplace_back(pose.position.x, pose.position.y);
    }
    if (isReady()) {
      solve();
    }
  }

  void GoalCallback(const clcbs_ros::GoalPoseArray::ConstPtr& msg) {
    std::cout << "get goal" << std::endl;
    m_ini_goal = true;
    m_num_agent = msg->num_agent;
    m_num_waypoint = msg->num_waypoint;
    m_scale = msg->scale;
    m_minx = msg->minx;
    m_miny = msg->miny;
    m_maxx = msg->maxx;
    m_maxy = msg->maxy;
    for (auto &goal : msg->goals) {
      m_goal_pose.emplace_back();
      for (auto &pose : goal.poses) {
        m_goal_pose.back().points.emplace_back(pose.position.x, pose.position.y, yawFromQuat(pose.orientation));
      }
    }
    if (isReady()) {
        solve();
    }
  }

  void solve() {
    m_planning = true;
    std::cout << "start planning" << std::endl;
    std::unordered_set<Location> obstacles;
    std::vector<State> startStates;
    std::vector<Waypoints> goals;

    // init goals
    for (auto& goal: m_goal_pose) {
      std::vector<State> ls;
      for (auto& waypoint: goal.points) {
        ls.emplace_back(waypoint.x, waypoint.y, waypoint.yaw);
      }
      goals.emplace_back(ls);
    }
    for (auto & g: goals) {
      std::cout << g << std::endl;
    }
    // init obstacles
    for (auto& pos: m_obs_pose) {
      obstacles.insert(Location(scalex(pos.first), scaley(pos.second)));
    }
    // init start states
    for (int i = 0; i < m_num_agent; i++) {
      State pos = m_car_pose[i];
      startStates.emplace_back(scalex(pos.x), scaley(pos.y), -pos.yaw);
    }

    int mkid = 0; //visualize

    int dimx = std::ceil(scalex(m_maxx));
    int dimy = std::ceil(scaley(m_maxy));
    bool success = true;
    std::shared_ptr<bool> within_time = std::make_shared<bool>(true);
    std::multimap<int, State> dynamic_obstacles;
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::cout << "x: " << dimx << ", y: " << dimy << ", agent: " << m_num_agent << ", task: "<< m_goal_pose.size() << ", waypoint: " << m_num_waypoint << std::endl;

    for (int i = 0; i < m_num_waypoint && success; i++) {
      // set constraints for this waypoint
      setCarParams(i);

      // set offset if pushing
      double offset = 0;

      std::string profile("default");
      std::string next_profile("default");
      nh.getParam("/clcbs_ros/profile" + std::to_string(i), profile);
      nh.getParam("/clcbs_ros/profile" + std::to_string(i + 1), next_profile);

      if (profile == "pushing" || next_profile == "pushing") {
        double LF, default_LF;
        nh.getParam("/clcbs_ros/profiles/pushing/LF", LF);
        nh.getParam("/clcbs_ros/profiles/default/LF", default_LF);
        offset = default_LF + (LF - default_LF) / 2;
      }

      // get goals for current waypoint
      std::vector<State> mid_goals;
      std::vector<State> cur_goals;
      double state_eps = 0.01;
      for (int j = 0; j < goals.size(); j++) {
        auto waypoints = goals[j];
        double x = waypoints[i].x;
        double y = waypoints[i].y;
        double yaw = waypoints[i].yaw;

        if (std::abs(scalex(x) - startStates[j].x) > state_eps || std::abs(scaley(y) - startStates[j].y) > state_eps || std::abs(yaw + startStates[j].yaw) > state_eps) {
          x -= offset * std::cos(yaw);
          y -= offset * std::sin(yaw);
        }

        if (std::abs(scalex(x) - startStates[j].x) > state_eps || std::abs(scaley(y) - startStates[j].y) > state_eps || std::abs(yaw + startStates[j].yaw) > state_eps) {
          mid_goals.emplace_back(scalex(x - 0.5 * std::cos(yaw)), scaley(y - 0.5 * std::sin(yaw)), -yaw);
          cur_goals.emplace_back(scalex(x), scaley(y), -yaw);
        } else {
          mid_goals.emplace_back(scalex(x), scaley(y), -yaw);
          cur_goals.emplace_back(scalex(x), scaley(y), -yaw);
        }
      }

      Environment mid_mapf(dimx, dimy, obstacles, dynamic_obstacles, mid_goals, within_time);
      CL_CBS<State, Action, Cost, Conflict, Constraints, Environment>
          mid_cbs(mid_mapf);

      Environment mapf(dimx, dimy, obstacles, dynamic_obstacles, cur_goals, within_time);
      CL_CBS<State, Action, Cost, Conflict, Constraints, Environment>
          cbs(mapf);

      // Plan to 0.5m behind goal to allow for a straight path at the end, then plan straight path to goal
      std::vector<PlanResult<State, Action, Cost>> mid_solution;
      std::vector<PlanResult<State, Action, Cost>> sub_solution;

      bool buffer_success = false;
      while (!buffer_success && Constants::space_buffer >= 0.0f) {
        Constants::carWidth += 2 * Constants::space_buffer;
        double old_LF = Constants::LF;
        double old_LB = Constants::LB;
        Constants::LF += Constants::space_buffer * (old_LF + old_LB) / 2;
        Constants::LB += Constants::space_buffer * (old_LF + old_LB) / 2;

        *within_time = true;
        mid_solution.clear();
        sub_solution.clear();
        std::cout << "Trying with buffer " << Constants::space_buffer << std::endl;
        buffer_success = mid_cbs.search(startStates, mid_solution) && cbs.search(mid_goals, sub_solution) && *within_time;

        Constants::carWidth -= 2 * Constants::space_buffer;
        Constants::LF = old_LF;
        Constants::LB = old_LB;

        if (!buffer_success) {
          Constants::space_buffer -= 0.1f;
          if (std::abs(Constants::space_buffer) < 0.01f) {
            Constants::space_buffer = 0.0f;
          }
        }
      }
      success &= buffer_success;
      startStates.clear();

      // Calculate makespan of first part of the plan
      int mid_makespan = 0;
      for (const auto& s : mid_solution) {
        mid_makespan = std::max<int64_t>(mid_makespan, s.states.back().first.time);
      }

      for (auto& s : sub_solution) {
        // Add mid_makespan to all times in the second part of the plan
        for (std::pair<State, double>& pair : s.states) {
          pair.first.time += mid_makespan;
        }
        State last = s.states.back().first;
        startStates.emplace_back(last.x, last.y, last.yaw);
      }

      // Add pushed blocks as obstacles if they're being dropped off
      if (profile == "pushing" && next_profile != "pushing" && i + 1 < m_num_waypoint) {
        for (int j = 0; j < goals.size(); j++) {
          auto waypoints = goals[j];
          if (std::abs(scalex(waypoints[i + 1].x - offset * std::cos(waypoints[i + 1].yaw)) - startStates[j].x) > state_eps || std::abs(scaley(waypoints[i + 1].y - offset * std::sin(waypoints[i + 1].yaw)) - startStates[j].y) > state_eps || std::abs(waypoints[i + 1].yaw + startStates[j].yaw) > state_eps) {
            obstacles.emplace(scalex(waypoints[i].x), scaley(waypoints[i].y));
          }
        }
      }

      // Combine the plan from start to 0.5m behind goal and the plan from there to goal
      for (int i = 0; i < mid_solution.size(); i++) {
        sub_solution[i].states.insert(sub_solution[i].states.begin(), mid_solution[i].states.begin(), mid_solution[i].states.end());
        sub_solution[i].actions.insert(sub_solution[i].actions.begin(), mid_solution[i].actions.begin(), mid_solution[i].actions.end());
        sub_solution[i].cost += mid_solution[i].cost;
        sub_solution[i].fmin += mid_solution[i].fmin;
      }
      solution.insert(solution.end(), sub_solution.begin(), sub_solution.end());
    }

    if (success) {
      std::cout << "planner success with buffer " << Constants::space_buffer << std::endl;
    } else {
      std::cout << "planner failed" << std::endl;
    }

    create_border(1, 1, 1, 0.02);

    if (success) {
      for (size_t a = 0; a < m_num_agent; ++a) {
        geometry_msgs::PoseArray plan;
        plan.header.stamp = ros::Time::now();
        plan.header.frame_id = "map";
        for(int t = 0; t < m_num_waypoint; t++) {
          int j = t * m_num_agent + a;
          double prev_time = 0.0;
          for (size_t i = 0; i < solution[j].states.size(); i++) {
            geometry_msgs::Pose p;

            double x = r_scalex(solution[j].states[i].first.x);
            double y = r_scaley(solution[j].states[i].first.y);
            double time = solution[j].states[i].first.time;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, -solution[j].states[i].first.yaw);

            p.position.x = x;
            p.position.y = y;
            p.position.z = (time - prev_time) * 0.001 / m_scale;
            p.orientation = tf2::toMsg(quat);

            if ((solution[j].actions[i].first >= 3 && solution[j].actions[i].first < 6) || (solution[j].actions[i].first == 6 && plan.poses.back().position.z < 0)) {
              p.position.z *= -1;
            }

            plan.poses.push_back(p);
            prev_time = time;
          }
        }

        // publish waypoint markers
        double marker_size = 0.25;
        for (int i = 0; i < m_num_waypoint; i++) {
          visualization_msgs::Marker marker;
          create_marker(&marker, &mkid, m_goal_pose[a][i].x, m_goal_pose[a][i].y, r_color(m_car_color[a]), g_color(m_car_color[a]), b_color(m_car_color[a]), marker_size, i);
          m_pub_marker[a].publish(marker);
        }

        m_pub_plan[a].publish(plan);
        plan.poses.clear();
        std::cout << "publish plan for car " << a + 1 << std::endl;
      }
    }

    m_goal_pose.clear();
    m_obs_pose.clear();
    m_car_pose = std::vector<State>(m_max_agent);
    m_ini_obs = false;
    m_ini_goal = false;
    m_assigned.clear();
    m_planning = false;
  }

  void create_border(double r, double b, double g, double thickness) {
    visualization_msgs::Marker marker;

    geometry_msgs::Point p;
    p.x = m_minx; p.y = m_miny; p.z = 0.0;
    marker.points.push_back(p);
    p.x = m_minx; p.y = m_maxy; p.z = 0.0;
    marker.points.push_back(p);
    p.x = m_maxx; p.y = m_maxy; p.z = 0.0;
    marker.points.push_back(p);
    p.x = m_maxx; p.y = m_miny; p.z = 0.0;
    marker.points.push_back(p);
    p.x = m_minx; p.y = m_miny; p.z = 0.0;
    marker.points.push_back(p);
    p.x = m_minx; p.y = m_maxy; p.z = 0.0;
    marker.points.push_back(p);

    marker.color.r = r;
    marker.color.g = b;
    marker.color.b = g;
    marker.color.a = 1.0;

    marker.scale.x = thickness;
    marker.scale.y = thickness;
    marker.scale.z = thickness;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    m_pub_border.publish(marker);
  }

  void create_marker(visualization_msgs::Marker* marker, int* mkid, double x, double y, double r, double g, double b, double size, int last) {
    marker->pose.position.x = x;
    marker->pose.position.y = y;
    marker->pose.position.z = 0;

    marker->pose.orientation.x = 0;
    marker->pose.orientation.y = 0;
    marker->pose.orientation.z = 0;
    marker->pose.orientation.w = 1;

    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    marker->color.a = 1.0;

    marker->scale.x = size;
    marker->scale.y = size;
    marker->scale.z = size;

    marker->header.frame_id = "map";
    marker->header.stamp = ros::Time();
    marker->id = (*mkid)++;

    marker->type = last ? visualization_msgs::Marker::SPHERE : visualization_msgs::Marker::CUBE;
    marker->action = visualization_msgs::Marker::ADD;
  }

  double r_color(std::string hex) {
    int hexValue = std::stoi(hex, 0, 16);
    return ((hexValue >> 16) & 0xFF) / 255.0;
  }

  double g_color(std::string hex) {
    int hexValue = std::stoi(hex, 0, 16);
    return  ((hexValue >> 8) & 0xFF) / 255.0;
  }

  double b_color(std::string hex) {
    int hexValue = std::stoi(hex, 0, 16);
    return ((hexValue) & 0xFF) / 255.0;
  }

  bool isReady() {
    return m_assigned.size() == m_num_agent && m_ini_obs && m_ini_goal && !m_planning;
  }

  double scalex(double x) {
    x -= m_minx;
    // Round close to 0 numbers to 0 (keeps points within boundaries)
    if (std::abs(x) < 0.01) {
      x = 0.01;
    } else if (std::abs(x - (m_maxx - m_minx)) < 0.01) {
      // Hacky fix for segfaults in mushr_environment functions
      x -= 0.01;
    }
    return x;
  }

  double scaley(double y) {
    y -= m_miny;
    if (std::abs(y) < 0.01) {
      y = 0.01;
    } else if (std::abs(y - (m_maxy - m_miny)) < 0.01) {
      y -= 0.01;
    }
    return y;
  }

  double r_scalex(double x) {
    return x + m_minx;
  }

  double r_scaley(double y) {
    return y + m_miny;
  }

  double yawFromQuat(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion quat;
    tf2::fromMsg(q, quat);
    double r, p, y;
    tf2::Matrix3x3(quat.normalized()).getRPY(r, p, y);
    return y;
  }

  void setCarParams(int waypoint) {
    float L, speed_limit, steer_limit, deltat, penaltyTurning, penaltyHardTurning, penaltyReversing;
    float penaltyCOD, heuristicWeight, mapResolution, carWidth, LF, LB, obsRadius, space_buffer;
    int constraintWaitTime;
    bool allow_reverse;

    std::string profile("default");
    nh.getParam("/clcbs_ros/profile" + std::to_string(waypoint), profile);
    std::string name = "/clcbs_ros/profiles/" + profile + "/";
    std::string def = "/clcbs_ros/profiles/default/";

    if (!nh.getParam(name + "allow_reverse", allow_reverse)) {
      nh.getParam(def + "allow_reverse", allow_reverse);
    }
    if (!nh.getParam(name + "L", L)) {
      nh.getParam(def + "L", L);
    }
    if (!nh.getParam(name + "speed_limit", speed_limit)) {
      nh.getParam(def + "speed_limit", speed_limit);
    }
    if (!nh.getParam(name + "steer_limit", steer_limit)) {
      nh.getParam(def + "steer_limit", steer_limit);
    }
    if (!nh.getParam(name + "penaltyTurning", penaltyTurning)) {
      nh.getParam(def + "penaltyTurning", penaltyTurning);
    }
    if (!nh.getParam(name + "penaltyHardTurning", penaltyHardTurning)) {
      nh.getParam(def + "penaltyHardTurning", penaltyHardTurning);
    }
    if (!nh.getParam(name + "penaltyReversing", penaltyReversing)) {
      nh.getParam(def + "penaltyReversing", penaltyReversing);
    }
    if (!nh.getParam(name + "penaltyCOD", penaltyCOD)) {
      nh.getParam(def + "penaltyCOD", penaltyCOD);
    }
    if (!nh.getParam(name + "heuristicWeight", heuristicWeight)) {
      nh.getParam(def + "heuristicWeight", heuristicWeight);
    }
    if (!nh.getParam(name + "mapResolution", mapResolution)) {
      nh.getParam(def + "mapResolution", mapResolution);
    }
    if (!nh.getParam(name + "carWidth", carWidth)) {
      nh.getParam(def + "carWidth", carWidth);
    }
    if (!nh.getParam(name + "LF", LF)) {
      nh.getParam(def + "LF", LF);
    }
    if (!nh.getParam(name + "LB", LB)) {
      nh.getParam(def + "LB", LB);
    }
    if (!nh.getParam(name + "obsRadius", obsRadius)) {
      nh.getParam(def + "obsRadius", obsRadius);
    }
    if (!nh.getParam(name + "constraintWaitTime", constraintWaitTime)) {
      nh.getParam(def + "constraintWaitTime", constraintWaitTime);
    }
    if (!nh.getParam(name + "space_buffer", space_buffer)) {
      nh.getParam(def + "space_buffer", space_buffer);
    }

    Constants::allow_reverse = allow_reverse;

    Constants::L = L;
    Constants::speed_limit = speed_limit / m_scale;
    Constants::steer_limit = steer_limit;

    Constants::r = Constants::L / tanf(fabs(Constants::steer_limit));
    Constants::deltat = Constants::speed_limit / Constants::r;

    Constants::penaltyTurning = penaltyTurning;
    Constants::penaltyHardTurning = penaltyHardTurning;
    Constants::penaltyReversing = penaltyReversing;
    Constants::penaltyCOD = penaltyCOD;
    Constants::heuristicWeight = heuristicWeight;
    Constants::mapResolution = mapResolution;
    Constants::xyResolution = Constants::r * Constants::deltat;
    Constants::yawResolution = Constants::deltat;

    Constants::carWidth = carWidth;
    Constants::LF = LF;
    Constants::LB = LB;
    Constants::obsRadius = obsRadius;
    Constants::constraintWaitTime = constraintWaitTime;
    Constants::space_buffer = std::max(space_buffer, 0.0f);

    Constants::dyaw = {0, Constants::deltat, -Constants::deltat, 0, -Constants::deltat, Constants::deltat};
    Constants::dx = {Constants::r * Constants::deltat, Constants::r *sin(Constants::deltat),  Constants::r *sin(Constants::deltat),
                              -Constants::r *Constants::deltat, -Constants::r *sin(Constants::deltat), -Constants::r *sin(Constants::deltat)};
    Constants::dy = {0, -Constants::r *(1 - cos(Constants::deltat)), Constants::r *(1 - cos(Constants::deltat)),
                              0, -Constants::r *(1 - cos(Constants::deltat)), Constants::r *(1 - cos(Constants::deltat))};
  }

  ros::NodeHandle nh;
  std::vector<ros::Subscriber> m_sub_car_pose;
  ros::Subscriber m_sub_obs_pose;
  ros::Subscriber m_sub_goal;
  ros::Publisher m_pub_border;
  std::vector<ros::Publisher> m_pub_plan;
  std::vector<ros::Publisher> m_pub_marker;
  std::vector<std::string> m_car_name;
  std::vector<std::string> m_car_color;
  std::vector<State> m_car_pose;
  std::vector<std::pair<double, double>> m_obs_pose;
  std::vector<Waypoints> m_goal_pose;
  std::set<size_t> m_assigned;
  bool m_planning;
  bool m_ini_obs;
  bool m_ini_goal;
  int m_max_agent;
  int m_num_agent;
  int m_num_waypoint;
  double m_scale;
  double m_minx;
  double m_miny;
  double m_maxx;
  double m_maxy;

};

} // namespace clcbs_ros
