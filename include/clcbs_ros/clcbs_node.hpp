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
    nh.getParam("/clcbs_ros/num_waypoint", m_num_waypoint);
    nh.getParam("/clcbs_ros/num_agent", m_num_agent);
    m_car_pose = std::vector<State>(m_num_agent);
    for (size_t i = 0; i < m_num_agent; ++i) {
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

    int extra = 0;
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
    for(auto& pos: m_obs_pose) {
      obstacles.insert(Location(scalex(pos.first), scaley(pos.second)));
    }
    // init start states
    for(auto& pos: m_car_pose) {
      startStates.emplace_back(scalex(pos.x), scaley(pos.y), -pos.yaw);
    }

    int mkid = 0; //visualize

    int dimx = std::ceil(scalex(m_maxx));
    int dimy = std::ceil(scaley(m_maxy));
    bool success = true;
    std::vector<int> startTime;
    std::multimap<int, State> dynamic_obstacles;
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::cout << "x: " << dimx << ", y: " << dimy << ", agent: " << m_num_agent << ", task: "<< m_goal_pose.size() << ", waypoint: " << m_num_waypoint << std::endl;

    startTime.push_back(0);
    for (int i = 0; i < m_num_waypoint && success; i++) {
      // set constraints for this waypoint
      setCarParams(i);

      // get goals for current waypoint
      std::vector<State> mid_goals;
      std::vector<State> cur_goals;
      for (auto& waypoints : goals) {
        mid_goals.emplace_back(scalex(waypoints[i].x - 0.5 * std::cos(waypoints[i].yaw)), scaley(waypoints[i].y - 0.5 * std::sin(waypoints[i].yaw)), -waypoints[i].yaw);
        cur_goals.emplace_back(scalex(waypoints[i].x), scaley(waypoints[i].y), -waypoints[i].yaw);
      }

      std::unordered_set<Location> mid_obstacles(obstacles);
      std::multimap<int, State> mid_dynamic_obstacles(dynamic_obstacles);

      Environment mid_mapf(dimx, dimy, mid_obstacles, mid_dynamic_obstacles, mid_goals);
      CL_CBS<State, Action, Cost, Conflict, Constraints, Environment>
          mid_cbs(mid_mapf);

      Environment mapf(dimx, dimy, obstacles, dynamic_obstacles, cur_goals);
      CL_CBS<State, Action, Cost, Conflict, Constraints, Environment>
          cbs(mapf);
      
      std::vector<PlanResult<State, Action, Cost>> mid_solution;
      std::vector<PlanResult<State, Action, Cost>> sub_solution;
      success &= mid_cbs.search(startStates, mid_solution) && cbs.search(mid_goals, sub_solution);
      startStates.clear();
      int mid_makespan = 0;
      for (const auto& s : mid_solution) {
        mid_makespan = std::max<int64_t>(mid_makespan, s.cost);
      }
      int sub_makespan = 0;
      for (const auto& s : sub_solution) {
        State last = s.states.back().first;
        startStates.emplace_back(State(last.x, last.y, last.yaw));
        sub_makespan = std::max<int64_t>(sub_makespan, s.cost);
      }
      sub_makespan += mid_makespan;
      startTime.push_back(sub_makespan);
      for (int i = 0; i < mid_solution.size(); i++) {
        sub_solution[i].states.insert(sub_solution[i].states.begin(), mid_solution[i].states.begin(), mid_solution[i].states.end());
        sub_solution[i].actions.insert(sub_solution[i].actions.begin(), mid_solution[i].actions.begin(), mid_solution[i].actions.end());
        sub_solution[i].cost += mid_solution[i].cost;
        sub_solution[i].fmin += mid_solution[i].fmin;
      }
      solution.insert(solution.end(), sub_solution.begin(), sub_solution.end());
    }

    if (success) {
      std::cout << "planner success" << std::endl;
    } else {
      std::cout << "planner failed" << std::endl;
    }

    create_border(1, 1, 1, 0.1);

    if (success) {
      for (size_t a = 0; a < m_num_agent; ++a) {
        geometry_msgs::PoseArray plan;
        plan.header.stamp = ros::Time::now();
        plan.header.frame_id = "map";
        for(int t = 0; t < startTime.size() - 1; t++) {
          int j = t * m_num_agent + a;
          double prev_time = 0.0;
          for (size_t i = 0; i < solution[j].states.size(); i++) {
            geometry_msgs::Pose p;
                        
            double x = r_scalex(solution[j].states[i].first.x);
            double y = r_scaley(solution[j].states[i].first.y);
            double time = startTime[t] + solution[j].states[i].first.time;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, -solution[j].states[i].first.yaw);

            p.position.x = x;
            p.position.y = y;
            p.position.z = (time - prev_time) * (Constants::xyResolution / Constants::speed_limit) * 0.001;
            p.orientation = tf2::toMsg(quat);

            plan.poses.push_back(p);
            prev_time = time;
          }
          // visualize
          visualization_msgs::Marker pick;
          visualization_msgs::Marker drop;
          double marker_size = 0.25; 
          for (size_t i = 0; i < m_goal_pose.size(); i++) {
            if (fabs(scalex(m_goal_pose[i][1].x) - solution[j].states.back().first.x) < 0.001 &&
                fabs(scaley(m_goal_pose[i][1].y) - solution[j].states.back().first.y) < 0.001) {
              create_marker(&pick, &mkid, m_goal_pose[i][0].x, m_goal_pose[i][0].y, r_color(m_car_color[a]), g_color(m_car_color[a]), b_color(m_car_color[a]), marker_size, 0);
              create_marker(&drop, &mkid, m_goal_pose[i][1].x, m_goal_pose[i][1].y, r_color(m_car_color[a]), g_color(m_car_color[a]), b_color(m_car_color[a]), marker_size, 1);

              m_pub_marker[a].publish(pick);
              m_pub_marker[a].publish(drop);
              break;
            }
          } 
        }
        m_pub_plan[a].publish(plan);
        plan.poses.clear();
        std::cout << "publish plan for car " << a + 1 << std::endl;
      }
    }
    
    m_goal_pose.clear();
    m_obs_pose.clear();
    m_car_pose = std::vector<State>(m_num_agent);
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
    x = (x - m_minx) * m_scale;
    // Round close to 0 numbers to 0 (keeps points within boundaries)
    if (std::abs(x) < 0.01) {
      x = 0;
    } else if (std::abs(x - ((m_maxx - m_minx) * m_scale)) < 0.01) {
      // Hacky fix for segfaults in mushr_environment functions
      x -= 0.01;
    }
    return x;
  }

  double scaley(double y) {
    y = (y - m_miny) * m_scale;
    if (std::abs(y) < 0.01) {
      y = 0;
    } else if (std::abs(y - ((m_maxy - m_miny) * m_scale)) < 0.01) {
      y -= 0.01;
    }
    return y;
  }

  double r_scalex(double x) {
    return x / m_scale + m_minx;
  }

  double r_scaley(double y) {
    return y / m_scale + m_miny;
  }

  double yawFromQuat(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion quat;
    tf2::fromMsg(q, quat);
    double r, p, y;
    tf2::Matrix3x3(quat.normalized()).getRPY(r, p, y);
    return y;
  }

  void setCarParams(int waypoint) {
    float L, speed_limit, steer_limit, r, deltat, penaltyTurning, penaltyReversing;
    float penaltyCOD, mapResolution, carWidth, LF, LB, obsRadius;
    int constraintWaitTime;
    bool allow_reverse;

    std::string profile("default");
    nh.getParam("/clcbs_ros/profile" + std::to_string(waypoint), profile);
    std::string name = "/clcbs_ros/profiles/" + profile + "/";

    if (nh.getParam(name + "allow_reverse", allow_reverse)) {
      Constants::allow_reverse = allow_reverse;
    }
    if (nh.getParam(name + "L", L)) {
      Constants::L = L * m_scale;
    }
    if (nh.getParam(name + "speed_limit", speed_limit)) {
      Constants::speed_limit = speed_limit * m_scale;
    }
    if (nh.getParam(name + "steer_limit", steer_limit)) {
      Constants::steer_limit = steer_limit;
    }
    if (nh.getParam(name + "r", r)) {
      Constants::r = r * m_scale;
    } else {
      Constants::r = Constants::L / tanf(fabs(Constants::steer_limit));
    }
    if (nh.getParam(name + "deltat", deltat)) {
      Constants::deltat = deltat;
    } else {
      Constants::deltat = Constants::speed_limit / Constants::r;
    }
    if (nh.getParam(name + "penaltyTurning", penaltyTurning)) {
      Constants::penaltyTurning = penaltyTurning;
    }
    if (nh.getParam(name + "penaltyReversing", penaltyReversing)) {
      Constants::penaltyReversing = penaltyReversing;
    }
    if (nh.getParam(name + "penaltyCOD", penaltyCOD)) {
      Constants::penaltyCOD = penaltyCOD;
    }
    if (nh.getParam(name + "mapResolution", mapResolution)) {
      Constants::mapResolution = mapResolution;
    }
    if (nh.getParam(name + "carWidth", carWidth)) {
      Constants::carWidth = carWidth * m_scale;
    }
    if (nh.getParam(name + "LF", LF)) {
      Constants::LF = LF * m_scale;
    }
    if (nh.getParam(name + "LB", LB)) {
      Constants::LB = LB * m_scale;
    }
    if (nh.getParam(name + "obsRadius", obsRadius)) {
      Constants::obsRadius = obsRadius * m_scale;
    }
    if (nh.getParam(name + "constraintWaitTime", constraintWaitTime)) {
      Constants::constraintWaitTime = constraintWaitTime;
    }

    Constants::xyResolution = Constants::r * Constants::deltat;
    Constants::yawResolution = Constants::deltat;

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
  int m_num_agent;
  int m_num_waypoint;
  double m_scale;
  double m_minx;
  double m_miny;
  double m_maxx;
  double m_maxy;

};

} // namespace clcbs_ros
