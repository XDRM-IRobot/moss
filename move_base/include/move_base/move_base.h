/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING,            // 规划状态
    CONTROLLING,         // 控制状态
    CLEARING             // 清空状态
  };

  enum RecoveryTrigger
  {
    PLANNING_R,          // 规划恢复
    CONTROLLING_R,       // 控制恢复
    OSCILLATION_R        // 震荡恢复
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;   // tf转换的引用
      MoveBaseActionServer* as_;    // Actionlib服务器指针
      
      // 配置规划器plugin和实例化
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;       // 全局规划器loader
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner>  blp_loader_;       // 局部控制器loader
      pluginlib::ClassLoader<nav_core::RecoveryBehavior>  recovery_loader_;  // 恢复器loader

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;   // 全局规划 共享指针
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;         // 局部控制 共享指针

      // 全局规划器线程
      boost::thread*                planner_thread_;      // 线程指针
      boost::recursive_mutex        planner_mutex_;       // 互斥锁
      boost::condition_variable_any planner_cond_;        // 条件变量
      geometry_msgs::PoseStamped    planner_goal_;        // 全局规划 目标点
      bool                          runPlanner_;          // 是否需要执行规划
      bool                          new_global_plan_;     // 是否得到新的规划

      // 规划器路径向量的指针
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;    // 全局规划
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;     // 最近的规划
      std::vector<geometry_msgs::PoseStamped>* controller_plan_; // 局部控制

      // 规划器和控制器的频率和时间
      double    planner_frequency_,         // 全局规划频率
                controller_frequency_;      // 局部控制频率
      bool      p_freq_change_,             // 全局规划频率更改标记
                c_freq_change_;             // 局部控制频率更改标记
      ros::Time last_valid_plan_,           // 上一次有效全局规划的时间
                last_valid_control_,        // 上一次有效控制的时间
                last_oscillation_reset_;    // 上一次震荡重置时间
      double    planner_patience_,          // 等待有效全局规划容忍时间
                controller_patience_;       // 等待有效局部控制容忍时间

      // 恢复行为
      bool         recovery_behavior_enabled_;                                           // 恢复行为使能
      unsigned int recovery_index_;                                                      // 恢复行为等待的序号
      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;   // 恢复行为的向量

      // 震荡行为
      double oscillation_timeout_,   // 震荡超时
             oscillation_distance_;  // 震荡距离

      // 代价图的指针
      costmap_2d::Costmap2DROS  *planner_costmap_ros_,     // 全局
                                *controller_costmap_ros_;  // 局部
      bool                       shutdown_costmaps_;       // 代价图是否激活

      // 动态调参服务
      boost::recursive_mutex configuration_mutex_;                    // 互斥锁
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;  // 指针
      

      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_;

      std::string robot_base_frame_, global_frame_;
      tf::Stamped<tf::Pose> global_pose_;
      double inscribed_radius_, circumscribed_radius_;
      
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool clearing_rotation_allowed_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;
      geometry_msgs::PoseStamped oscillation_pose_;
      
  };
};
#endif

