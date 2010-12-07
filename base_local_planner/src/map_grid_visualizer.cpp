/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Eric Perko
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
 *   * Neither the name of Eric Perko nor the names of its
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
 *********************************************************************/
#include <base_local_planner/map_grid_visualizer.h>
#include <base_local_planner/map_cell.h>
#include <vector>

namespace base_local_planner {
  MapGridVisualizer::MapGridVisualizer(const std::string &name, const costmap_2d::Costmap2D * costmap, const MapGrid *grid) {
    name_ = name;
    costmap_p_ = costmap;
    grid_p_ = grid;

    ns_nh_ = ros::NodeHandle("~/" + name_);
    ns_nh_.param("path_distance_bias", pdist_gain_, 0.6);
    ns_nh_.param("goal_distance_bias", gdist_gain_, 0.8);
    ns_nh_.param("occdist_scale", ocost_gain_, 0.01);
    ns_nh_.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    ns_nh_.param("global_frame_id", frame_id_, std::string("odom"));

    cost_cloud_.header.frame_id = frame_id_;
    pub_.advertise(ns_nh_, "cost_cloud", 1);
  }

  void MapGridVisualizer::publishCostCloud() {
    if(publish_cost_grid_pc_) {
      std::vector<MapCell> map_cells = grid_p_->map_;
      double z_coord = 0.0;
      double x_coord, y_coord;
      std::vector<MapCell>::const_iterator it;
      MapGridCostPoint pt;
      cost_cloud_.points.clear();
      cost_cloud_.header.stamp = ros::Time::now();
      unsigned char occ_cost;
      for (it = map_cells.begin(); it < map_cells.end(); it++) {
        if(it->within_robot) {
          continue;
        }
        occ_cost = costmap_p_->getCost(it->cx, it->cy);
        if(it->path_dist >= map_cells.size() || it->goal_dist >= map_cells.size() || occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          continue;
        }
        costmap_p_->mapToWorld(it->cx, it->cy, x_coord, y_coord);
        pt.x = x_coord;
        pt.y = y_coord;
        pt.z = z_coord;
        pt.path_cost = it->path_dist;
        pt.goal_cost = it->goal_dist;
        pt.occ_cost = occ_cost;
        double total_cost = it->path_dist * pdist_gain_ + it->goal_dist * gdist_gain_ + occ_cost * ocost_gain_;
        pt.total_cost = total_cost;
        cost_cloud_.push_back(pt);
      }
      pub_.publish(cost_cloud_);
    }
  }
};
