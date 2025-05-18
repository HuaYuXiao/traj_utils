#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
//#include <bspline_opt/uniform_bspline.h>
#include <iostream>
//#include <bspline_opt/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
#include <unordered_map>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <bspline/non_uniform_bspline.h>
#include <path_searching/topo_prm.h>
#include <plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>

using std::vector, std::map, std::unordered_map;

/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/
namespace fast_planner {
    class PlanningVisualization {
    private:
        enum TRAJECTORY_PLANNING_ID {
            GOAL = 1,
            PATH = 200,
            BSPLINE = 300,
            BSPLINE_CTRL_PT = 400,
            POLY_TRAJ = 500
        };

        enum TOPOLOGICAL_PATH_PLANNING_ID {
            GRAPH_NODE = 1,
            GRAPH_EDGE = 100,
            RAW_PATH = 200,
            FILTERED_PATH = 300,
            SELECT_PATH = 400
        };

        /* data */
        /* visib_pub is seperated from previous ones for different info */
        ros::NodeHandle node;
        ros::Publisher traj_pub_;      // 0
        ros::Publisher topo_pub_;      // 1
        ros::Publisher predict_pub_;   // 2
        ros::Publisher visib_pub_;     // 3, visibility constraints
        ros::Publisher frontier_pub_;  // 4, frontier searching
        ros::Publisher yaw_pub_;       // 5, yaw trajectory
        vector<ros::Publisher> pubs_;  //

        int last_topo_path1_num_;
        int last_topo_path2_num_;
        int last_bspline_phase1_num_;
        int last_bspline_phase2_num_;
        int last_frontier_num_;

    public:
        PlanningVisualization(/* args */) {}
        ~PlanningVisualization() {}
        PlanningVisualization(ros::NodeHandle& nh);

        // draw basic shapes
        void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                               const Eigen::Vector4d& color, int id, int pub_id = 0);
        void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                             const Eigen::Vector4d& color, int id, int pub_id = 0);
        void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                             double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);

        // draw a piece-wise straight line path
        void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                               const Eigen::Vector4d& color, int id = 0);

        // draw a polynomial trajectory
        void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
                                int id = 0);

        // draw a bspline trajectory
        void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                         bool show_ctrl_pts = false, double size2 = 0.1,
                         const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                         int id2 = 0);

        // draw a set of bspline trajectories generated in different phases
        void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
        void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

        // draw topological graph and paths
        void drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
                           const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
                           const Eigen::Vector4d& color3, int id = 0);

        void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);
        void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);

        void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
        void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

        Eigen::Vector4d getColor(double h, double alpha = 1.0);

        typedef std::shared_ptr<PlanningVisualization> Ptr;

        // SECTION developing
        void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);
        void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);
    };
}  // namespace fast_planner

namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher intermediate_state_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace ego_planner

/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of PlanningVisualization class, which 
 *                   implements the visualization tools for FC-Planner.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or 
 *                   modify it under the terms of the GNU Lesser General Public 
 *                   License as published by the Free Software Foundation, 
 *                   either version 3 of the License, or (at your option) any 
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/
namespace predrecon {

class PlanningVisualization {
private:

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;
  ros::Publisher traj_pub_;       // 0
  ros::Publisher topo_pub_;       // 1
  ros::Publisher predict_pub_;    // 2
  ros::Publisher visib_pub_;      // 3, visibility constraints
  ros::Publisher frontier_pub_;   // 4, frontier searching
  ros::Publisher yaw_pub_;        // 5, yaw trajectory
  ros::Publisher viewpoint_pub_;  // 6, viewpoint planning
  // GlobalPlanner
  ros::Publisher pred_pub_;       // 7, prediction point cloud
  ros::Publisher localReg_pub_;   // 8. local region point cloud
  ros::Publisher global_pub_;     // 9, global planning tour
  ros::Publisher vpg_pub_;        // 10, global planning viewpoints
  ros::Publisher internal_pub_;   // 11, global prediction internal
  ros::Publisher global_dir_pub_; // 12, global direction
  ros::Publisher global_c_pub_;   // 13, global cluster
  ros::Publisher global_n_pub_;   // 14, global normals
  // LocalPlanner
  ros::Publisher local_pub_;      // 15, local planning tour
  ros::Publisher localob_pub_;    // 15_1, local planning tour
  ros::Publisher localVP_pub_;    // 16, local planning viewpoints
  // ROSA
  ros::Publisher pcloud_pub_; 
  ros::Publisher mesh_pub_; 
  ros::Publisher normal_pub_;
  ros::Publisher rosa_orientation_pub_;
  ros::Publisher drosa_pub_;
  ros::Publisher le_pts_pub_;
  ros::Publisher le_lines_pub_;
  ros::Publisher rr_pts_pub_;
  ros::Publisher rr_lines_pub_;
  ros::Publisher decomp_pub_;
  ros::Publisher branch_start_end_pub_;
  ros::Publisher branch_dir_pub_;
  ros::Publisher cut_plane_pub_;
  ros::Publisher cut_pt_pub_;
  ros::Publisher sub_space_pub_;
  ros::Publisher sub_endpts_pub_;
  ros::Publisher vertex_ID_pub_;
  // ROSA Intermediate Vis
  ros::Publisher checkPoint_pub_;
  ros::Publisher checkNeigh_pub_;
  ros::Publisher checkCPdir_pub_;
  ros::Publisher checkRP_pub_;
  ros::Publisher checkCPpts_pub_;
  ros::Publisher checkCPptsCluster_pub_;
  ros::Publisher checkBranch_pub_;
  ros::Publisher checkAdj_pub_;
  // ROSA Opt
  ros::Publisher optArea_pub_;
  // HCOPP
  ros::Publisher init_vps_pub_;
  ros::Publisher sub_vps_hull_pub_;
  ros::Publisher before_opt_vp_pub_;
  ros::Publisher after_opt_vp_pub_;
  ros::Publisher hcopp_viewpoints_pub_;
  ros::Publisher hcopp_occ_pub_;
  ros::Publisher hcopp_internal_pub_;
  ros::Publisher hcopp_fov_pub_;
  ros::Publisher hcopp_uncovered_pub_;
  ros::Publisher hcopp_validvp_pub_;
  ros::Publisher hcopp_correctnormal_pub_;
  ros::Publisher hcopp_sub_finalvps_pub_;
  ros::Publisher hcopp_vps_drone_pub_;
  ros::Publisher hcopp_globalseq_pub_;
  ros::Publisher hcopp_globalboundary_pub_;
  ros::Publisher hcopp_local_path_pub_;
  ros::Publisher hcopp_full_path_pub_;
  ros::Publisher fullatsp_full_path_pub_;
  ros::Publisher fullgdcpca_full_path_pub_;
  ros::Publisher pca_vec_pub_;
  ros::Publisher cylinder_pub_;
  ros::Publisher posi_traj_pub_, pitch_traj_pub_, yaw_traj_pub_;
  ros::Publisher jointSphere_pub_;
  ros::Publisher hcoppYaw_pub_;
  ros::Publisher pathVisible_pub_;
  // Updates Visualization
  ros::Publisher currentPose_pub_;
  ros::Publisher currentVoxels_pub_;
  // Flight
  ros::Publisher drawFoV_pub_;
  ros::Publisher drone_pub_;
  ros::Publisher traveltraj_pub_;
  nav_msgs::Path path_msg;
  ros::Publisher visible_pub_;
  std::string droneMesh;

  vector<ros::Publisher> pubs_;  

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {
  }
  ~PlanningVisualization() {
  }
  PlanningVisualization(ros::NodeHandle& nh);

  /* VISUALIZATION */
  void publishSurface(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);
  void publishVisCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
  void publishMesh(std::string& mesh);
  void publishSurfaceNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publishROSAOrientation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publish_dROSA(const pcl::PointCloud<pcl::PointXYZ>& input_cloud);
  void publish_lineextract_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj);
  void publish_recenter_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj, Eigen::MatrixXd& realVertices);
  void publish_decomposition(Eigen::MatrixXd& nodes, vector<vector<int>>& branches, vector<Eigen::Vector3d>& dirs, vector<Eigen::Vector3d>& centroids);
  void publishCutPlane(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, Eigen::Vector3d& p, Eigen::Vector3d& v);
  void publishSubSpace(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& space);
  void publishSubEndpts(map<int, vector<Eigen::Vector3d>>& endpts);
  void publishSegViewpoints(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_vps);
  void publishOccupied(pcl::PointCloud<pcl::PointXYZ>& occupied);
  void publishInternal(pcl::PointCloud<pcl::PointXYZ>& internal);
  void publishFOV(const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2);
  void publishUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered);
  void publishRevisedNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals);
  void publishFinalFOV(map<int, vector<vector<Eigen::Vector3d>>>& list1, map<int, vector<vector<Eigen::Vector3d>>>& list2, map<int, vector<double>>& yaws);
  void publishGlobalSeq(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& sub_rep, vector<int>& global_seq);
  void publishGlobalBoundary(Eigen::Vector3d& start_, map<int, vector<int>>& boundary_id_, map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& global_seq);
  void publishLocalPath(map<int, vector<Eigen::VectorXd>>& sub_paths_);
  void publishHCOPPPath(vector<Eigen::VectorXd>& fullpath_);
  void publishFullATSPPath(vector<Eigen::VectorXd>& fullpath_);
  void publishFullGDCPCAPath(vector<Eigen::VectorXd>& fullpath_);
  void publishPCAVec(vector<Eigen::Vector3d>& sub_center, map<int, Eigen::Matrix3d>& sub_pcavec);
  void publishVPOpt(pcl::PointCloud<pcl::PointNormal>::Ptr& before_, pcl::PointCloud<pcl::PointNormal>::Ptr& after_);
  void publishFitCylinder(map<int, vector<double>>& cylinder_param);
  void publishHCOPPTraj(quadrotor_msgs::PolynomialTraj& posi, quadrotor_msgs::PolynomialTraj& pitch, quadrotor_msgs::PolynomialTraj& yaw);
  void publishJointSphere(vector<Eigen::Vector3d>& joints, double& radius, vector<vector<Eigen::Vector3d>>& InnerVps);
  void publishYawTraj(vector<Eigen::Vector3d>& waypt, vector<double>& yaw);
  void publishCurrentFoV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, const double& yaw);
  void publishTravelTraj(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id);
  void publishVisiblePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentCloud, int id);
  void publishVpsCHull(std::map<int, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& vpHull, vector<Eigen::Vector3d>& hamiPath);
  void publishInitVps(pcl::PointCloud<pcl::PointNormal>::Ptr& init_vps);
  /* ROSA Debug Vis */
  void publishCheckNeigh(Eigen::Vector3d& checkPoint, const pcl::PointCloud<pcl::PointXYZ>& checkNeigh, Eigen::MatrixXi& edgeMat);
  void publishCheckCP(Eigen::Vector3d& CPPoint, Eigen::Vector3d& CPDir, Eigen::Vector3d& checkRP, const pcl::PointCloud<pcl::PointXYZ>& CPPts, const pcl::PointCloud<pcl::PointXYZ>& CPPtsCluster);
  /* Updates Visualization */
  void publishUpdatesPose(pcl::PointCloud<pcl::PointXYZ>& visCloud, vector<vector<Eigen::Vector3d>>& list1, vector<vector<Eigen::Vector3d>>& list2, vector<double>& yaws);
  /* ROSA Opt */
  void publishOptArea(vector<pcl::PointCloud<pcl::PointXYZ>>& optArea);
};
}
#endif