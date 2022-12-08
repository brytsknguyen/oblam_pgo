/**
* This file is part of oblam_pgo.
* 
* Copyright (C) 2020 Thien-Minh Nguyen <thienminh.npn at gmail.com>,
* 
* For more information please see <https://brytsknguyen.github.io>.
* or <https://github.com/britsknguyen/SLICT>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* oblam_pgo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* oblam_pgo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with SLICT.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by Thien-Minh Nguyen on 11/11/22.
//


/* #region HEADERS ---------------------------------------------------------------------------------------------------*/

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <condition_variable>
#include <deque>
#include <thread>

#include <Eigen/Dense>
#include <ceres/ceres.h>

/* All needed for pointcloud manipulation -------------*/
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/impl/uniform_sampling.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>
/* All needed for pointcloud manipulation -------------*/

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

// Factor
#include "factor/RelOdomFactor.h"
#include "PoseLocalParameterization.h"

// Custom for package
#include "utility.h"

/* #endregion HEADERS -----------------------------------------------------------------------------------------------*/

using namespace Eigen;
using namespace pcl;

ros::NodeHandlePtr nh_ptr;

// Visualizing the current pose
void publishPose(PointPose &currPose)
{
    myTf tf_W_Bcurr(currPose);

    static ros::Publisher currKfPosePub = nh_ptr->advertise<nav_msgs::Odometry>("/curr_kf_pose", 10);

    // Publish latest pose for visualization
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";

    odom_msg.pose.pose.position.x = tf_W_Bcurr.pos.x();
    odom_msg.pose.pose.position.y = tf_W_Bcurr.pos.y();
    odom_msg.pose.pose.position.z = tf_W_Bcurr.pos.z();

    odom_msg.pose.pose.orientation.x = tf_W_Bcurr.rot.x();
    odom_msg.pose.pose.orientation.y = tf_W_Bcurr.rot.y();
    odom_msg.pose.pose.orientation.z = tf_W_Bcurr.rot.z();
    odom_msg.pose.pose.orientation.w = tf_W_Bcurr.rot.w();

    currKfPosePub.publish(odom_msg);

    // Publish the transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
    transform.setRotation(tf::Quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
}

// Visualizing the loop
void publishLoop(PointPose currPose, PointPose prevPose)
{
    static visualization_msgs::Marker loop_marker;
    static ros::Publisher loop_marker_pub = nh_ptr->advertise<visualization_msgs::Marker>("/loop_marker", 100);
    static std_msgs::ColorRGBA color;

    static bool loop_marker_inited = false;
    if (!loop_marker_inited)
    {
        // Set up the loop marker
        loop_marker_inited = true;
        loop_marker.header.frame_id = "world";
        loop_marker.ns = "loop_marker";
        loop_marker.type = visualization_msgs::Marker::LINE_LIST;
        loop_marker.action = visualization_msgs::Marker::ADD;
        loop_marker.pose.orientation.w = 1.0;
        loop_marker.lifetime = ros::Duration(0);
        loop_marker.id = 0;

        loop_marker.scale.x = 0.3;
        loop_marker.scale.y = 0.3;
        loop_marker.scale.z = 0.3;
        loop_marker.color.r = 0.0;
        loop_marker.color.g = 1.0;
        loop_marker.color.b = 1.0;
        loop_marker.color.a = 1.0;

        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
    }

    loop_marker.points.clear();
    loop_marker.colors.clear();

    geometry_msgs::Point point;

    point.x = currPose.x;
    point.y = currPose.y;
    point.z = currPose.z;

    loop_marker.points.push_back(point);
    loop_marker.colors.push_back(color);

    point.x = prevPose.x;
    point.y = prevPose.y;
    point.z = prevPose.z;

    loop_marker.points.push_back(point);
    loop_marker.colors.push_back(color);

    // Publish the loop marker
    loop_marker_pub.publish(loop_marker);
}

// Optimizing the pose graph.
void OptimizePoseGraph(CloudPosePtr &kfPose, int prevId, int currId, myTf<double> &tf_Bprev_Bcurr)
{
    // Number of keyframe poses = number of params to optimize
    int KF_NUM = kfPose->size();

    /* #region Create the ceres problem and settings ----------------------------------------------------------------*/
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = MAX_THREADS;
    /* #endregion Create the ceres problem and settings -------------------------------------------------------------*/

    /* #region Create the ceres params load the current values to them ----------------------------------------------*/
    double **PARAM_POSE = new double *[KF_NUM];
    for (int i = 0; i < KF_NUM; i++)
    {
        PARAM_POSE[i] = new double[7];

        PARAM_POSE[i][0] = kfPose->points[i].x;
        PARAM_POSE[i][1] = kfPose->points[i].y;
        PARAM_POSE[i][2] = kfPose->points[i].z;
        PARAM_POSE[i][3] = kfPose->points[i].qx;
        PARAM_POSE[i][4] = kfPose->points[i].qy;
        PARAM_POSE[i][5] = kfPose->points[i].qz;
        PARAM_POSE[i][6] = kfPose->points[i].qw;

        // Declare the parameter block to the ceres problem
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(PARAM_POSE[i], 7, local_parameterization);

        // Fix the current pose (Good question for quiz: why fix a pose, and why fix the current pose)
        if (i == currId)
            problem.SetParameterBlockConstant(PARAM_POSE[i]);
    }
    /* #endregion Create the ceres params load the data to them -----------------------------------------------------*/

    /* #region Add the {}^{k-j}_{k-j+1}\bar{T} factors --------------------------------------------------------------*/
    vector<ceres::internal::ResidualBlock *> res_ids_relpose;
    double cost_relpose_init = -1, cost_relpose_final = -1;

    /* ASSIGNMENT BLOCK START ---------------------------------------------------------------------------------------*/

        // Create prior relative pose factors and the residual block to ceres. Use the RelOdomFactor() class
        // ...

    /* ASSIGNMENT BLOCK END -----------------------------------------------------------------------------------------*/

    /* #endregion Add the {}^k_{k+1}\bar{T} factors -----------------------------------------------------------------*/

    /* #region Add the loop prior factors ---------------------------------------------------------------------------*/
    vector<ceres::internal::ResidualBlock *> res_ids_loop;
    double cost_loop_init = -1, cost_loop_final = -1;

    /* ASSIGNMENT BLOCK START ---------------------------------------------------------------------------------------*/

        // Create loop relative pose factors and the residual block to ceres. Use the RelOdomFactor() class
        // ..

    /* ASSIGNMENT BLOCK END -----------------------------------------------------------------------------------------*/

    /* #endregion Add the loop prior factors ------------------------------------------------------------------------*/

    /* #region Compute the initial cost -----------------------------------------------------------------------------*/
    Util::ComputeCeresCost(res_ids_relpose, cost_relpose_init, problem);
    Util::ComputeCeresCost(res_ids_loop, cost_loop_init, problem);
    /* #endregion Compute the initial cost -------------------------------------------------------------------------*/

    // Solve the ceres problem
    ceres::Solve(options, &problem, &summary);

    /* #region Compute the final cost -------------------------------------------------------------------------------*/
    Util::ComputeCeresCost(res_ids_relpose, cost_relpose_final, problem);
    Util::ComputeCeresCost(res_ids_loop, cost_loop_final, problem);
    /* #endregion Compute the final cost ----------------------------------------------------------------------------*/

    // Print out a report of the optimization problem
    printf("PGO. J: %9.3f -> %9.3f. Jrp: %9.3f -> %9.3f. Jlp: %9.3f -> %9.3f\n",
           summary.initial_cost, summary.final_cost, cost_relpose_init, cost_relpose_final, cost_loop_init, cost_loop_final);

    /* #region Load the param value back to keyframe ----------------------------------------------------------------*/
    for (int i = 0; i < KF_NUM; i++)
    {
        kfPose->points[i].x = PARAM_POSE[i][0];
        kfPose->points[i].y = PARAM_POSE[i][1];
        kfPose->points[i].z = PARAM_POSE[i][2];
        kfPose->points[i].qx = PARAM_POSE[i][3];
        kfPose->points[i].qy = PARAM_POSE[i][4];
        kfPose->points[i].qz = PARAM_POSE[i][5];
        kfPose->points[i].qw = PARAM_POSE[i][6];
    }
    /* #endregion Load the param value back to keyframe -------------------------------------------------------------*/

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oblam_pgo");
    ros::NodeHandle nh("~");
    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    printf(KGRN "OBLAM Pose Graph Optimization Started\n" RESET);

    /* #region Read the keyframe pose from memory -------------------------------------------------------------------*/

    // Find the recorded data
    string data_path; nh.param("/data_path", data_path, string("/home/tmn"));
    cout << "Data Path: " << data_path << endl;

    CloudPosePtr kfPose(new CloudPose());
    pcl::io::loadPCDFile<PointPose>(data_path + "KfCloudPose.pcd", *kfPose);

    int KF_NUM = kfPose->size();

    deque<CloudXYZITPtr> kfCloud(KF_NUM);
    #pragma omp parallel for num_threads(MAX_THREADS)
    for (int i = 0; i < KF_NUM; i++)
    {
        ROS_ASSERT((int)(kfPose->points[i].intensity) == i);

        kfCloud[i] = CloudXYZITPtr(new CloudXYZIT()); std::stringstream iss; iss << std::setw(4) << std::setfill('0') << i;
        string kf_file = data_path + "KFCloudInB/KfCloudinB_" + iss.str() + ".pcd";
        cout << "Reading file: " << kf_file << endl;
        pcl::io::loadPCDFile<PointXYZIT>(kf_file, *kfCloud[i]);
    }

    /* #endregion Read the keyframe pose from memory ----------------------------------------------------------------*/

    /* #region Create some common objects ---------------------------------------------------------------------------*/

    // Create a kdtree
    KdTreeFLANN<PointPose> kdTreeKF;

    // Recorded keyframe poses
    ros::Publisher kfAllPub = nh.advertise<sensor_msgs::PointCloud2>("/all_kf_pose", 10);

    // Up-to-current keyframe poses
    ros::Publisher kfUpToCurrPub = nh.advertise<sensor_msgs::PointCloud2>("/up_to_curr_kf_pose", 10);

    // Current keyframe and its neighbours
    ros::Publisher currKfNbrPub = nh.advertise<sensor_msgs::PointCloud2>("/curr_kf_nbr", 10);

    // Previous keyframe and its neigbours
    ros::Publisher prevKfNbrPub = nh.advertise<sensor_msgs::PointCloud2>("/prev_kf_nbr", 10);

    // Publish the point cloud
    ros::Publisher currCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/curr_cloud", 10);
    ros::Publisher prevCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/prev_cloud", 10);

    /* #endregion Create some common objects ------------------------------------------------------------------------*/

    while (ros::ok())
    {
        // Increment the keyframe index
        static int currId = -1; currId++;
        if (currId == kfPose->size() - 1)
        {
            printf("Process finished.");
            exit(0);
        }

        CloudPosePtr kfPoseUpToCurr(new CloudPose());
        for(int i = 0; i <= currId; i++)
            kfPoseUpToCurr->push_back(kfPose->points[i]);

        // Publish keyframe pose for vizualization
        Util::publishCloud(kfAllPub, *kfPose, ros::Time::now(), "world");
        Util::publishCloud(kfUpToCurrPub, *kfPoseUpToCurr, ros::Time::now(), "world");

        // Search for the neigboring keyframes
        kdTreeKF.setInputCloud(kfPoseUpToCurr);

        // Load current pose and point cloud
        PointPose currPose = kfPose->points[currId];
        CloudXYZITPtr &currCloudInB = kfCloud[currId];
        CloudXYZITPtr currCloudInW(new CloudXYZIT());
        pcl::transformPointCloud(*currCloudInB, *currCloudInW, myTf(currPose).cast<float>().tfMat());
        
        // Publish current cloud for visualization
        Util::publishCloud(currCloudPub, *currCloudInW, ros::Time::now(), "world");

        // Publish pose for visualization
        publishPose(currPose);

        // Search for a loop candidate
        int prevId = -1;
        bool prevKfCandidateFound = false;

        /* ASSIGNMENT BLOCK START -----------------------------------------------------------------------------------*/

            // Step 1. Use the kdtreekf.nearestksearch() function to search for N neighbours of currpose (you decide N).
            // Step 2: Come up with some logics to determine a loop closure keyframe candidate in this neigbourhood.
            // Step 3: Pass the ID of the keyframe candidate to "previd" and set prevkfcandidatefound to "true" to proceed.

        /* ASSIGNMENT BLOCK END -------------------------------------------------------------------------------------*/

        if (!prevKfCandidateFound)
        {
            this_thread::sleep_for(chrono::milliseconds(25)); // You can reduce 25 ms to shorter wait
            continue;
        }

        // oldest time is more than 20s, begin the loop closure check
        PointPose prevPose = kfPose->points[prevId];
        CloudXYZITPtr prevCloudInW(new CloudXYZIT());

        // Search for 10 keyframes arround the previous keyframe
        int loop_kf_nbr = 10; int bId = prevId; int fId = prevId; int span = fId - bId;
        while (span < loop_kf_nbr)
        {
            bId = max(0, bId - 1);
            fId = min(fId + 1, currId - 1);

            if (fId - bId == span || fId - bId >= loop_kf_nbr)
                break;

            span = fId - bId;
        }

        // Merge all 10 keyframe pointclouds around the previous keyframe
        CloudPosePtr kfPrevNbr(new CloudPose());
        for (int idx = bId; idx <= fId; idx++)
        {
            myTf tf_W_B(kfPose->points[idx]);
            CloudXYZITPtr kfCloudInW(new CloudXYZIT());
            pcl::transformPointCloud(*kfCloud[idx], *kfCloudInW, tf_W_B.pos, tf_W_B.rot);

            *prevCloudInW += *kfCloudInW;

            kfPrevNbr->push_back(kfPose->points[idx]);
        }
        
        // Publish neighbour of previous keyframe for visualization
        Util::publishCloud(prevKfNbrPub, *kfPrevNbr, ros::Time::now(), "world");

        // Create Downsampler
        VoxelGrid<PointXYZIT> downsampler;
        double voxel_size = 0.4;
        downsampler.setLeafSize(voxel_size, voxel_size, voxel_size);

        // Downsample Previous Cloud and publish for visualization
        downsampler.setInputCloud(prevCloudInW);
        downsampler.filter(*prevCloudInW);
        Util::publishCloud(prevCloudPub, *prevCloudInW, ros::Time::now(), "world");

        // Downsample Current Cloud
        downsampler.setInputCloud(currCloudInB);
        downsampler.filter(*currCloudInB);

        // Create ICP object
        pcl::IterativeClosestPoint<PointXYZIT, PointXYZIT> icp;
        icp.setMaxCorrespondenceDistance(15 * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-3);
        icp.setEuclideanFitnessEpsilon(1e-3);
        icp.setRANSACIterations(0);

        // Declare the clouds to align
        icp.setInputSource(currCloudInB);
        icp.setInputTarget(prevCloudInW);

        myTf tf_IcpGuess = myTf(currPose);

        // Run ICP
        CloudXYZITPtr aligned_result(new CloudXYZIT());
        icp.align(*aligned_result, tf_IcpGuess.cast<float>().tfMat());

        bool icpconverged = icp.hasConverged();
        float icpFitnessRes = icp.getFitnessScore();
        Matrix4f tfm_W_Bcurr = icp.getFinalTransformation(); // ICP-based alignment

        // If fitness is below a threshold, trigger the loop closure and PGO
        bool icp_passed = icpFitnessRes < 0.3;

        // Report on the ICP result
        printf("%sPrevId: %d. ICP %s. Fitness: %f.\n" RESET, icpFitnessRes < 0.3 ? KGRN : KYEL, prevId, icp_passed ? "passed" : "failed", icpFitnessRes);
        if (icp_passed)
        {
            // Icp passes, run PGO
            cout << "tf_W_Bcurr" << endl
                 << tfm_W_Bcurr << endl;

            // Loop closure prior {}^Bprev_Bcurr\bar{T}
            myTf tf_Bprev_Bcurr = myTf(prevPose).inverse() * myTf(tfm_W_Bcurr);

            // Optimize the pose graph
            OptimizePoseGraph(kfPose, prevId, currId, tf_Bprev_Bcurr);
        }

        // Publish all of the keyframe pose
        Util::publishCloud(kfAllPub, *kfPose, ros::Time::now(), "world");

        // Visualize the loop
        if (icp_passed)
            publishLoop(kfPose->points[currId], kfPose->points[prevId]);

        // Write down the keyframe pose
        PCDWriter writer; writer.writeASCII(data_path + "/KfCloudPoseOptimized.pcd", *kfPose, 18);

        this_thread::sleep_for(chrono::milliseconds(25));
    }

    return 0;
}
