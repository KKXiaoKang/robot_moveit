
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include <nlohmann/json.hpp>
#include "std_msgs/Float64MultiArray.h"
// #include "arm_traj.hpp"

using namespace KDL;
using json = nlohmann::json;

#define l_ee_pose_data   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/lee_pose.txt"
#define r_ee_pose_data   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/ree_pose.txt"
#define l_jnt_ang_data   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/ljnt_ang.txt"
#define r_jnt_ang_data   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/rjnt_ang.txt"
#define l_jnt_ang_json   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/ljnt_ang.json"
#define r_jnt_ang_json   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/rjnt_ang.json"
#define tmp_data   "/home/lab/yapeng/moveit_ws/src/humanoidmanipulation/trac_ik_examples/data/tmp_data.txt"

const double degree = M_PI/180;
int move_num = 500;

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}


KDL::JntArray EigVec2kdlArray(Eigen::VectorXd eigenVector)
{
  KDL::JntArray kdlArray(eigenVector.size());
  for (int i = 0; i < eigenVector.size(); ++i) {
        kdlArray(i) = eigenVector(i);
    }

  return kdlArray;
}

Eigen::VectorXd kdlJnt2EigVec(const KDL::JntArray& kdlArray) {
    Eigen::VectorXd eigenVector(kdlArray.rows());

    for (int i = 0; i < kdlArray.rows(); ++i) {
        eigenVector(i) = kdlArray(i);
    }

    return eigenVector;
}

KDL::Frame eigenVectorXdToKDLFrame(const Eigen::VectorXd& vec) {
    if (vec.size() != 6) {
        return KDL::Frame();
    }

    Eigen::Vector3d linearCoords = vec.head(3);

    double roll = vec(3);
    double pitch = vec(4);
    double yaw = vec(5);
    KDL::Rotation rotation = KDL::Rotation::RPY(roll, pitch, yaw);

    KDL::Vector position(linearCoords(0), linearCoords(1), linearCoords(2));
    KDL::Frame frame(rotation, position);

    return frame;
}

Eigen::VectorXd kdlFrameToEigenVectorXd(const KDL::Frame& end_effector_pose) {
  Eigen::VectorXd result(6);

  // 获取线性坐标
  Eigen::Vector3d linearCoords;
  linearCoords << end_effector_pose.p(0), end_effector_pose.p(1), end_effector_pose.p(2);
  result.head(3) = linearCoords;

  // 获取旋转矩阵并将其转换为欧拉角
  double roll, pitch, yaw;
  end_effector_pose.M.GetRPY(roll, pitch, yaw);

  // 将欧拉角作为后三个元素添加到向量中
  result.tail(3) << roll, pitch, yaw;

  return result;
}

std::vector<Eigen::VectorXd> concatenateVectors(const std::vector<Eigen::VectorXd>& v1, const std::vector<Eigen::VectorXd>& v2) {
    std::vector<Eigen::VectorXd> result;

    result.insert(result.end(), v1.begin(), v1.end());
    result.insert(result.end(), v2.begin(), v2.end());

    return result;
}

std::vector<Eigen::VectorXd> concatenateVectorsCol(const std::vector<Eigen::VectorXd>& v1, const std::vector<Eigen::VectorXd>& v2) {
    std::vector<Eigen::VectorXd> result(v1.size());
    Eigen::VectorXd combined_vec(2*v1[0].size());
    for (size_t i = 0; i < v1.size(); ++i) {
      combined_vec << v1[i], v2[i];
      result[i] = combined_vec;
    }
    return result;
}

void splitVector(const std::vector<Eigen::VectorXd>& concatenatedVector,
                 std::vector<Eigen::VectorXd>& v1,
                 std::vector<Eigen::VectorXd>& v2) {
    // 计算拆分点的位置（这里假设是一半）
    size_t splitPoint = concatenatedVector.size() / 2;

    // 使用迭代器进行拆分
    v1.assign(concatenatedVector.begin(), concatenatedVector.begin() + splitPoint);
    v2.assign(concatenatedVector.begin() + splitPoint, concatenatedVector.end());
}
void splitVectorCol(const std::vector<Eigen::VectorXd>& concatenatedVector,
                 std::vector<Eigen::VectorXd>& v1,
                 std::vector<Eigen::VectorXd>& v2) {
    // 使用迭代器进行拆分
    for (size_t i = 0; i < concatenatedVector.size(); ++i) {
        // 前7列
        v1[i] = concatenatedVector[i].head(7);
        // 后7列
        v2[i] = concatenatedVector[i].tail(7);
    }
}

void writevec(const char * datafile, Eigen::VectorXd vec)
{
  std::ofstream savevec;
  savevec.open(datafile, std::ios::app);

  for (unsigned i = 0; i < vec.size(); i++) {
    savevec << vec[i] << "\t";
  }
  savevec <<"\n";
  savevec.close();
}

void writevec2json(const char * datafile, std::vector<Eigen::VectorXd> vec)
{
  json jsonData;
  for (const auto& vecEntry : vec) {
      json jsonVecEntry;
      for (int i = 0; i < vecEntry.size(); ++i) {
          jsonVecEntry.push_back(vecEntry(i));
      }
      jsonData.push_back(jsonVecEntry);
  }

  // 将 JSON 写入文件
  std::ofstream outputFile(datafile);
  outputFile << std::setw(4) << jsonData << std::endl;
  outputFile.close();
}

std::vector<Eigen::VectorXd> readvecfromjson(const char * datafile)
{
  std::ifstream inputFile(datafile);
  if (!inputFile.is_open()) {
      std::cerr << "Error opening JSON file." << std::endl;
  }
  // 解析 JSON 数据
  json jsonData;
  inputFile >> jsonData;
  inputFile.close();
  // 将 JSON 数据存储到 std::vector<Eigen::VectorXd>
  std::vector<Eigen::VectorXd> vec;
  for (const auto& jsonVecEntry : jsonData) {
      Eigen::VectorXd vecEntry(jsonVecEntry.size());
      int i = 0;
      for (const auto& entry : jsonVecEntry) {
          vecEntry(i++) = entry;
      }
      vec.push_back(vecEntry);
  }
  return vec;
}

std::vector<Eigen::VectorXd> readvec6(const char * datafile)
{
  std::vector<Eigen::VectorXd> vec;
  std::ifstream file(datafile, std::ios::in );

  if( !file )
    std::cerr << "Cannot open " << std::endl;

  double v0, v1, v2, v3, v4, v5;

  while( !file.eof() )
  {
      file  >> v0 >> v1 >> v2 >> v3 >> v4 >> v5;
      Eigen::VectorXd eigenVec(6);
      eigenVec << v0, v1, v2, v3, v4, v5;
      vec.push_back(eigenVec);
  }
  file.close();

  return vec;
}

std::vector<Eigen::VectorXd> calioffset(Eigen::VectorXd ObsePose, Eigen::VectorXd DefaPose, std::vector<Eigen::VectorXd> traj)
{
  /**
   * @brief get the Frame vector and rotation
   * @param offset is the user-defined input offset (x, y, theta);
   * @ref http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29
   */

  Eigen::VectorXd DeltPose(7);
  Eigen::VectorXd CaliPose(6);
  Quat CaliQuat;
  Quat defaultQuat_inv(4);
  DeltPose.head(3) = ObsePose.head(3) - DefaPose.head(3);
  defaultQuat_inv << DefaPose[3], -DefaPose[4], -DefaPose[5], -DefaPose[6];
  DeltPose.tail(4) = quatProduct(ObsePose.tail(4), defaultQuat_inv); // d = 1 - 2
  
  // calibrate traj 
  std::vector<Eigen::VectorXd> cali(traj.size());
  Quat OrinQuat;
  Eigen::VectorXd result(6);

  for(int i=0; i<traj.size(); i++)
  {
    result.head(3) = traj[i].head(3) + DeltPose.head(3);
    OrinQuat = rpyToQuat(traj[i].tail(3));
    CaliQuat = quatProduct(DeltPose.tail(4), OrinQuat);
    result.tail(3) = quatToRPY(CaliQuat);
    cali[i] = result;
    // std::cout<<" output :"<<i<<std::endl;
  }
  
  return cali;
}

std::vector<Eigen::VectorXd> movetoready(Eigen::VectorXd lq1, Eigen::VectorXd rq1, int num)
{
  std::vector<Eigen::VectorXd> v1(num);
  std::vector<Eigen::VectorXd> v2(num);
  std::vector<Eigen::VectorXd> v_move(2*num);
  for (int i = 0; i < num; ++i)
  {
    double alpha = static_cast<double>(i) / 100.0;
    v1[i] = alpha * lq1;
    v2[i] = alpha * rq1;
  }
  v_move = concatenateVectors(v1, v2);
  return v_move;
}

void test(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{

  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);

  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);
  // std::cout<<"ll:"<<ll.data.transpose()<<std::endl;

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO("Using %d joints", chain.getNrOfJoints());


  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());  // size = 7

  // std::cout<<"nominal.data.size()="<<nominal.data.size()<<std::endl;  

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  for (uint i = 0; i < num_samples; i++)
  {
    for (uint j = 0; j < ll.data.size(); j++)
    {
      q(j) = fRand(ll(j), ul(j));
    }
    JointList.push_back(q);
  }

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;

  double total_time = 0;
  uint success = 0;

  ROS_INFO_STREAM("*** Testing KDL with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    double elapsed = 0;
    result = nominal; // start with nominal
    start_time = boost::posix_time::microsec_clock::local_time();
    do
    {
      q = result; // when iterating start with last solution
      rc = kdl_solver.CartToJnt(q, end_effector_pose, result);
      diff = boost::posix_time::microsec_clock::local_time() - start_time;
      elapsed = diff.total_nanoseconds() / 1e9;
    }
    while (rc < 0 && elapsed < timeout);
    total_time += elapsed;
    if (rc >= 0)
      success++;

    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  ROS_INFO_STREAM("KDL found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");


  total_time = 0;
  success = 0;

  ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time += elapsed;
    if (rc >= 0)
      success++;

    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }
  std::cout<< "-----------------" <<std::endl;

  ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");

}

std::vector<Eigen::VectorXd> generate_eepos(double num_samples, KDL::ChainFkSolverPos& lfk_solver, KDL::ChainFkSolverPos& rfk_solver, int num_joints)
{
  std::vector<Eigen::VectorXd> ee_pose(2*num_samples);
  std::vector<Eigen::VectorXd> lee_pose(num_samples);
  std::vector<Eigen::VectorXd> ree_pose(num_samples);
  std::vector<Eigen::VectorXd> lee_pose_c(num_samples + move_num);
  std::vector<Eigen::VectorXd> ree_pose_c(num_samples + move_num);
  Eigen::VectorXd lj_init(num_joints);
  Eigen::VectorXd lj_end(num_joints);
  Eigen::VectorXd rj_init(num_joints);
  Eigen::VectorXd rj_end(num_joints);
  std::vector<Eigen::VectorXd> lj_traj(num_samples);
  std::vector<Eigen::VectorXd> rj_traj(num_samples);
  std::vector<KDL::JntArray> lJntList(num_samples);
  std::vector<KDL::JntArray> rJntList(num_samples);
  KDL::Frame lee_pose_, ree_pose_;

  // lj_init << -0.61, 0.55, -1.31, -1.63, 0.0, 0.0, 0.0; 
  // lj_end  << -0.38, 0.31, -1.31, -1.2,  0.0, 0.0, 0.0; 
  // rj_init << -0.61, -0.55, 1.31, -1.63, 0.0, 0.0, 0.0;
  // rj_end  << -0.38, -0.31, 1.31, -1.2,  0.0, 0.0, 0.0;
  lj_init << -0.2, 0.25, -0.7, -1.7, 0.0, 0.0, 0.0; 
  lj_end  <<  0.18, 0.25, -0.85, -1.3, 0.0, 0.0, 0.0;
  rj_init << -0.2, -0.25, 0.7, -1.7, 0.0, 0.0, 0.0; 
  rj_end  <<  0.18, -0.25, 0.85, -1.3, 0.0, 0.0, 0.0;

  // lj_init << -0.61, 0.55, -1.31, -1.63, 0.0, 0.0, 0.0; 
  // lj_end  << -0.38, 0.31, -1.31, -1.2,  0.0, 0.0, 0.0; 
  // rj_init << -0.61, -0.55, 1.31, -1.63, 0.0, 0.0, 0.0;
  // rj_end  << -0.38, -0.31, 1.31, -1.2,  0.0, 0.0, 0.0;
  std::vector<Eigen::VectorXd> move2ready_q(2*move_num);
  std::vector<Eigen::VectorXd> move2ready_lq(move_num);
  std::vector<Eigen::VectorXd> move2ready_rq(move_num);
  move2ready_q = movetoready(lj_init, rj_init, move_num);
  splitVector(move2ready_q, move2ready_lq, move2ready_rq);

  for (int i = 0; i < num_samples; ++i)
  {
    double alpha = static_cast<double>(i) / (num_samples - 1);
    lj_traj[i] = (1 - alpha) * lj_init + alpha * lj_end;
    rj_traj[i] = (1 - alpha) * rj_init + alpha * rj_end;
    lJntList[i] = EigVec2kdlArray(lj_traj[i]);
    rJntList[i] = EigVec2kdlArray(rj_traj[i]);
    bool lfwdkin_status = lfk_solver.JntToCart(lJntList[i], lee_pose_);
    bool rfwdkin_status = rfk_solver.JntToCart(rJntList[i], ree_pose_);
    if(lfwdkin_status >= 0) {
    }
    else {
      printf("%s \n", "Error:could not calculate left arm forward kinematics : ");
    }
    if(rfwdkin_status >= 0) {
    }
    else {
      printf("%s \n", "Error:could not calculate right arm forward kinematics : ");
    }
    lee_pose[i] = kdlFrameToEigenVectorXd(lee_pose_);
    ree_pose[i] = kdlFrameToEigenVectorXd(ree_pose_);

    writevec(l_ee_pose_data, lee_pose[i]);
    writevec(r_ee_pose_data, ree_pose[i]);
  }
  lee_pose_c = concatenateVectors(move2ready_lq, lee_pose);
  ree_pose_c = concatenateVectors(move2ready_rq, ree_pose);
  ee_pose = concatenateVectors(lee_pose_c, ree_pose_c);
  return ee_pose;
}

std::vector<Eigen::VectorXd> LinearInterpolation(double num_samples, Eigen::VectorXd v1, Eigen::VectorXd v2)
{
  std::vector<Eigen::VectorXd> arm_traj(num_samples);
  for (int i = 0; i < num_samples; ++i)
  {
    double alpha = static_cast<double>(i) / (num_samples - 1);
    arm_traj[i] = (1 - alpha) * v1 + alpha * v2;
  }
  return arm_traj;
}

KDL::JntArray guess_init(KDL::JntArray ll, KDL::JntArray ul)
{
  KDL::JntArray nominal(ll.data.size());
  for (uint j = 0; j < ll.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }
  return nominal;
}

/**T21：原坐标系Frame1在新坐标系Frame2中的位姿；
 * P1：刚体在Frame1中的位姿；
 * 返回刚体在Frame2中的位姿**/
KDL::Frame FrameTrans(KDL::Frame T21, KDL::Frame P1)
{
  KDL::Frame frame2;
  frame2 = T21 * P1;
  return frame2;
}

void guess_q(KDL::JntArray& lq, KDL::JntArray& rq)
{
  // lq(0) = -0.61;  lq(1) = 0.55;  lq(2) = -1.31;  lq(3) = -1.63;  lq(4) = 0.0;
  // lq(5) = 0.0;    lq(6) =  0.0;
  // rq(0) =  -0.61;  rq(1) =  -0.55;  rq(2) = 1.31;  rq(3) = -1.63;   rq(4) = 0.0;
  // rq(5) = 0.0;    rq(6) =  0.0;
  lq(0) = -0.2;  lq(1) = 0.25;  lq(2) = -0.7;  lq(3) = -1.7;  lq(4) = 0.0;
  lq(5) = 0.0;    lq(6) =  0.0;
  rq(0) = lq(0);  rq(1) = -lq(1);  rq(2) = -lq(2);  rq(3) = lq(3);   rq(4) = 0.0;
  rq(5) = 0.0;    rq(6) =  0.0;
}
std::vector<Eigen::VectorXd> test_traj(ros::NodeHandle& nh, int num_samples, std::string chain_start, std::string lchain_end, std::string rchain_end, double timeout, std::string urdf_param)
{
  double eps = 1e-5;
  TRAC_IK::TRAC_IK ltracik_solver(chain_start, lchain_end, urdf_param, timeout, eps, TRAC_IK::Distance);
  TRAC_IK::TRAC_IK rtracik_solver(chain_start, rchain_end, urdf_param, timeout, eps, TRAC_IK::Distance);

  KDL::Chain lchain, rchain;
  KDL::JntArray l_ll, l_ul, r_ll, r_ul; //lower joint limits, upper joint limits

  bool lvalid = ltracik_solver.getKDLChain(lchain);
  bool rvalid = rtracik_solver.getKDLChain(rchain);
  if (!lvalid)
    ROS_ERROR("There was no valid KDL chain found");

  lvalid = ltracik_solver.getKDLLimits(l_ll, l_ul);
  rvalid = rtracik_solver.getKDLLimits(r_ll, r_ul);
  if (!lvalid)
    ROS_ERROR("There were no valid KDL joint limits found");
  if (!rvalid)
    ROS_ERROR("There were no valid KDL joint limits found");

  assert(lchain.getNrOfJoints() == l_ll.data.size());
  assert(lchain.getNrOfJoints() == l_ul.data.size());
  assert(rchain.getNrOfJoints() == r_ll.data.size());
  assert(rchain.getNrOfJoints() == r_ul.data.size());
  ROS_INFO("Using %d joints", lchain.getNrOfJoints());

  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive lfk_solver(lchain); // Forward kin. solver
  KDL::ChainFkSolverPos_recursive rfk_solver(rchain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv lvik_solver(lchain); // PseudoInverse vel solver
  KDL::ChainIkSolverVel_pinv rvik_solver(rchain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL lkdl_solver(lchain, l_ll, l_ul, lfk_solver, lvik_solver, 1, eps); // Joint Limit Solver
  KDL::ChainIkSolverPos_NR_JL rkdl_solver(rchain, r_ll, r_ul, rfk_solver, rvik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(lchain.getNrOfJoints());

  /**generate ee_pos traj **/
  std::vector<Eigen::VectorXd> ee_pose(2*num_samples);  
  std::vector<Eigen::VectorXd> lee_pose(num_samples);  
  std::vector<Eigen::VectorXd> lee_pose_cali(num_samples);  
  std::vector<Eigen::VectorXd> ree_pose(num_samples);  
  std::vector<KDL::Frame> lee_pose_(num_samples);
  std::vector<KDL::Frame> ree_pose_(num_samples);
  int rc;

  remove(l_ee_pose_data);
  remove(r_ee_pose_data);
  ee_pose = generate_eepos(num_samples, lfk_solver, rfk_solver, lchain.getNrOfJoints());

  /**from ee_pose to joint_angle traj **/
  lee_pose = readvec6(l_ee_pose_data);
  ree_pose = readvec6(r_ee_pose_data);
  
  /**define a DefaultPose ideally, the CalibratedPose can be calculated based ObservedPose x,y,z,(w,x,y,z)**/
  Eigen::VectorXd DefaPose(7);
  Eigen::VectorXd ObsePose(7);
  Eigen::VectorXd CaliPose(7);
 
  Quat ObseQuat;
  ObseQuat = rpyToQuat(Eigen::Vector3d(2.02636, -1.23993, -2.68189));

  DefaPose << 0.224985,	0.156554,	0.171272,	0.578154, -0.142019, -0.742493, -0.307039;
  ObsePose << 0.224985,	0.156554,	0.171272,	0.578154, -0.142019, -0.742493, -0.307039;
  lee_pose_cali = calioffset(ObsePose, DefaPose, lee_pose);
  
  KDL::JntArray l_guessq(lchain.getNrOfJoints());  // size = 7
  KDL::JntArray l_res_q_(lchain.getNrOfJoints());
  Eigen::VectorXd l_res_q(lchain.getNrOfJoints());
  std::vector<Eigen::VectorXd> l_q_Array(num_samples);
  std::vector<Eigen::VectorXd> r_q_Array(num_samples);
  KDL::JntArray r_guessq(rchain.getNrOfJoints());  // size = 7
  KDL::JntArray r_res_q_(rchain.getNrOfJoints());
  Eigen::VectorXd r_res_q(rchain.getNrOfJoints());
  std::vector<Eigen::VectorXd> l_res_q_Array(num_samples);
  std::vector<Eigen::VectorXd> r_res_q_Array(num_samples);
  std::vector<Eigen::VectorXd> res_q_Array(2*num_samples);

  guess_q(l_guessq, r_guessq);
  // l_guessq = guess_init(l_ll, l_ul);
  // r_guessq = guess_init(r_ll, r_ul);

  for(int i = 0; i < lee_pose.size()-1; i++)
  {
    lee_pose_[i] = eigenVectorXdToKDLFrame(lee_pose_cali[i]);
    rc = ltracik_solver.CartToJnt(l_guessq, lee_pose_[i], l_res_q_);
    if (rc < 0)
      ROS_FATAL("ik solve failure !");
    l_guessq = l_res_q_;
    l_res_q = kdlJnt2EigVec(l_res_q_);
    writevec(l_jnt_ang_data, l_res_q);

    l_res_q_Array[i] = l_res_q;
  }
  for(int i = 0; i < ree_pose.size()-1; i++)
  {
    ree_pose_[i] = eigenVectorXdToKDLFrame(ree_pose[i]);
    rc = rtracik_solver.CartToJnt(r_guessq, ree_pose_[i], r_res_q_);
    if (rc < 0)
      ROS_FATAL("ik solve failure !");
    r_guessq = r_res_q_;
    r_res_q = kdlJnt2EigVec(r_res_q_);
    writevec(r_jnt_ang_data, r_res_q);

    r_res_q_Array[i] = r_res_q;
  }

  res_q_Array = concatenateVectorsCol(l_res_q_Array, r_res_q_Array);

  // writevec2json(l_jnt_ang_json, l_res_q_Array);
  // writevec2json(r_jnt_ang_json, r_res_q_Array);
  // l_q_Array = readvecfromjson(l_jnt_ang_json);
  // r_q_Array = readvecfromjson(r_jnt_ang_json);
  // res_q_Array = concatenateVectorsCol(l_q_Array, r_q_Array);

  return res_q_Array;
}

Eigen::VectorXd est_q(33);

std::vector<Eigen::VectorXd> test_demo(ros::NodeHandle& nh, int num_samples, std::string chain_start, std::string lchain_end, std::string rchain_end, double timeout, std::string urdf_param, Eigen::VectorXd init_q)
{
  double eps = 1e-5;
  TRAC_IK::TRAC_IK ltracik_solver(chain_start, lchain_end, urdf_param, timeout, eps, TRAC_IK::Distance);
  TRAC_IK::TRAC_IK rtracik_solver(chain_start, rchain_end, urdf_param, timeout, eps, TRAC_IK::Distance);

  KDL::Chain lchain, rchain;
  KDL::JntArray l_ll, l_ul, r_ll, r_ul; //lower joint limits, upper joint limits

  bool lvalid = ltracik_solver.getKDLChain(lchain);
  bool rvalid = rtracik_solver.getKDLChain(rchain);
  if (!lvalid)
    ROS_ERROR("There was no valid KDL chain found");

  lvalid = ltracik_solver.getKDLLimits(l_ll, l_ul);
  rvalid = rtracik_solver.getKDLLimits(r_ll, r_ul);
  if (!lvalid)
    ROS_ERROR("There were no valid KDL joint limits found");
  if (!rvalid)
    ROS_ERROR("There were no valid KDL joint limits found");   

  assert(lchain.getNrOfJoints() == l_ll.data.size());
  assert(lchain.getNrOfJoints() == l_ul.data.size());
  assert(rchain.getNrOfJoints() == r_ll.data.size());
  assert(rchain.getNrOfJoints() == r_ul.data.size());
  ROS_INFO("Using %d joints", lchain.getNrOfJoints()+rchain.getNrOfJoints());

  // DefaPose << 0.224985,	0.156554,	0.171272,	0.578154, -0.142019, -0.742493, -0.307039;
  
  KDL::JntArray lq_mid_(lchain.getNrOfJoints());  // size = 7
  KDL::JntArray rq_mid_(rchain.getNrOfJoints());  // size = 7
  Eigen::VectorXd lq_mid(lchain.getNrOfJoints());  // size = 7
  Eigen::VectorXd rq_mid(lchain.getNrOfJoints());  // size = 7
  Eigen::VectorXd q_end(2*lchain.getNrOfJoints());  // size = 7
  std::vector<Eigen::VectorXd> q_traj(num_samples);

  lq_mid_ = guess_init(l_ll, l_ul);
  rq_mid_ = guess_init(r_ll, r_ul);
  lq_mid = kdlJnt2EigVec(lq_mid_);
  rq_mid = kdlJnt2EigVec(rq_mid_);
  // q_end << lq_mid, rq_mid;
  std::cout<<"q_mid = "<<q_end.transpose()<<std::endl;
  q_end = init_q.tail(14);
  q_end[3] = -0.5;
  q_end[10] = -0.5;

  q_traj = LinearInterpolation(num_samples, init_q.tail(14), q_end);
  for(int i=0;i<q_traj.size();i++)
  {
    writevec(tmp_data, q_traj[i]);
  }
  return q_traj;
}

void kuavoEstCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) 
{
  for (size_t i = 0; i < msg->data.size(); i++)
  {
    est_q[i] = msg->data[i]*deg2rad;
  }
}

int main(int argc, char** argv)
{

  remove(l_jnt_ang_data);
  remove(r_jnt_ang_data);

  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 1);

  ros::Rate loop_rate(400);    //这个设置的太大，效果很不好，目前觉得为10最好了
  ros::Subscriber kuavo_est_sub = nh.subscribe("/kuavo_sensors", 1000, kuavoEstCallback);
  std_msgs::Float64MultiArray::ConstPtr initial_msg =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/kuavo_sensors", nh);
  // 检查是否成功接收到消息
  if (!initial_msg) {
      ROS_ERROR("Failed to receive initial message from /kuavo_sensors. Exiting.");
      return 1;
  }
  for (size_t i = 0; i < initial_msg->data.size(); i++)
  {
      est_q[i] = initial_msg->data[i]*deg2rad;
  }
  // std::cout<<"est_q = "<<est_q[19]*rad2deg<<"|"<<est_q[21]*rad2deg<<"|"<<est_q[22]*rad2deg<<"\n";
  // std::cout<<"est_q = "<<est_q[26]*rad2deg<<"|"<<est_q[28]*rad2deg<<"|"<<est_q[29]*rad2deg<<"\n";

  int num_samples;
  std::string chain_start, lchain_end, rchain_end, urdf_param;
  double timeout;
  bool kuavo_flag = true;

  nh.param("num_samples", num_samples, 1000);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("lchain_end", lchain_end, std::string(""));
  nh.param("rchain_end", rchain_end, std::string(""));

  if (chain_start == "" || lchain_end == "" || rchain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  if (num_samples < 1)
    num_samples = 1;

  std::vector<Eigen::VectorXd> Cali_angles(2*num_samples);
  std::vector<Eigen::VectorXd> l_Cali_angles(num_samples);
  std::vector<Eigen::VectorXd> r_Cali_angles(num_samples);
  Cali_angles = test_traj(nh, num_samples, chain_start, lchain_end, rchain_end, timeout, urdf_param);
  // Cali_angles = test_demo(nh, num_samples, chain_start, lchain_end, rchain_end, timeout, urdf_param, est_q);
  splitVectorCol(Cali_angles, l_Cali_angles, r_Cali_angles);
  sensor_msgs::JointState joint_state;
  int num = 0;
  int step = 0;
  int count = 0;

  while (ros::ok())
  {
    joint_state.header.stamp = ros::Time::now();
    if(!kuavo_flag)
    {
      joint_state.name.resize(43);
      joint_state.position.resize(43);
      joint_state.name[0]="l_leg_roll"; joint_state.name[1]="l_leg_yaw";    joint_state.name[2]="l_leg_pitch";
      joint_state.name[3]="l_knee";     joint_state.name[4]="l_foot_pitch"; joint_state.name[5]="l_foot_roll";
      joint_state.name[6]="l_l_bar";    joint_state.name[7]="l_l_tendon";   joint_state.name[8]="l_r_bar";
      joint_state.name[9]="l_r_tendon"; 
      joint_state.name[10]="r_leg_roll";  joint_state.name[11]="r_leg_yaw";    joint_state.name[12]="r_leg_pitch";
      joint_state.name[13]="r_knee";      joint_state.name[14]="r_foot_pitch"; joint_state.name[15]="r_foot_roll";
      joint_state.name[16]="r_r_bar";     joint_state.name[17]="r_r_tendon";   joint_state.name[18]="r_l_bar";
      joint_state.name[19]="r_l_tendon"; 
      joint_state.name[20]="l_arm_pitch";       joint_state.name[21]="l_arm_roll";       joint_state.name[22]="l_arm_yaw";
      joint_state.name[23]="l_forearm_pitch";   joint_state.name[24]="l_forearm_yaw";    joint_state.name[25]="l_hand_roll";
      joint_state.name[26]="l_hand_pitch";      joint_state.name[27]="l_font_finger";    joint_state.name[28]="l_l_arm_bar";     
      joint_state.name[29]="l_l_arm_tendon";    joint_state.name[30]="l_r_arm_bar";      joint_state.name[31]="l_r_arm_tendon";

      joint_state.name[32]="r_arm_pitch";       joint_state.name[33]="r_arm_roll";        joint_state.name[34]="r_arm_yaw";
      joint_state.name[35]="r_forearm_pitch";   joint_state.name[36]="r_forearm_yaw";     joint_state.name[37]="r_hand_roll";
      joint_state.name[38]="r_hand_pitch";      joint_state.name[39]="r_r_arm_bar";       joint_state.name[40]="r_r_arm_tendon"; 
      joint_state.name[41]="r_l_arm_bar";       joint_state.name[42]="r_l_arm_tendon";
      
      joint_state.position[0] = 0.0;  joint_state.position[1] = 0.0;  joint_state.position[2] = 0.0; 
      joint_state.position[3] = 0.0;  joint_state.position[4] = 0.0;  joint_state.position[5] = 0.0; 
      joint_state.position[6] = 0.0;  joint_state.position[7] = 0.0;  joint_state.position[8] = 0.0; 
      joint_state.position[9] = 0.0;  
      joint_state.position[10] = 0.0;  joint_state.position[11] = 0.0; 
      joint_state.position[13] = 0.0;  joint_state.position[14] = 0.0;  joint_state.position[15] = 0.0; 
      joint_state.position[16] = 0.0;  joint_state.position[17] = 0.0;  joint_state.position[18] = 0.0; 
      joint_state.position[19] = 0.0;  
      joint_state.position[20] = 0.0;  joint_state.position[21] = 0.0; 
      joint_state.position[23] = 0.0;  joint_state.position[24] = 0.0;  joint_state.position[25] = 0.0; 
      joint_state.position[26] = 0.0;  joint_state.position[27] = 0.0;  joint_state.position[28] = 0.0; 
      joint_state.position[29] = 0.0;  joint_state.position[30] = 0.0;  joint_state.position[31] = 0.0; 
      joint_state.position[32] = 0.0;  joint_state.position[33] = 0.0;  joint_state.position[34] = 0.0; 
      joint_state.position[35] = 0.0;  joint_state.position[36] = 0.0;  joint_state.position[37] = 0.0; 
      joint_state.position[38] = 0.0;  joint_state.position[39] = 0.0;  joint_state.position[40] = 0.0; 
      joint_state.position[41] = 0.0;  joint_state.position[42] = 0.0; 
      
      joint_state.position[20] = l_Cali_angles[num][0];   joint_state.position[32] = r_Cali_angles[num][0];
      joint_state.position[21] = l_Cali_angles[num][1];   joint_state.position[33] = r_Cali_angles[num][1];
      joint_state.position[22] = l_Cali_angles[num][2];   joint_state.position[34] = r_Cali_angles[num][2];
      joint_state.position[23] = l_Cali_angles[num][3];   joint_state.position[35] = r_Cali_angles[num][3];
      joint_state.position[24] = l_Cali_angles[num][4];   joint_state.position[36] = r_Cali_angles[num][4];
      joint_state.position[25] = l_Cali_angles[num][5];   joint_state.position[37] = r_Cali_angles[num][5];
      joint_state.position[26] = l_Cali_angles[num][6];   joint_state.position[38] = r_Cali_angles[num][6];
    }
    else
    {
      joint_state.name.resize(14);
      joint_state.position.resize(14);
      joint_state.name[0]="l_arm_pitch";       joint_state.name[1]="l_arm_roll";       joint_state.name[2]="l_arm_yaw";
      joint_state.name[3]="l_forearm_pitch";   joint_state.name[4]="l_forearm_yaw";    joint_state.name[5]="l_hand_roll";
      joint_state.name[6]="l_hand_pitch"; 
      joint_state.name[7]="r_arm_pitch";       joint_state.name[8]="r_arm_roll";        joint_state.name[9]="r_arm_yaw";
      joint_state.name[10]="r_forearm_pitch";   joint_state.name[11]="r_forearm_yaw";     joint_state.name[12]="r_hand_roll";
      joint_state.name[13]="r_hand_pitch";
      for(int i = 0; i < 7; i++)
      {
        joint_state.position[i]   = l_Cali_angles[num][i]*rad2deg;
        joint_state.position[7+i] = r_Cali_angles[num][i]*rad2deg;
      }
    }

    std::cout<<"est_ql = "<<est_q[19]*rad2deg<<"|"<<est_q[21]*rad2deg<<"|"<<est_q[22]*rad2deg<<"\n";
    std::cout<<"est_qr = "<<est_q[26]*rad2deg<<"|"<<est_q[28]*rad2deg<<"|"<<est_q[29]*rad2deg<<"\n";
    
    if (count % 999 == 0)
      step ++;
    
    // std::cout << "step = "<<step<< std::endl;
    
    if (step % 2 == 0)
      num--;
    else
      num++;
    
    count++;
    joint_pub.publish(joint_state);

    // This will adjust as needed per iteration
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "finished" << std::endl;

  return 0;
}
