#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/Float64.h"
#include "utils.h"
#include "main.hpp"


double joint_positions[6] = {0};
bool get_states = false;
ros::Time current_time;
ros::Time initial_time;
bool get_initial_time = false;
void ChatterCallBack_JointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
//  std::cout << "arm callback" << std::endl;
  for (int i = 0; i < 6;i++) {
    joint_positions[i] = msg->position[i];
  }

//  std::cout << "current joint positions: " << std::endl;
  for (int i = 0; i < 6;i++) {
//    std::cout << joint_positions[i]*180/3.1415 << std::endl;
  }
  get_states = true;
}


int main(int argc, char** argv)
{
  remove(InterPolationData);
  remove(CalibratedAnglesData);

  std::cout << "enter main function" << std::endl;
  ros::init(argc, argv, "main");
  ros::NodeHandle masterNode;

  ros::Publisher J1_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j1_position_controller/command", 1);
  ros::Publisher J2_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j2_position_controller/command", 1);
  ros::Publisher J3_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j3_position_controller/command", 1);
  ros::Publisher J4_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j4_position_controller/command", 1);
  ros::Publisher J5_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j5_position_controller/command", 1);
  ros::Publisher J6_cmd_pub = masterNode.advertise<std_msgs::Float64>("/rrbot/j6_position_controller/command", 1);

  std::cout << "publisher defined" << std::endl;
  ros::Subscriber joint_states_sub = masterNode.subscribe("/rrbot/joint_states",1,ChatterCallBack_JointStates);
  ros::Rate loop_rate(25);
  std::cout << "subscriber defined" << std::endl;

  std_msgs::Float64 joint_cmd1;
  std_msgs::Float64 joint_cmd2;
  std_msgs::Float64 joint_cmd3;
  std_msgs::Float64 joint_cmd4;
  std_msgs::Float64 joint_cmd5;
  std_msgs::Float64 joint_cmd6;
  std::vector<double> joint_cmd(6, 0.0);

  KDL::Tree my_tree;
//  KDL::TreeFkSolverPosFull_recursive fk_solver_();
  if (!kdl_parser::treeFromFile(ReadURDFModel, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
     }
  int tree_nj = my_tree.getNrOfJoints();
  int tree_ns = my_tree.getNrOfSegments();
  std::cout << "number of tree joints: " << tree_nj << std::endl;
  std::cout << "number of tree segments: " << tree_ns << std::endl;

//kdl tree to kdl chain
  KDL::Chain my_chain;
  bool exit_value;
  exit_value = my_tree.getChain("base_link", "link6", my_chain);
  int chain_nj = my_chain.getNrOfJoints();
  int chain_ns = my_chain.getNrOfSegments();
  std::cout << "exit_value: " << exit_value << std::endl;
  std::cout << "number of chain joints: " << chain_nj << std::endl;   //
  std::cout << "number of chain segments: " << chain_ns << std::endl; //

  // define kinematic solvers
  using namespace KDL;
  int n = my_chain.getNrOfJoints();
  ChainFkSolverPos_recursive fwdkin(my_chain);
  KDL::ChainJntToJacSolver JcbSolver(my_chain);
  ChainIkSolverPos_LMA iksolver(my_chain);
  JntArray q_init(n);
  for (int i = 0; i < chain_nj; i++) {
      q_init.data[i] = 0.0;
//      std::cout<< "init joint: " << q_init.data[i] << std::endl;
  }

  KDL::Frame init_pos;
  fwdkin.JntToCart(q_init, init_pos);
//  std::cout << "current frame: " << init_pos << std::endl;
  KDL::Frame F2(init_pos);


/**
 * read the raw angle from teaching
 * Interpolate the raw angle
 */
  ReadRawAng();
  Interpolation(angles);

/// remove the weird angles (last 10)
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j<10; j++) {
      Inter_angles[i].pop_back();
    }
  }

  /**
   * @brief Interpolated joint angles to cartesion position and then to new joint angles with offset
   * @ref https://blog.csdn.net/lordofrobots/article/details/77949111
   *
   */
  std::vector<double> offset(3, 0.0);
  /**
   * We can get the offset position via input
   */
  printf("Enter the offset x: ");
  scanf("%lf", &offset.at(0));
  printf("Enter the offset y: ");
  scanf("%lf", &offset.at(1));
  printf("Enter the offset theta: ");
  scanf("%lf", &offset.at(2));

//  cout<<"Inter_angles[0].size()="<<Inter_angles[0].size()<<endl;
  for (int i = 0; i < Inter_angles[0].size(); i++) {
    KDL::Frame interp_pos;
    JntArray current_Interang(chain_nj);

    current_Interang(0) = Inter_angles[0][i];   current_Interang(1) = Inter_angles[1][i];
    current_Interang(2) = Inter_angles[2][i];   current_Interang(3) = Inter_angles[3][i];
    current_Interang(4) = Inter_angles[4][i];   current_Interang(5) = Inter_angles[5][i];

    bool fwdkin_status = fwdkin.JntToCart(current_Interang, interp_pos);
    if(fwdkin_status >= 0) {
//      std::cout<<"interp_pos="<<interp_pos<<std::endl;
//      printf("%s \n", "Success, thanks KDL!");
    }
    else {
      printf("%s \n", "Error:could not calculate forward kinematics : ");
    }

    // gives the offset to Cartesian position
    Frame interp_pos_ = setoffset(interp_pos, offset);
//    std::cout<<"interp_pos "<<interp_pos <<endl;
//    std::cout<<"interp_pos_"<<interp_pos_<<endl;

    JntArray calibrated_angles(tree_nj);
    JntArray jointguesspositions(tree_nj);
    // defined jointguesspositions
    jointguesspositions(0) = Inter_angles[0][i];
    jointguesspositions(1) = Inter_angles[1][i];
    jointguesspositions(2) = Inter_angles[2][i];
    jointguesspositions(3) = Inter_angles[3][i];
    jointguesspositions(4) = Inter_angles[4][i];
    jointguesspositions(5) = Inter_angles[5][i];
//    cout<<"Inter_angles[0]="<<Inter_angles[0][i]<<"\t"<<Inter_angles[1][i]<<"\t"<<Inter_angles[2][i]<<endl;
    ///
    /// \brief get the corrected joint angles
    /// @param calibrated_angles is the destination.
    ///
    static struct timespec t0_sol, t1_sol;
    clock_gettime(CLOCK_MONOTONIC, &t0_sol);
    bool iksolver_status = iksolver.CartToJnt(jointguesspositions, interp_pos_, calibrated_angles);
    clock_gettime(CLOCK_MONOTONIC, &t1_sol);
    #if 1
      printf("Solve(ms): %f\n",
             TIME_DIFF(t0_sol, t1_sol) * 1000.0);
    #endif
    if (iksolver_status>=0) {
//      for (int i = 0; i < tree_nj; i++) {
//        std::cout<<jointguesspositions(i)<<"\t";
//      }
//      for (int i = 0; i < tree_nj; i++) {
//        std::cout<<calibrated_angles(i)<<"\t";
//      }
//      cout<<endl;
//      printf("%s \n", "Success KDL!");
      /// write the calibrated angles to file CalibratedAngles.txt
        std::ofstream WriteCaliAng;
        WriteCaliAng.open(CalibratedAnglesData, ios::app);

        for (unsigned i = 0; i < calibrated_angles.data.size(); i++) {
          WriteCaliAng << calibrated_angles.data[i]*180/PI << "\t";
          Calibrated_angles[i].push_back(calibrated_angles.data[i]);
        }
        WriteCaliAng <<"\n";
        WriteCaliAng.close();
    }
  }

  /***
   * verify the calc position and offset position
   */
  KDL::Frame Inpos_test,Outpos_test;
  JntArray In_ang(chain_nj);
  JntArray Out_ang(chain_nj);
  int num_t = 10;
  In_ang(0) = Inter_angles[0][num_t];   In_ang(1) = Inter_angles[1][num_t];
  In_ang(2) = Inter_angles[2][num_t];   In_ang(3) = Inter_angles[3][num_t];
  In_ang(4) = Inter_angles[4][num_t];   In_ang(5) = Inter_angles[5][num_t];
  Out_ang(0) = Calibrated_angles[0][num_t];   Out_ang(1) = Calibrated_angles[1][num_t];
  Out_ang(2) = Calibrated_angles[2][num_t];   Out_ang(3) = Calibrated_angles[3][num_t];
  Out_ang(4) = Calibrated_angles[4][num_t];   Out_ang(5) = Calibrated_angles[5][num_t];

  bool fwdkinIn_status = fwdkin.JntToCart(In_ang, Inpos_test);
  if(fwdkinIn_status >= 0) {
      std::cout<<"Original XYZ="<<Inpos_test.p<<std::endl;
//      printf("%s \n", "Success, thanks KDL!");
  }
  else {
    printf("%s \n", "Error:could not calculate forward kinematics : ");
  }

  bool fwdkinOut_status = fwdkin.JntToCart(Out_ang, Outpos_test);
  if(fwdkinOut_status >= 0) {
      std::cout<<"Calibrated XYZ="<<Outpos_test.p<<std::endl;
//      printf("%s \n", "Success, thanks KDL!");
  }
  else {
    printf("%s \n", "Error:could not calculate forward kinematics : ");
  }
  cout<<"Offset Verify ="<<Outpos_test.p-Inpos_test.p<<endl;

//  cout<<"Inter_angles[0].size()"<<Inter_angles[0].size()<<endl;
  for (int i = 0; i < Calibrated_angles[0].size(); i++) {

//    joint_cmd1.data = Inter_angles[0][i];
//    joint_cmd2.data = Inter_angles[1][i];
//    joint_cmd3.data = Inter_angles[2][i];
//    joint_cmd4.data = Inter_angles[3][i];
//    joint_cmd5.data = Inter_angles[4][i];
//    joint_cmd6.data = Inter_angles[5][i];

    joint_cmd1.data = Calibrated_angles[0][i];
    joint_cmd2.data = Calibrated_angles[1][i];
    joint_cmd3.data = Calibrated_angles[2][i];
    joint_cmd4.data = Calibrated_angles[3][i];
    joint_cmd5.data = Calibrated_angles[4][i];
    joint_cmd6.data = Calibrated_angles[5][i];
    J1_cmd_pub.publish(joint_cmd1);
    J2_cmd_pub.publish(joint_cmd2);
    J3_cmd_pub.publish(joint_cmd3);
    J4_cmd_pub.publish(joint_cmd4);
    J5_cmd_pub.publish(joint_cmd5);
    J6_cmd_pub.publish(joint_cmd6);

    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "finished" << std::endl;
  return 0;
}


void ReadRawAng()
{
  //  read joint angles();
    std::ifstream file( ReadRawAnglesData, std::ios::in );

    if( !file )
        std::cerr << "Cannot open " << std::endl;

  std::string jname1, jname2, jname3, jname4, jname5, jname6, jname_aux;
  double jang1, jang2, jang3, jang4, jang5, jang6, jang_aux;

  while( !file.eof() )
  {
      file >> jname1 >> jang1 >> jname2 >> jang2 >> jname3 >> jang3 >> jname4 >> jang4 >> jname5 >> jang5 >> jname6 >> jang6 >> jname_aux >> jang_aux;
      angles[0].push_back(jang1/180*PI);      angles[1].push_back(jang2/180*PI);      angles[2].push_back(jang3/180*PI);      angles[3].push_back(jang4/180*PI);
      angles[4].push_back(jang5/180*PI);      angles[5].push_back(jang6/180*PI);      angles[6].push_back(jang_aux/180*PI);
//      std::cout << jname_aux << ": " << jang_aux << std::endl;
  }
  file.close();
//  cout<<"read finished!"<<endl;
}

void Interpolation(vector<vector<double> > angles)
{
  /**
   * Interpolation
   */

  //  std::vector<std::vector<double> > Inter_angles(chain_nj);
    std::vector<double> xx,x;
    int size_0 = angles[0].size();
//    cout<<"size_0 defined!"<<endl;
//    cout<<"angles end:"<<angles[0][size_0-1]*180/PI<<std::endl;
//    cout<<"angles end:"<<angles[0][size_0-2]*180/PI<<std::endl;

    double step = 0.1;
    for (int i = 0; i < size_0; i++) {
      x.push_back(i);
    }
    double value = x[0];
    int num = size_0 / step;
    for (double i = 0; i <= num; i++) {
      xx.push_back(value);
      value = value + step;
    }
    cout<<"xx defined!"<<endl;
    int size_xx = xx.size();

    vector<double> dx,dy;
    for (int i = 0; i < size_0 - 1; i++) {
      double temp_dx = x[i + 1] - x[i];
      dx.push_back(temp_dx);
    }
    MatrixXd H = MatrixXd::Random(size_0, size_0);
    for (int i = 0; i < size_0; i++) {
      for (int j = 0; j < size_0; j++) {
        H(i, j) = 0;
      }
    }
    VectorXd Y(size_0);
    for (int i = 0; i < size_0; i++) {
      Y(i) = 0;
    }
    VectorXd M(size_0);
    for (int i = 0; i < size_0; i++) {
      M(i) = 0;
    }

    H(0, 0) = 1;
    H(size_0 - 1, size_0 - 1) = 1;
    for (int i = 1; i < size_0 - 1; i++) {
        H(i, i - 1) = dx[i - 1];
        H(i, i) = 2 * (dx[i - 1] + dx[i]);
        H(i, i + 1) = dx[i];
      }
    /// ************** ///
    for (int k_ = 0; k_ < 7; k_++) {
      for (int i = 0; i < size_0 - 1; i++) {
        double temp_dy = angles[k_][i+1] - angles[k_][i];
        dy.push_back(temp_dy);
      }
      for (int i = 0; i < size_0 - 1; i++) {
        Y(i) = 3 * (dy[i] / dx[i] - dy[i - 1] / dx[i - 1]);
      }
      M = H.colPivHouseholderQr().solve(Y);
      vector<double> ai, bi, ci, di;
      for (int i = 0; i < size_0 - 1; i++) {
        ai.push_back(angles[k_][i]);
        di.push_back((M(i + 1) - M(i)) / (3 * dx[i]));
        bi.push_back(dy[i] / dx[i] - dx[i] * (2 * M(i) + M(i + 1)) / 3);
        ci.push_back(M(i));
      }
      vector<int> x_, xx_;
      for (int i = 0; i < size_0; i++) {
        int temp = x[i] / 0.1;
        x_.push_back(temp);
      }
      for (int i = 0; i < size_xx; i++) {
        int temp = xx[i] / 0.1;
        xx_.push_back(temp);
      }
  //    cout<<"M defined!"<<endl;

      for (int i = 0; i < size_xx; i++) {
        int k = 0;
        for (int j = 0; j < size_0 - 1; j++) {
          if (xx_[i] >= x_[j] && xx_[i] < x_[j + 1]) {
            k = j;
            break;
          }
          else if (xx[i] == x[size_0 - 1]) {
            k = size_0 - 1;
          }
        }
        //yy(i) = y[i] + bi(k) * (xx[i] - x[k]) + 1 / 2.0 * M(i) * pow((xx[i] - x[k]) , 2) + di(k) * pow((xx[i] - x[k]),3);
        double temp = ai[k] + bi[k] * (xx[i] - x[k]) + M(k) * pow((xx[i] - x[k]), 2) + di[k] * pow((xx[i] - x[k]), 3);
        Inter_angles[k_].push_back(temp);
      }

//      cout<<"start write!"<<endl;

      std::ofstream output;
        output.open(InterPolationData, ios::app);
        for (unsigned i = 0; i < size_xx; i++) {
          output << xx[i] << '\t' << Inter_angles[k_][i]*180/PI << std::endl;
        }
      output.close();
      ai.clear(); bi.clear(); ci.clear(); di.clear(); x_.clear(); xx_.clear(); dy.clear();
    }
}

Frame setoffset(KDL::Frame interp_pos, std::vector<double> offset)
{
  /**
   * @brief get the Frame vector and rotation
   * @param offset is the user-defined input offset (x, y, theta);
   * @ref http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29
   */
  Vector origin = interp_pos.p;
  double origin_x = origin.x();

  // set the offset position
  double offset_x, offset_y, offset_theta;
  offset_x=offset.at(0);
  offset_y=offset.at(1);
  offset_theta=offset.at(2);

//  offset_x = 0.05;
//  offset_y = 0.05;

  Vector vector=Vector(offset_x+origin_x, offset_y+origin.y(), origin.z());
  Rotation rot = interp_pos.M;  // rotation [3x3]
  Frame calibrated_angles = Frame(rot, vector);

  return calibrated_angles;
}
