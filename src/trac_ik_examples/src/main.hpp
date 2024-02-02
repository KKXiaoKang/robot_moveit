#ifndef MAIN_HPP
#define MAIN_HPP

#include <iostream>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>
//trajectory
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>

using namespace std;
using namespace Eigen;
using namespace KDL;

#define PI 3.1415926
// reading from files, cannot delete the files below
#define ReadURDFModel       "/home/u20-2/codes/github/manipulator/src/ros_robotics/urdf/arm_er20.urdf"
#define ReadRawAnglesData   "/home/u20-2/codes/github/manipulator/src/stage_first/src/RawAngles.txt"
// writing to files, the files are removed once you run the main function
#define InterPolationData   "//home/u20-2/codes/github/manipulator/src/stage_first/src/InterPolation.txt"
#define CalibratedAnglesData "/home/u20-2/codes/github/manipulator/src/stage_first/src/CalibratedAngles.txt"

std::vector<std::vector<double> > angles(7);
std::vector<std::vector<double> > Inter_angles(7);
std::vector<std::vector<double> > Calibrated_angles(6);
void Interpolation(vector<vector<double> > angles);
void ReadCalibratedAng();
void ReadRawAng();
Frame setoffset(KDL::Frame interp_pos, std::vector<double> offset);

#endif // MAIN_HPP
