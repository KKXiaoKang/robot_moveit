#ifndef ARM_TRAJ_HPP
#define ARM_TRAJ_HPP

#include <sensor_msgs/JointState.h>

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

#endif // ARM_TRAJ_HPP
