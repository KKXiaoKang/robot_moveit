#include "trac_ik/orientation_tools.h"

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
Vec3 quatToRPY(const Quat& q) {
  Vec3 rpy;
  double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  rpy(2) =
      std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                 square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
  rpy(1) = std::asin(as);
  rpy(0) =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                 square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
  return rpy;
}

Quat quatProduct(const Quat& q1, const Quat& q2) {
  double r1 = q1[0];
  double r2 = q2[0];
  Vec3 v1(q1[1], q1[2], q1[3]);
  Vec3 v2(q2[1], q2[2], q2[3]);

  double r = r1 * r2 - v1.dot(v2);
  Vec3 v = r1 * v2 + r2 * v1 + v1.cross(v2);
  Quat q;
  q<<r, v[0], v[1], v[2];
  return q;
}

Quat rpyToQuat(const Vec3& rpy) {
  Mat3 R = rpyToRotMat(rpy);
  Quat q = rotationMatrixToQuaternion(R);
  return q;
}

Mat3 rpyToRotMat(const Vec3& v) {
  Mat3 m = coordinateRotation(CoordinateAxis::X, v[0]) *
            coordinateRotation(CoordinateAxis::Y, v[1]) *
            coordinateRotation(CoordinateAxis::Z, v[2]);
  return m;
}

/*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 */
Mat3 quaternionToRotationMatrix(const Quat& q) {
  double e0 = q(0);
  double e1 = q(1);
  double e2 = q(2);
  double e3 = q(3);
  Mat3 R;
  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
      2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
      2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
//  std::cout<<"R"<<std::endl<<R<<std::endl;
  R.transposeInPlace();
//        std::cout<<"RT"<<std::endl<<R<<std::endl;
  return R;
}

Quat rotationMatrixToQuaternion( const Mat3& r1) {
  Quat q;
  Mat3 r = r1.transpose();
  double tr = r.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (r(2, 1) - r(1, 2)) / S;
    q(2) = (r(0, 2) - r(2, 0)) / S;
    q(3) = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    double S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q(0) = (r(2, 1) - r(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (r(0, 1) + r(1, 0)) / S;
    q(3) = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    double S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q(0) = (r(0, 2) - r(2, 0)) / S;
    q(1) = (r(0, 1) + r(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (r(1, 2) + r(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q(0) = (r(1, 0) - r(0, 1)) / S;
    q(1) = (r(0, 2) + r(2, 0)) / S;
    q(2) = (r(1, 2) + r(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

Vec3 rotationMatrixToRPY(const Mat3& R) {
  Quat q = rotationMatrixToQuaternion(R);
  Vec3 rpy = quatToRPY(q);
  return rpy;
}



Mat3 coordinateRotation(CoordinateAxis axis, double theta) {
  double s = std::sin(theta);
  double c = std::cos(theta);

  Mat3 R;
  if (axis == CoordinateAxis::X) {
    R << 1, 0, 0, 0, c, s, 0, -s, c;
  } else if (axis == CoordinateAxis::Y) {
    R << c, 0, -s, 0, 1, 0, s, 0, c;
  } else if (axis == CoordinateAxis::Z) {
    R << c, s, 0, -s, c, 0, 0, 0, 1;
  }

  return R;
}

Vec3 rotMatToExp(const Mat3& rm){
    double cosValue = rm.trace()/2.0-1/2.0;
    if(cosValue > 1.0f){
        cosValue = 1.0f;
    }else if(cosValue < -1.0f){
        cosValue = -1.0f;
    }

    double angle = acos(cosValue);
    Vec3 exp;
    if (fabs(angle) < 1e-5){
        exp=Vec3(0,0,0);
    }
    else if (fabs(angle - M_PI) < 1e-5){
        exp = angle * Vec3(rm(0,0)+1, rm(0,1), rm(0,2)) / sqrt(2*(1+rm(0, 0)));
    }
    else{
        exp=angle/(2.0f*sin(angle))*Vec3(rm(2,1)-rm(1,2),rm(0,2)-rm(2,0),rm(1,0)-rm(0,1));
    }
    return exp;
}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
  {

    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    Eigen::Vector3d retVec(3);
    retVec << x,y,z;
    return retVec;
  }

  Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
  {
      // Calculate rotation about x axis
      Eigen::Matrix3d R_x; 
      Eigen::Matrix3d R_y; 
      Eigen::Matrix3d R_z; 
      Eigen::Matrix3d R; 
      R_x <<1,     0,              0,
            0,     cos(theta[0]),  -sin(theta[0]),
            0,     sin(theta[0]),  cos(theta[0]);
      // Calculate rotation about y axis
      R_y <<cos(theta[1]),      0,    sin(theta[1]),
                0,              1,    0,
                -sin(theta[1]), 0,    cos(theta[1]);
      // Calculate rotation about z axis
      R_z <<cos(theta[2]),   -sin(theta[2]),     0,
            sin(theta[2]),   cos(theta[2]),      0,
            0,               0,                  1;
      // Combined rotation matrix
      R = R_z * R_y * R_x;
      return R;
  }

  Quat negativeYaw_Quat2(Quat &quat)
  {
    Vec3 Angu_ini = quatToRPY(quat);
    Angu_ini[0] = 0;
    Angu_ini[1] = 0;
    Quat quat_ = rpyToQuat(-Angu_ini);
    return quat_;
  }

  Quat negativeYaw_R2Q(Mat3 &Rbody)
  {
    Vec3 Angu_ini = rotationMatrixToRPY(Rbody);
    Angu_ini[0] = 0;
    Angu_ini[1] = 0;
    Quat quat_ = rpyToQuat(-Angu_ini);
    return quat_;
  }
  
Mat3 negativeYaw_R2R(Mat3 &Rbody)
{
    Vec3 Angu_ini = rotationMatrixToRPY(Rbody);
    Angu_ini[0] = 0;
    Angu_ini[1] = 0;
    Mat3 Rbody_ = rpyToRotMat(-Angu_ini);
    return Rbody_;
}

Mat3 negativeYaw_Q2R(Quat &quat)
{
    Vec3 Angu_ini = quatToRPY(quat);
    Angu_ini[0] = 0;
    Angu_ini[1] = 0;
    Mat3 Rbody_ = rpyToRotMat(-Angu_ini);
    return Rbody_;
}

Quat removeYaw_Q2Q(Quat &quat)
{
    Vec3 Angu_ini = quatToRPY(quat);
    Angu_ini[2] = 0;
    Quat quat_ = rpyToQuat(Angu_ini);
    return quat_;
}

Quat quatunify(const Quat &quat)
{
  Quat quat_;
  quat_ = quat;
  if(quat_[0] < 0) 
  {
    quat_ *= -1;
  }
  return quat_;
}

double anglelimit_pi(double angle)
{
  double out = fmod(angle + M_PI, 2*M_PI) - M_PI;
  return out;
}

void AddYaw4ik(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &a, Vec4 &posY)
{
  double torsoR_yaw = posY[3];
  Vec3 pos = posY.segment(0,3);
  Vec3 rpy = quatToRPY(q.segment(0,4));
  rpy[2] += torsoR_yaw;
  q.segment(0,4) = rpyToQuat(rpy);
  q.segment(0,4) = quatunify(q.segment(0,4));
  Mat3 R_T = rpyToRotMat(Vec3(0., 0., torsoR_yaw)).transpose();
  q.segment(4,3) = R_T*q.segment(4,3) + pos;
  v.segment(0,3) = R_T*v.segment(0,3);
  v.segment(3,3) = R_T*v.segment(3,3);
  a.segment(0,3) = R_T*a.segment(0,3);
  a.segment(3,3) = R_T*a.segment(3,3);
}

void prevRemoveYaw4ik(Eigen::VectorXd &preq, Eigen::VectorXd &prev, Vec4 &posY)
{
  double torsoR_yaw = posY[3];
  Vec3 pos = posY.segment(0,3);
  Vec3 rpy = quatToRPY(preq.segment(0,4));
  rpy[2] -= torsoR_yaw;
  preq.segment(0,4) = rpyToQuat(rpy);
  Mat3 Rbody = rpyToRotMat(Vec3(0., 0., torsoR_yaw));
  Vec3 torso = preq.segment(4,3) - pos;
  preq.segment(4,3) = Rbody*torso;
  preq.segment(0,4) = quatunify(preq.segment(0,4));
  // prev.segment(0,3) = Rbody*prev.segment(0,3);
  prev.segment(3,3) = Rbody*prev.segment(3,3);
}

Vec4 RemoveYaw4ik(Eigen::VectorXd &x_d, Eigen::VectorXd &xd_d, Eigen::VectorXd &xdd_d)
{
  Vec4 posY;
  double torsoR_yaw = anglelimit_pi(x_d[2]);
  Vec3 lf, rf, torso, lfv, rfv, torsov, lfa, rfa, torsoa, lfv_ang, rfv_ang, lfa_ang, rfa_ang;
  torso = x_d.segment(3, 3);
  lf = x_d.segment(6, 3);
  lf = x_d.segment(9, 3);
  rf = x_d.segment(15, 3);
  torsov = xd_d.segment(3, 3);
  lfv_ang = xd_d.segment(6, 3);
  lfv = xd_d.segment(9, 3);
  rfv_ang = xd_d.segment(12, 3);
  rfv = xd_d.segment(15, 3);
  torsoa = xdd_d.segment(3, 3);
  lfa_ang = xdd_d.segment(6, 3);
  lfa = xdd_d.segment(9, 3);
  rfa_ang = xdd_d.segment(12, 3);
  rfa = xdd_d.segment(15, 3);
  Mat3 Rbody = rpyToRotMat(Vec3(0., 0., torsoR_yaw));
  lf = lf - torso;
  rf = rf - torso;

  posY<<torso, torsoR_yaw;

// yaw angle torso/lf/rf
  x_d[2] = anglelimit_pi(x_d[2] - torsoR_yaw);
  x_d[8] = anglelimit_pi(x_d[8] - torsoR_yaw);
  x_d[14] = anglelimit_pi(x_d[14] - torsoR_yaw);

// pos torso/lf/rf
  x_d.segment(3, 3) = Vec3(0, 0, 0);
  x_d.segment(9, 3) = Rbody*lf;
  x_d.segment(15, 3) = Rbody*rf;
  xd_d.segment(3, 3) = Rbody*torsov;
  xd_d.segment(6, 3) = Rbody*lfv_ang;
  xd_d.segment(9, 3) = Rbody*lfv;
  xd_d.segment(12, 3) = Rbody*rfv_ang;
  xd_d.segment(15, 3) = Rbody*rfv;
  xdd_d.segment(3, 3) = Rbody*torsoa;
  xdd_d.segment(6, 3) = Rbody*lfa_ang;
  xdd_d.segment(9, 3) = Rbody*lfa;
  xdd_d.segment(12, 3) = Rbody*rfa_ang;
  xdd_d.segment(15, 3) = Rbody*rfa;

  return posY;
}

double RemoveYaw4pb(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &vd)
{
  Vec3 rpy = quatToRPY(q.segment(0,4));
  double yaw = rpy[2];
  rpy[2] = 0.0;
  q.segment(0,4) = rpyToQuat(rpy);
  q.segment(0,4) = quatunify(q.segment(0,4));
  Mat3 Rbody = rpyToRotMat(Vec3(0., 0., yaw));
  q.segment(4,3) = Rbody*q.segment(4,3);
  v.segment(0,3) = Rbody*v.segment(0,3);
  v.segment(3,3) = Rbody*v.segment(3,3);
  vd.segment(0,3) = Rbody*vd.segment(0,3);
  vd.segment(3,3) = Rbody*vd.segment(3,3);
  return yaw;
}

double RemoveYaw4pb(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &vd, double &yaw)
{
  Vec3 rpy = quatToRPY(q.segment(0,4));
  rpy[2] -= yaw;
    //TODO：
    rpy[2] = anglelimit_pi(rpy[2]);

  q.segment(0,4) = rpyToQuat(rpy);
  q.segment(0,4) = quatunify(q.segment(0,4));
  Mat3 Rbody = rpyToRotMat(Vec3(0., 0., yaw));
  q.segment(4,3) = Rbody*q.segment(4,3);
  v.segment(0,3) = Rbody*v.segment(0,3);
  v.segment(3,3) = Rbody*v.segment(3,3);
  vd.segment(0,3) = Rbody*vd.segment(0,3);
  vd.segment(3,3) = Rbody*vd.segment(3,3);
  return rpy[2];
}

void AddYaw4pb(Eigen::VectorXd &q, Eigen::VectorXd &v, Eigen::VectorXd &vd, double &yaw)
{
  Vec3 rpy = quatToRPY(q.segment(0,4));
  rpy[2] += yaw;
  q.segment(0,4) = rpyToQuat(rpy);
  q.segment(0,4) = quatunify(q.segment(0,4));
  Mat3 R_T = rpyToRotMat(Vec3(0., 0., yaw)).transpose();
  q.segment(4,3) = R_T*q.segment(4,3);
  v.segment(0,3) = R_T*v.segment(0,3);
  v.segment(3,3) = R_T*v.segment(3,3);
  vd.segment(0,3) = R_T*vd.segment(0,3);
  vd.segment(3,3) = R_T*vd.segment(3,3);
}

Vec4 RemoveYaw4se(Eigen::VectorXd &r_vec_, Vec3 &p_st, const Quat &quat)
{
  Vec4 posY;
  posY.segment(0, 3) = r_vec_.segment(0, 3);
  Vec3 rpy = quatToRPY(quat);
  posY[3] = rpy[2];
  p_st -= r_vec_.segment(0, 3);
  Mat3 Rbody = rpyToRotMat(Vec3(0., 0., rpy[2]));

  r_vec_.segment(0, 3) = Vec3(0, 0, 0);
  r_vec_.segment(3, 3) = Rbody*r_vec_.segment(3, 3);
  p_st = Rbody*p_st;

  return posY;
}

Vec4 AddYaw4se(Eigen::VectorXd &r_vec_, Vec3 &p_st, const Vec4 &posY)
{
  Mat3 R_T = rpyToRotMat(Vec3(0., 0., posY[3])).transpose();
  Vec3 pos = posY.segment(0, 3);

  r_vec_.segment(0, 3) = R_T*r_vec_.segment(0, 3) + pos;
  r_vec_.segment(3, 3) = R_T*r_vec_.segment(3, 3);

  p_st = R_T*p_st;
  p_st += pos;

  return posY;
}