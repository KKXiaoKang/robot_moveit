#include "trac_ik/MathUtilities.h"



/*!
 * Square a number
 */
double square(double a) {
  return a * a;
}

Mat3 skew(const Vec3 &v){
    Eigen::Matrix3d skew_symmetric;
    skew_symmetric << 0, -v(2), v(1),
                      v(2), 0, -v(0),
                      -v(1), v(0), 0;
    return skew_symmetric;
}

double saturation(const double a, Vec2 limits){
    double lowLim, highLim;
    if(limits(0) > limits(1)){
        lowLim = limits(1);
        highLim= limits(0);
    }else{
        lowLim = limits(0);
        highLim= limits(1);
    }

    if(a < lowLim){
        return lowLim;
    }
    else if(a > highLim){
        return highLim;
    }
    else{
        return a;
    }
}

MatrixXd LieAdjoint(Eigen::MatrixXd X)
{
    assert(X.cols() == 7);
    // Separate state vector into components
    int N = X.cols() - 3;
    Eigen::Matrix3d R = X.block(0, 0, 3, 3);
    Eigen::MatrixXd AdjX = Eigen::MatrixXd::Zero(R.rows() * (N + 1), R.rows() * (N + 1));
    for (int i = 0; i < N + 1; i++)
    {
        AdjX.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
    }
    for (int j = 0; j < N; j++)
    {
        AdjX.block(3 * j + 3, 0, 3, 3) = skew(X.block(0, j + 3, 3, 1)) * R;
    }
    return AdjX;
}

Eigen::Matrix3d LieExp_SO3(const Eigen::Vector3d& w) {
    double theta = w.norm();
    Eigen::Matrix3d A = skew(w);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (theta < 1e-10) {
        return R;
    }
    else {
        R = Eigen::Matrix3d::Identity() + (sin(theta) / theta) * A + ((1 - cos(theta)) / (theta * theta)) * A * A;
    }
    return R;
}

Eigen::Matrix3d LieGamma(const Eigen::Vector3d& w, int n) {
    assert(n >= 0);
    Eigen::Matrix3d R = LieExp_SO3(w);
    Eigen::Matrix3d A = skew(w);
    double theta = w.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d output;

    if (theta < 1e-10) {
        output = (1.0 / tgamma(n + 1)) * I;
        return output;
    }

    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    for (int k = 1; k <= n; k++) {
        S += A.pow(k) / tgamma(k + 1);
    }

    if (n == 0) {
        output = R;
    } 
    else if (n % 2 == 1) { // odd
        output = (1.0 / tgamma(n + 1)) * I + (pow((-1.0), ((n + 1) / 2)) / std::pow(theta, n + 1)) * A * (R - S);
    } 
    else { // even
        output = (1.0 / tgamma(n + 1)) * I + (pow((-1.0), (n / 2)) / std::pow(theta, n)) * (R - S);
    }
    return output;
}

Vec4 QuaterniondExp(Vec3 w)
{
    double nw = w.norm();
    if (nw < 1e-20)
    {
        nw = 1e-20; // Prevents divisions by 0
    }
    Vec4 output;
    output << std::cos(nw / 2.0), std::sin(nw / 2.0) * (w / nw);
    return output;
}

Vec4 Quaternion_Multiply(const Vec4& p, const Vec4& q)
{
    double qw = q(0);
    Vec3 qv = q.tail<3>();

    Mat4 mat;
    mat << qw, -qv.transpose(),
           qv, qw * Eigen::Matrix3d::Identity() - qv * qv.transpose();

    return mat * p;
}

Vec2 Contactsmooth(Vec2 contact, bool a, bool b)
{
    Vec2 output;
    output = contact;
    double delta = 0.005;
    if(a){
        output[0] = min(contact[0]+delta, 1.0);
    }
    else{
        output[0] = max(contact[0]-delta, 0.0);
    }
    if(b){
        output[1] = min(contact[1]+delta, 1.0);
    }
    else{
        output[1] = max(contact[1]-delta, 0.0);
    }
    return output;
}

double min(double a, double b) {
    return a <= b ? a : b;
}
double max(double a, double b) {
    return a >= b ? a : b;
}

Eigen::Matrix4d quat_rate(Eigen::Vector3d v)
{
    Eigen::Matrix4d mat;
    mat << 0, -v(0), -v(1), -v(2),
        v(0),     0,  v(2), -v(1),
        v(1), -v(2),     0,  v(0),
        v(2),  v(1), -v(0),     0;
    return mat;
}

Eigen::MatrixXd getPhi(const Eigen::Vector3d& w, 
                    const Eigen::Vector3d& a, 
                    const Eigen::Vector3d& g, 
                    const Eigen::MatrixXd& R, 
                    const Eigen::Vector3d& v_pred, 
                    const Eigen::Vector3d& p_pred, 
                    const Eigen::Vector3d &dL_pred, 
                    const Eigen::Vector3d &dR_pred, 
                    const double& dt)
  {
    Eigen::Matrix3d ax = skew(a);
    Eigen::Matrix3d gx = skew(g);
    Eigen::MatrixXd Phi = Eigen::Matrix<double, 21, 21>::Identity();
    Mat3 G0 = LieGamma(w*dt,0);
    Mat3 G1 = LieGamma(w*dt,1);
    Mat3 G2 = LieGamma(w*dt,2);

    Eigen::Vector3d phi = w * dt;
    Eigen::Matrix3d phix = skew(phi);
    double theta = phi.norm();
    Eigen::Matrix3d Psi1, Psi2;
    if (w.norm() > 1e-6) {
      Psi1 = ax * LieGamma(-phi, 2)
          + ((sin(theta) - theta * cos(theta)) / pow(theta, 3)) * (phix * ax)
          - ((cos(2 * theta) - 4 * cos(theta) + 3) / (4 * pow(theta, 4))) * (phix * ax * phix)
          + ((4 * sin(theta) + sin(2 * theta) - 4 * theta * cos(theta) - 2 * theta) / (4 * pow(theta, 5))) * (phix * ax * phix * phix)
          + (((pow(theta, 2) - 2 * theta * sin(theta) - 2 * cos(theta) + 2) / (2 * pow(theta, 4)))) * (phix * phix * ax)
          - ((6 * theta - 8 * sin(theta) + sin(2 * theta)) / (4 * pow(theta, 5))) * (phix * phix * ax * phix)
          + ((2 * pow(theta, 2) - 4 * theta * sin(theta) - cos(2 * theta) + 1) / (4 * pow(theta, 6))) * (phix * phix * ax * phix * phix);

      Psi2 = ax * LieGamma(-phi, 3)
          - ((theta * sin(theta) + 2 * cos(theta) - 2) / pow(theta, 4)) * (phix * ax)
          - ((6 * theta - 8 * sin(theta) + sin(2 * theta)) / (8 * pow(theta, 5))) * (phix * ax * phix)
          - ((2 * pow(theta, 2) + 8 * theta * sin(theta) + 16 * cos(theta) + cos(2 * theta) - 17) / (8 * pow(theta, 6))) * (phix * ax * phix * phix)
          + (((pow(theta, 3) + 6 * theta - 12 * sin(theta) + 6 * theta * cos(theta)) / (6 * pow(theta, 5)))) * (phix * phix * ax)
          - ((6 * pow(theta, 2) + 16 * cos(theta) - cos(2 * theta) - 15) / (8 * pow(theta, 6))) * (phix * phix * ax * phix)
          + ((4 * pow(theta, 3) + 6 * theta - 24 * sin(theta) - 3 * sin(2 * theta) + 24 * theta * cos(theta)) / (24 * pow(theta, 7))) * (phix * phix * ax * phix * phix);
    } else {
      Psi1 = 0.5 * ax;
      Psi2 = (1.0 / 6.0) * ax;
    }
    Phi.block<3, 3>(3, 0) = gx * dt;
    Phi.block<3, 3>(6, 0) = 0.5 * gx * dt * dt;
    Phi.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
    Phi.block<3, 3>(0, 15) = -R * G1 * dt;
    Phi.block<3, 3>(3, 15) = -skew(v_pred) * R * G1 * dt + R * Psi1 * dt * dt;
    Phi.block<3, 3>(6, 15) = -skew(p_pred) * R * G1 * dt + R * Psi2 * pow(dt, 3);
    Phi.block<3, 3>(9, 15) = -skew(dL_pred) * R * G1 * dt;
    Phi.block<3, 3>(12, 15)= -skew(dR_pred) * R * G1 * dt;
    Phi.block<3, 3>(3, 18) = -R * LieGamma(w * dt, 1) * dt;
    Phi.block<3, 3>(6, 18) = -R * LieGamma(w * dt, 2) * dt * dt;

    return Phi;
  }

  Eigen::MatrixXd getFk( const Eigen::Vector3d& a, 
                    const Eigen::Vector3d& g, 
                                    const Eigen::MatrixXd& R, 
                                    const Eigen::Vector3d& v, 
                                    const Eigen::Vector3d& p, 
                                    const Eigen::Vector3d& dL, 
                                    const Eigen::Vector3d& dR, 
                                    const double& dt)
  {
    Eigen::MatrixXd Fc(21, 21);
    Fc.setZero();
    Fc.block(0, 0, 9, 9) << Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3),
        skew(g), Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3),
        Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3);
    Fc.block(0, Fc.cols() - 6, 15, 6) << -R, Eigen::MatrixXd::Zero(3, 3),
        -skew(v) * R, -R,
        -skew(p) * R, Eigen::MatrixXd::Zero(3, 3),
        -skew(dL) * R, Eigen::MatrixXd::Zero(3, 3),
        -skew(dR) * R, Eigen::MatrixXd::Zero(3, 3);
    // Discretize
    Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(21, 21) + Fc * dt;
    return Fk;
  }