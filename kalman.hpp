#pragma once

#include <Eigen/Dense>

// reference: https://www.kalmanfilter.net/multiSummary.html fuck http://ros-developer.com/2019/04/10/kalman-filter-explained-with-python-code-from-scratch/
// Everything is fitted for 2d tracking

class Kalman{
public:
    static const int    nx = 4,
                        nu = 2,
                        nz = 2, NUM_VARS = 1;

    //  Actual state
    Eigen::Matrix<double, nx, 1> Xn;
    // Output state
    Eigen::Matrix<double, nz, 1> Zn;
    // State transition matrix
    Eigen::Matrix<double, nx, nx> F;
    // Control matrix
    Eigen::Matrix<double, nx, nu> G;
    // Covariance matrix
    Eigen::Matrix<double, nx, nx> P;
    // Process covariance matrix
    Eigen::Matrix<double, nx, nz> Q;
    // Uncertainty matrix
    Eigen::Matrix<double, nz, nz> R;
    // Observation matrix
    Eigen::Matrix<double, nz, nx> H;
    // Kalman gain matrix
    Eigen::Matrix<double, nx, nz> K;


    // Variable indexes
    static const int iX  = 0; // X position
    static const int iY  = 1; // Y position
    static const int idX = 2; // X velocity
    static const int idY = 3; // Y velocity

    using Vector = Eigen::Matrix<double, NUM_VARS, 1>;
    using Matrix = Eigen::Matrix<double, NUM_VARS, NUM_VARS>;

    Kalman(double iniX, double inidX, double iniY, double inidY){
        Xn(iX)  = iniX;
        Xn(idX) = inidX;
        Xn(iY)  = iniY;
        Xn(idY) = inidY;

        // Prediction matrices
        F .setIdentity();
        G.setZero();
        Q.setIdentity();
        P.setIdentity();

        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        R <<    5, 0,
                0, 5;
    }

    void predict(double dt)
    {
        F(iX, idX) = dt;
        F(iY, idY) = dt;

        // Predict:   Xn+1 = F * Xn + G * Un
        Xn = F * Xn;

        G(iX, iX) = 0.5 * dt * dt;
        G(iY, iY) = 0.5 * dt * dt;
        G(idX, iX) = dt;
        G(idY, iY) = dt;

        P  = F * P * F.transpose() + G * G.transpose() * 0.2; // 0.2 as acceleration covariance
    }

    void update(double x, double y, double measVariance)
    {
        const double y = measValue - H * Xp;
        const double S = H * Pp * H.transpose() + measVariance;

        const Vector K = Pp * H.transpose() * 1.0 / S;

        

        Vector newX = Xp + K * y;
        Matrix newP = (Matrix::Identity() - K * H) * Pp;

        Pp = newP;
        Xp = newX;
    }

private:
    Vector Xp; // X vector previous
    Matrix Pp; // P matrix previous

    const double m_accelVariance;
};