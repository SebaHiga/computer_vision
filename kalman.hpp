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
    Eigen::Matrix<double, nx, nx> Q;
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

    // Time
    double dt;

    Kalman(){
        // Prediction matrices
        F.setIdentity();
        G.setZero();
        Q.setIdentity();
        P.setIdentity();

        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        R <<    1, 0,
                0, 1;
    }

    Kalman(double iniX, double inidX, double iniY, double inidY, double dt) : dt (dt){
        Xn(iX)  = iniX;
        Xn(idX) = inidX;
        Xn(iY)  = iniY;
        Xn(idY) = inidY;

        // Prediction matrices
        F.setIdentity();
        G.setZero();
        Q.setIdentity();

        R.setIdentity();
        R *= 5;

        P = P.setIdentity();

        H.setZero();
        H(iX, iX) = 1;
        H(iY, iY) = 1;
    }

    void init(double iniX, double iniY, double dt){
        Xn(iX)  = iniX;
        Xn(idX) = 0;
        Xn(iY)  = iniY;
        Xn(idY) = 0;
        this->dt = dt;
    }

    // Prediction method for object tracking
    void predict(){
        F(iX, idX) = dt;
        F(iY, idY) = dt;

        // Predict:   Xn+1 = F * Xn + G * Un
        Xn = F * Xn;

        G(iX, iX) = 0.5 * dt * dt;
        G(iY, iY) = 0.5 * dt * dt;
        G(idX, iX) = dt;
        G(idY, iY) = dt;

        Q = G * 0.5 * G.transpose();

        P  = F * P * F.transpose() + Q; 
    }

    void update(int x, int y){
        // K
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // Xn
        Zn <<   x,
                y;
        Xn = Xn + K * (Zn - H * Xn);

        // P
        Eigen::Matrix<double, nx, nx> I;
        I.Identity();
        P = P * (I.Identity() - K * H);
        // P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
        // P = (I - K * H) * P;
    }

    void getPosition(double *x, double *y){
        *x = Xn(iX);
        *y = Xn(iY);
    }
};