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

    Kalman(){
        // Prediction matrices
        F.setIdentity();
        G.setZero();
        Q.setIdentity();
        P.setIdentity();

        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        R <<    5, 0,
                0, 5;
    }

    Kalman(double iniX, double inidX, double iniY, double inidY){
        Xn(iX)  = iniX;
        Xn(idX) = inidX;
        Xn(iY)  = iniY;
        Xn(idY) = inidY;

        // Prediction matrices
        F.setIdentity();
        G.setZero();
        Q.setIdentity();
        P.setIdentity();

        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        R <<    5, 0,
                0, 5;
    }

    void init(double iniX, double iniY){
        Xn(iX)  = iniX;
        Xn(idX) = 0;
        Xn(iY)  = iniY;
        Xn(idY) = 0;
    }

    // Prediction method for object tracking
    void predict(double dt){
        F(iX, idX) = dt;
        F(iY, idY) = dt;

        // Predict:   Xn+1 = F * Xn + G * Un
        Xn = F * Xn;

        G(iX, iX) = 0.5 * dt * dt;
        G(iY, iY) = 0.5 * dt * dt;
        G(idX, iX) = dt;
        G(idY, iY) = dt;

        // Q = G * 0.2 *G.transpose();

        P  = F * P * F.transpose() + Q; // 0.2 as acceleration covariance
    }

    void update(int x, int y){
        // K
        K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();

        // Xn
        Zn <<   x,
                y;
        Xn = Xn + K * (Zn - H * Xn);
        std::cout << Zn << std::endl;

        // P
        Eigen::Matrix<double, nx, nx> I;
        P = P * (I.Identity() - K * H);
    }

    void getPosition(double *x, double *y){
        *x = Xn(iX);
        *y = Xn(iY);
    }
};