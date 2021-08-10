#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>

using namespace Eigen;
class KalmanFilter {

public:

    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,  //dynamics matrix
        const Eigen::MatrixXd& C,  //out matrix
        const Eigen::MatrixXd& Q,  //process noise covariance 
        const Eigen::MatrixXd& B,  
        const Eigen::MatrixXd& R,   //measurment noise covariance 
        const Eigen::MatrixXd& P)   //estimate error covariance
        : A(A), C(C), Q(Q), B(B), R(R), P0(P),
        m(C.rows()), n(A.rows()), dt(dt), in(false),
        I(n, n), xh(n), xh_new(n)
    {
        I.setIdentity();
    }



  
    Eigen::VectorXd state() { return xh; };  //return state
    double time() { return t; };  //return time


   
    void init(double t0, const Eigen::VectorXd& x0) {  //initialization 
        xh = x0;
        P = P0;
        this->t0 = t0;
        t = t0;
        in = true;
    }

    void init() {
        xh.setZero();
        P = P0;
        t0 = 0;
        t = t0;
        in = true;
    }

   
    void update(const Eigen::VectorXd& u, const Eigen::VectorXd& y) {  //update the state 

    

        //prediction
        xh_new = (A * xh) + (B * u);
        P = A * P * A.transpose() + Q;

        //correction and update
        K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
        xh_new += K * (y - C * xh_new);
        P = (I - K * C) * P;
        xh = xh_new;

        t += dt;
    }

    void update(const Eigen::VectorXd& u, const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

        this->A = A;
        this->dt = dt;
        update(u, y);
    }



private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, B, P, K, P0;

    int m, n; // System dimensions

    double t0, t; // Initial and current time

    double dt; // Discrete time step

    bool in; // true if intialized 

    Eigen::MatrixXd I; // n-size identity

    Eigen::VectorXd xh, xh_new; // Estimated states


};


int main() {

    int n = 2; // Number of states
    int m = 1; // Number of measurements
    double dt = 0.8; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix - prediction matrix
    Eigen::MatrixXd C(m, n); // Output matrix - sensor matrix
    Eigen::MatrixXd B(n, m);
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance



    C << 1, 1;
    A << 1, dt, 1, 1;
    Q << .1, 1, 0, .1;
    R << 0.05;
    P << 0.01, 0, 0, 1;
    B << 0, dt;

    std::cout << "A: \n" <<"["<< A <<"]"<< std::endl;
    std::cout << "\nC: \n" << "[" << C << "]" << std::endl;
    std::cout << "\nB: \n" << "[" << B << "]" << std::endl;
    std::cout << "\nQ: \n" << "[" << Q << "]" << std::endl;
    std::cout << "\nR: \n" << "[" << R << "]" << std::endl;
    std::cout << "\nP: \n" << "[" << P << "]" << std::endl;

    KalmanFilter kf(dt, A, C, Q, B, R, P);

    Eigen::MatrixXd x0(n, m);
    x0 << 0, 5;

    double t = 0;
    kf.init(t, x0);


    Eigen::VectorXd y(m);
    Eigen::VectorXd u(m);

    std::cout << "\nt = " << t << ", " << "xh[0]: " << kf.state().transpose() << std::endl;

    y << 3.2;
    u << -4;
    kf.update(u, y);

    std::cout << "\nt = " << t << ", " << ", xh[" << 1 << "] = " << kf.state().transpose() << std::endl;

    return 0;
}