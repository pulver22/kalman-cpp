/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <Eigen/Dense>

#include "../kalman-cpp/kalman.hpp"

int main(int argc, char* argv[]) {

  int n = 4; // Number of states
  int m = 2; // Number of measurements

  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance (external uncertainty)
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  // A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  // C << 1, 0, 0;
  A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
  C << 1, 0, 0, 0, 0, 1, 0, 0;


  // Reasonable covariance matrices
  // Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  // R << 5;
  // P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
  // Q.fill(0.05);
  Q << .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, 0.5;
  R <<  0.1, 0.1, 0.1, 0.1;
  P << .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 10000, 100, .1, .1, 100, 10000;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // List of noisy position measurements (y)
  // std::vector<double> measurements = {
  //     1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
  //     1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
  //     2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
  //     2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
  //     2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
  //     2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
  //     2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
  //     1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
  //     0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  // };
  std::vector<double> measurements = {
        1.0420271 ,  1.97125435,
        1.1072679 ,  1.90713242,
        1.29135111,  1.77047746,
        1.48485251,  1.59290453,
        1.72825901,  2.03651545,
        1.7421649 ,  1.81403957,
        2.1167204 ,  2.95436506,
        2.14529225,  2.83082132,
        2.16029641,  2.86511146,
        2.21269371,  2.80456971,
        2.5770935 ,  2.76326424,
        2.66822157,  2.78468231,
        2.51641839,  3.23616546,
        2.76034057,  3.49974578,
        2.88131781,  2.93471433,
        2.88373787,  2.99267245,
        2.94484687,  3.06242828,
        2.828666  ,  2.96649138,
        3.00066019,  3.39960905,
        3.12920592,  3.73870541,
      //   2.85836178,  3.54768509,
      //   2.8380817 ,  3.62424607,
      //   2.68975331,  3.41436343,
      //   2.66533186,  3.07929824,
      //   2.816135  ,  3.59687918,
      //   2.81003612,  3.22918552,
      //   2.88321849,  3.32604252,
      //   2.69789265,  3.15407535,
      //   2.43422292,  3.01288319,
      //   2.23464792,  2.72786575,
      //   2.30278776,  3.21567218,
      //   2.0206977 ,  2.6355265 ,
      //   1.94393986,  2.60163411,
      //   1.82498399,  1.92347767,
      //   1.5252623 ,  1.96307009,
      //   1.86967808,  2.08297633,
      //   1.18073208,  1.25699047,
      //   1.10729605,  2.06696502,
      //   0.91616835,  1.14259021,
      //   0.67854766,  1.22833434,
      //   0.56238175,  1.15017452,
      //   0.35546847,  0.7309502 ,
      //  -0.15560749,  0.05948539,
      //  -0.28719866,  0.21761518,
      //  -0.60297317, -0.41740015
      };
      
  std::ofstream f_measurements;
  f_measurements.open("/home/pulver/Desktop/measurements.txt");
  std::ostream_iterator<double> output_iterator(f_measurements, "\n");
  std::copy(measurements.begin(), measurements.end(), output_iterator);
  f_measurements.close();
  

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  // x0 << measurements[0], 0, -9.81;
  x0[0] = measurements[0];
  x0[1] = measurements[1];
  x0[2] = 0;
  x0[3] = 0;
  

  // x0 << 1, 1, 1, 1, 1; //measurements[0], measurements[1], 0, 0;
  double t = 0;
  kf.init(t, x0);

  std::ofstream f_filtered;
  f_filtered.open("/home/pulver/Desktop/filtered.txt");
  
  // Feed measurements into filter, output estimated states
  Eigen::VectorXd y(m);
  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  for(int i = 0; i < measurements.size(); i++) {
    t += dt;
    // y << measurements[i];
    y[0] = measurements[i];
    y[1] = measurements[i+1];
    kf.update(y);
    std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
        << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
    i++;
    f_filtered << kf.state().transpose()[0] << "\n" << kf.state().transpose()[1] << std::endl;
  }
  f_filtered.close();

  return 0;
}
