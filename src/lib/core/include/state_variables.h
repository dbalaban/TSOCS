#ifndef CORE_STATE_VARIABLES_H
#define CORE_STATE_VARIABLES_H

#include <Eigen/Core>

template <typename T>
using StateVector = Eigen::Matrix<T, 2, 1>;

template <typename T>
StateVector<T> Acceleration(const T* params) {
  return StateVector<T>(params[0]*params[1]+params[3],
                    params[0]*params[2]+params[4]).normalized();
}

template <typename T>
StateVector<T> Velocity(const T* time, const T* params, const StateVector<T>& v0) {
  T psix = time[0]*params[0]+params[2];
  T psiy = time[0]*params[1]+params[3];

  StateVector<T> p(params[2], params[3]);
  StateVector<T> q(params[0], params[1]);

  T q1 = q.norm();
  T q2 = q.squaredNorm();
  T q3 = q2*q1;

  T h1 = sqrt(psix*psix + psiy*psiy);
  T h2 = h1*q1+time[0]*q2 + p.dot(q);
  T h3 = p.norm()*q1 + p.dot(q);

  T gamma = abs(h2/h3);
  if (ceres::isnan(gamma) || ceres::isinf(gamma) || gamma == T(0)) {
    gamma = T(1.0);
  }
  T detpq = p.x()*q.y()-p.y()*q.x();

  T f1 = (h1-p.norm())/q2;
  T f2 = detpq*log(gamma)/q3;

  T vx = v0.x() + params[0]*f1 + params[1]*f2;
  T vy = v0.y() + params[1]*f1 - params[0]*f2;

  if (ceres::isnan(vx) || ceres::isnan(vy)) {
    std::cout << "NaN in velocity" << std::endl;
    std::cout << "time: " << time[0] << std::endl;
    std::cout << "params: " << params[0] << " " << params[1] << " " << params[2] << " " << params[3] << " " << params[4] << std::endl;
    std::cout << "v0: " << v0.x() << " " << v0.y() << std::endl;
    std::cout << "psix: " << psix << std::endl;
    std::cout << "psiy: " << psiy << std::endl;
    std::cout << "p: " << p.x() << " " << p.y() << std::endl;
    std::cout << "q: " << q.x() << " " << q.y() << std::endl;
    std::cout << "h1: " << h1 << std::endl;
    std::cout << "h2: " << h2 << std::endl;
    std::cout << "h3: " << h3 << std::endl;
    std::cout << "gamma: " << gamma << std::endl;
    std::cout << "detpq: " << detpq << std::endl;
    exit(1);
  }

  return StateVector<T>(vx, vy);
}

template <typename T>
StateVector<T> Position(const T* time, const T* params, const StateVector<T>& v0, const StateVector<T>& x0) {
  T psix = time[0]*params[0]+params[2];
  T psiy = time[0]*params[1]+params[3];

  StateVector<T> p(params[2], params[3]);
  StateVector<T> q(params[0], params[1]);

  T q1 = q.norm();
  T q2 = q.squaredNorm();
  T q3 = q2*q1;
  T q5 = q3*q2;
  T pdotq = p.dot(q);

  T h1 = sqrt(psix*psix + psiy*psiy);
  T h2 = h1*q1+time[0]*q2 + pdotq;
  T h3 = p.norm()*q1 + pdotq;

  T gamma = abs(h2/h3);
  if (ceres::isnan(gamma) || ceres::isinf(gamma) || gamma == T(0)) {
    gamma = T(1.0);
  }

  T detpq = p.x()*q.y()-p.y()*q.x();
  T detpq2 = detpq*detpq;

  T f1 = (h1*(q1*pdotq + time[0]*q3) + detpq2*log(gamma) 
              - p.norm()*(q1*pdotq + T(2)*time[0]*q3))/(T(2)*q5);
  T f2 = detpq*(log(gamma)*(time[0] + pdotq/q2)
                  -(h1-p.norm())/q1)/q3;

  T x = x0.x() + time[0]*v0.x() + params[0]*f1 + params[1]*f2;
  T y = x0.y() + time[0]*v0.y() + params[1]*f1 - params[0]*f2;

  if (ceres::isnan(x) || ceres::isnan(y)) {
    std::cout << "NaN in position" << std::endl;
    std::cout << "time: " << time[0] << std::endl;
    std::cout << "params: " << params[0] << " " << params[1] << " " << params[2] << " " << params[3] << " " << params[4] << std::endl;
    std::cout << "v0: " << v0.x() << " " << v0.y() << std::endl;
    std::cout << "x0: " << x0.x() << " " << x0.y() << std::endl;
    std::cout << "psix: " << psix << std::endl;
    std::cout << "psiy: " << psiy << std::endl;
    std::cout << "p: " << p.x() << " " << p.y() << std::endl;
    std::cout << "q: " << q.x() << " " << q.y() << std::endl;
    std::cout << "h1: " << h1 << std::endl;
    std::cout << "h2: " << h2 << std::endl;
    std::cout << "h3: " << h3 << std::endl;
    std::cout << "gamma: " << gamma << std::endl;
    std::cout << "pdotq: " << pdotq << std::endl;
    std::cout << "detpq: " << detpq << std::endl;
    std::cout << "f1: " << f1 << std::endl;
    std::cout << "f2: " << f2 << std::endl;

    std::cout << "pnorm*qnorm: " << p.norm()*q1 << std::endl;

    exit(1);
  }

  return StateVector<T>(x, y);
}

template <typename T>
T Time(const T* time, const T* params) {
  return time[0];
}

#endif // CORE_STATE_VARIABLES_H