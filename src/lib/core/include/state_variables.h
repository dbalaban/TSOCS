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
StateVector<T> Velocity(const T* params, const StateVector<T>& v0) {
  T psix = params[0]*params[1]+params[3];
  T psiy = params[0]*params[2]+params[4];

  StateVector<T> p(params[3], params[4]);
  StateVector<T> q(params[1], params[2]);

  T h1 = sqrt(psix*psix + psiy*psiy);
  T h2 = q.norm()*(h1+params[0]*params[0]) + p.dot(q);
  T h3 = p.norm()*q.norm() + p.dot(q);

  T gamma = h2/h3;
  T q3 = q.norm()*q.norm()*q.norm();
  T detpq = p.x()*q.y()-p.y()*q.x();

  T f1 = (h1-p.norm())/q.squaredNorm();
  T f2 = detpq*log(gamma)/q3;

  T vx = vo.x() + params[1]*f1 + params[2]*f2;
  T vy = vo.y() + params[2]*f1 - params[1]*f2;

  return StateVector<T>(vx, vy);
}

template <typename T>
StateVector<T> Position(const T* params, const StateVector<T>& v0, const StateVector<T>& x0) {
  T psix = params[0]*params[1]+params[3];
  T psiy = params[0]*params[2]+params[4];

  StateVector<T> p(params[3], params[4]);
  StateVector<T> q(params[1], params[2]);

  T h1 = sqrt(psix*psix + psiy*psiy);
  T h2 = q.norm()*(h1+params[0]*params[0]) + p.dot(q);
  T h3 = p.norm()*q.norm() + p.dot(q);

  T gamma = h2/h3;
  T q1 = q.norm();
  T q2 = q.squaredNorm();
  T q3 = q2*q1;
  T q5 = q3*q2;

  T pdotq = p.dot(q);
  T pcrossq = p.cross(q).squaredNorm();
  T detpq = p.x()*q.y()-p.y()*q.x();

  T f1 = (h1*(q1*pdotq + 2*params[0]*q3) + pcrossq*log(gamma) 
              - p.norm()*(p1*pdotq + 2*params[0]*q3))/(2*q5);
  T f2 = detpq*(log(gamma)*(params[0] + pdotq/q2)
                  -(h1-p.norm())/q1)/q3;

  T x = x0.x() + params[0]*vo.x() + params[1]*f1 + params[2]*f2;
  T y = x0.y() + params[0]*vo.y() + params[2]*f1 - params[1]*f2;

  return StateVector<T>(vx, vy);
}

template <typename T>
T Time(const T* params) {
  return params[0];
}

#endif // CORE_STATE_VARIABLES_H