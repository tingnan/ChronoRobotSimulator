#pragma once

#include "core/ChMath.h"
#include <iostream>

template <class T>
std::ostream &operator<<(std::ostream &stream,
                         const chrono::ChVector<T> &tmpvec) {
  stream << tmpvec.x << " ";
  stream << tmpvec.y << " ";
  stream << tmpvec.z;
  return stream;
}

template <class T>
std::ostream &operator<<(std::ostream &stream,
                         const chrono::ChQuaternion<T> &tmpq) {
  stream << tmpq.e0 << " ";
  stream << tmpq.e1 << " ";
  stream << tmpq.e2 << " ";
  stream << tmpq.e3;
  return stream;
}

template <class T>
std::ostream &operator<<(std::ostream &stream, const std::vector<T> &myvec) {
  typename std::vector<T>::const_iterator iter1 = myvec.begin();
  for (; iter1 != myvec.end(); ++iter1) {
    stream << *iter1 << " ";
  }
  return stream;
}

inline double dot(const chrono::ChVector<> &v1, const chrono::ChVector<> &v2) {
  return v1.Dot(v2);
}

inline double length(const chrono::ChVector<> &v1) { return sqrt(v1.Dot(v1)); }
