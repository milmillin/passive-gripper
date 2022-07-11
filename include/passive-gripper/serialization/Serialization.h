// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

namespace psg {
namespace serialization {

struct Serializable {
  virtual void Serialize(std::ofstream& f) const = 0;
  virtual void Deserialize(std::ifstream& f) = 0;
  inline void SerializeFn(const std::string& fn) const {
    std::ofstream f(fn, std::ios::out | std::ios::binary);
    if (!f.is_open()) {
      throw std::invalid_argument("cannot open " + fn);
    }
    this->Serialize(f);
  }
  inline void DeserializeFn(const std::string& fn) {
    std::ifstream f(fn, std::ios::in | std::ios::binary);
    if (!f.is_open()) {
      throw std::invalid_argument("cannot open " + fn);
    }
    this->Deserialize(f);
  }
};

// Serializable
template <typename T,
          std::enable_if_t<std::is_base_of_v<Serializable, T>, bool> = true>
inline void Serialize(const T& obj, std::ofstream& f);
template <typename T,
          std::enable_if_t<std::is_base_of_v<Serializable, T>, bool> = true>
inline void Deserialize(T& obj, std::ifstream& f);

// Arithmetic types
template <
    typename T,
    std::enable_if_t<std::is_arithmetic<T>::value || std::is_enum<T>::value,
                     bool> = true>
inline void Serialize(const T& obj, std::ofstream& f);
template <
    typename T,
    std::enable_if_t<std::is_arithmetic<T>::value || std::is_enum<T>::value,
                     bool> = true>
inline void Deserialize(T& obj, std::ifstream& f);

// std::string
inline void Serialize(const std::string& obj, std::ofstream& f);
inline void Deserialize(std::string& obj, std::ifstream& f);

// std::vector
template <typename T1, typename T2>
inline void Serialize(const std::vector<T1, T2>& obj, std::ofstream& f);
template <typename T1, typename T2>
inline void Deserialize(std::vector<T1, T2>& obj, std::ifstream& f);

// std::map
template <typename T1, typename T2>
inline void Serialize(const std::map<T1, T2>& obj, std::ofstream& f);
template <typename T1, typename T2>
inline void Deserialize(std::map<T1, T2>& obj, std::ifstream& f);

// Eigen::Array
template <typename T, int R, int C, int P, int MR, int MC>
inline void Serialize(const Eigen::Array<T, R, C, P, MR, MC>& obj,
                      std::ofstream& f);
template <typename T, int R, int C, int P, int MR, int MC>
inline void Deserialize(Eigen::Array<T, R, C, P, MR, MC>& obj,
                        std::ifstream& f);

// Eigen::Matrix
template <typename T, int R, int C, int P, int MR, int MC>
inline void Serialize(const Eigen::Matrix<T, R, C, P, MR, MC>& obj,
                      std::ofstream& f);
template <typename T, int R, int C, int P, int MR, int MC>
inline void Deserialize(Eigen::Matrix<T, R, C, P, MR, MC>& obj,
                        std::ifstream& f);

// Wrapper
template <typename T>
inline void Serialize(const T& obj, const std::string& fn);
template <typename T>
inline void Deserialize(T& obj, const std::string& fn);

//============== IMPLEMENTATION =================

// Serializable
template <typename T,
          std::enable_if_t<std::is_base_of_v<Serializable, T>, bool> = true>
void Serialize(const T& obj, std::ofstream& f) {
  static_cast<const Serializable*>(&obj)->Serialize(f);
}
template <typename T,
          std::enable_if_t<std::is_base_of_v<Serializable, T>, bool> = true>
void Deserialize(T& obj, std::ifstream& f) {
  static_cast<Serializable*>(&obj)->Deserialize(f);
}

// Arithmetic types
template <
    typename T,
    std::enable_if_t<std::is_arithmetic<T>::value || std::is_enum<T>::value,
                     bool> = true>
void Serialize(const T& obj, std::ofstream& f) {
  f.write(reinterpret_cast<const char*>(&obj), sizeof(obj));
}

template <
    typename T,
    std::enable_if_t<std::is_arithmetic<T>::value || std::is_enum<T>::value,
                     bool> = true>
void Deserialize(T& obj, std::ifstream& f) {
  f.read((char*)&obj, sizeof(obj));
}

// std::string
void Serialize(const std::string& obj, std::ofstream& f) {
  Serialize(obj.size(), f);
  f.write(obj.c_str(), obj.size());
}

void Deserialize(std::string& obj, std::ifstream& f) {
  size_t size = 0;
  Deserialize(size, f);
  char* buf = new char[size + 1];
  f.read(buf, size);
  buf[size] = 0;
  obj = buf;
  delete buf;
}

// std::vector
template <typename T1, typename T2>
void Serialize(const std::vector<T1, T2>& obj, std::ofstream& f) {
  Serialize(obj.size(), f);
  for (size_t i = 0; i < obj.size(); i++) {
    Serialize(obj[i], f);
  }
}

template <typename T1, typename T2>
void Deserialize(std::vector<T1, T2>& obj, std::ifstream& f) {
  size_t size;
  Deserialize(size, f);
  obj.resize(size);
  for (size_t i = 0; i < size; i++) {
    Deserialize(obj[i], f);
  }
}

// std::map
template <typename T1, typename T2>
void Serialize(const std::map<T1, T2>& obj, std::ofstream& f) {
  Serialize(obj.size());
  for (const auto& kv : obj) {
    Serialize(kv.first, f);
    Serialize(kv.second, f);
  }
}

template <typename T1, typename T2>
void Deserialize(std::map<T1, T2>& obj, std::ifstream& f) {
  size_t size;
  Deserialize(size, f);
  obj.clear();
  for (size_t i = 0; i < size; i++) {
    T1 key;
    T2 value;
    Deserialize(key, f);
    Deserialize(value, f);
    obj.insert(std::make_pair(key, value));
  }
}

// Eigen::Array

template <typename T, int R, int C, int P, int MR, int MC>
void Serialize(const Eigen::Array<T, R, C, P, MR, MC>& obj, std::ofstream& f) {
  Serialize(obj.rows(), f);
  Serialize(obj.cols(), f);
  for (long long i = 0; i < obj.size(); i++) {
    Serialize(obj(i), f);
  }
}

template <typename T, int R, int C, int P, int MR, int MC>
void Deserialize(Eigen::Array<T, R, C, P, MR, MC>& obj, std::ifstream& f) {
  long long rows;
  long long cols;
  Deserialize(rows, f);
  Deserialize(cols, f);
  obj.resize(rows, cols);
  for (long long i = 0; i < obj.size(); i++) {
    Deserialize(obj(i), f);
  }
}

// Eigen::Matrix

template <typename T, int R, int C, int P, int MR, int MC>
void Serialize(const Eigen::Matrix<T, R, C, P, MR, MC>& obj, std::ofstream& f) {
  Serialize(obj.rows(), f);
  Serialize(obj.cols(), f);
  for (long long i = 0; i < obj.size(); i++) {
    Serialize(obj(i), f);
  }
}

template <typename T, int R, int C, int P, int MR, int MC>
void Deserialize(Eigen::Matrix<T, R, C, P, MR, MC>& obj, std::ifstream& f) {
  long long rows;
  long long cols;
  Deserialize(rows, f);
  Deserialize(cols, f);
  obj.resize(rows, cols);
  for (long long i = 0; i < obj.size(); i++) {
    Deserialize(obj(i), f);
  }
}

// Wrapper
template <typename T>
inline void Serialize(const T& obj, const std::string& fn) {
  std::ofstream f(fn, std::ios::out | std::ios::binary);
  if (!f.is_open()) {
    throw std::invalid_argument("cannot open " + fn);
  }
  Serialize(obj, f);
}
template <typename T>
inline void Deserialize(T& obj, const std::string& fn) {
  std::ifstream f(fn, std::ios::in | std::ios::binary);
  if (!f.is_open()) {
    throw std::invalid_argument("cannot open " + fn);
  }
  Deserialize(obj, f);
}

}  // namespace serialization
}  // namespace psg

#define SERIALIZE(obj) psg::serialization::Serialize(obj, f)
#define DESERIALIZE(obj) psg::serialization::Deserialize(obj, f)
#define DECL_SERIALIZE() inline void Serialize(std::ofstream& f) const override
#define DECL_DESERIALIZE() inline void Deserialize(std::ifstream& f) override
