// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <vector>

#include "mynteye/api.h"
#include "mynteye/glog_init.h"

namespace {

template <typename T>
inline void std_vector_assign(
    std::vector<T> &l, const boost::python::object &o) {  // NOLINT
  l.assign(
      boost::python::stl_input_iterator<T>(o),
      boost::python::stl_input_iterator<T>());
}

template <typename T>
inline std::vector<T> py_list_to_std_vector(const boost::python::object &o) {
  return std::vector<T>(
      boost::python::stl_input_iterator<T>(o),
      boost::python::stl_input_iterator<T>());
}

template <typename T>
inline boost::python::list std_vector_to_py_list(const std::vector<T> &v) {
  boost::python::list l;
  for (auto &&val : v) {
    l.append(val);
  }
  return l;
}

template <typename Container>
char **new_cstrings(const Container &strings, std::size_t n) {
  char **cstrings = new char *[n];
  for (std::size_t i = 0; i < n; i++) {
    cstrings[i] = new char[strings[i].size() + 1];
    std::strcpy(cstrings[i], strings[i].c_str());  // NOLINT
  }
  return cstrings;
}

void del_cstrings(char **cstrings, std::size_t n) {
  for (std::size_t i = 0; i < n; i++) {
    delete[] cstrings[i];
  }
  delete[] cstrings;
}

}  // namespace

using namespace boost::python;  // NOLINT

MYNTEYE_USE_NAMESPACE

// api create static methods

std::shared_ptr<API> (*api_create_1)() = &API::Create;

std::shared_ptr<API> api_create_2(list argv) {
  auto &&args = py_list_to_std_vector<std::string>(argv);
  auto &&n = args.size();
  if (n == 0) {
    return API::Create();
  }
  char **cstrings = new_cstrings(args, n);
  auto &&api = API::Create(args.size(), cstrings);
  del_cstrings(cstrings, n);
  return api;
}

// glog_init create static methods

std::shared_ptr<glog_init> glog_init_create(list argv) {
  auto &&args = py_list_to_std_vector<std::string>(argv);
  auto &&n = args.size();
  assert(n > 0);
  char **cstrings = new_cstrings(args, n);
  auto &&ret = std::make_shared<glog_init>(args.size(), cstrings);
  del_cstrings(cstrings, n);
  return ret;
}

// BOOST_PYTHON_MODULE

BOOST_PYTHON_MODULE(mynteye_py) {
  class_<API, boost::noncopyable>("api", no_init)
      .def("create", api_create_1)
      .def("create", &api_create_2)
      .staticmethod("create");

  register_ptr_to_python<std::shared_ptr<API>>();

  class_<glog_init, boost::noncopyable>("glog_init", no_init)
      .def("create", &glog_init_create)
      .staticmethod("create");

  register_ptr_to_python<std::shared_ptr<glog_init>>();
}
