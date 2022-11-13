// Copyright (c) 2022 OUXT Polaris
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

#ifndef OUAGV_EKF__VISIBILITY_CONTROL_H_
#define OUAGV_EKF__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OUAGV_EKF_EXPORT __attribute__((dllexport))
#define OUAGV_EKF_IMPORT __attribute__((dllimport))
#else
#define OUAGV_EKF_EXPORT __declspec(dllexport)
#define OUAGV_EKF_IMPORT __declspec(dllimport)
#endif
#ifdef OUAGV_EKF_BUILDING_LIBRARY
#define OUAGV_EKF_PUBLIC OUAGV_EKF_EXPORT
#else
#define OUAGV_EKF_PUBLIC OUAGV_EKF_IMPORT
#endif
#define OUAGV_EKF_PUBLIC_TYPE OUAGV_EKF_PUBLIC
#define OUAGV_EKF_LOCAL
#else
#define OUAGV_EKF_EXPORT __attribute__((visibility("default")))
#define OUAGV_EKF_IMPORT
#if __GNUC__ >= 4
#define OUAGV_EKF_PUBLIC __attribute__((visibility("default")))
#define OUAGV_EKF_LOCAL __attribute__((visibility("hidden")))
#else
#define OUAGV_EKF_PUBLIC
#define OUAGV_EKF_LOCAL
#endif
#define OUAGV_EKF_PUBLIC_TYPE
#endif

#endif  // OUAGV_EKF__VISIBILITY_CONTROL_H_
