// Copyright (c) 2020 OUXT Polaris
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

#ifndef NAVI_SIM__CAMERA_SIM_COMPONENT_HPP_
#define NAVI_SIM__CAMERA_SIM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT __attribute__((dllexport))
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT __declspec(dllexport)
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NAVI_SIM_CAMEAR_SIM_COMPONENT_BUILDING_DLL
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT
#endif
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC_TYPE NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_LOCAL
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_LOCAL
#endif
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#endif  // NAVI_SIM__CAMERA_SIM_COMPONENT_HPP_
