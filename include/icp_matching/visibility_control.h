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

#ifndef ICP_MATCHING_ICP_MATCHING_COMPONENT__VISIBILITY_CONTROL_H_
#define ICP_MATCHING_ICP_MATCHING_COMPONENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_EXPORT __attribute__((dllexport))
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_EXPORT __declspec(dllexport)
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef ICP_MATCHING_ICP_MATCHING_COMPONENT_BUILDING_LIBRARY
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC ICP_MATCHING_ICP_MATCHING_COMPONENT_EXPORT
#else
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC ICP_MATCHING_ICP_MATCHING_COMPONENT_IMPORT
#endif
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC_TYPE ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_LOCAL
#else
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_EXPORT __attribute__((visibility("default")))
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_LOCAL
#endif
#define ICP_MATCHING_ICP_MATCHING_COMPONENT_PUBLIC_TYPE
#endif

#endif // ICP_MATCHING_ICP_MATCHING_COMPONENT__VISIBILITY_CONTROL_H_