/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_UTILS__MACROS_HPP
#define RMF_UTILS__MACROS_HPP

#define RMF_UTILS__DEFAULT_COPY_MOVE(Class) \
  Class(const Class&) = default; \
  Class(Class&&) = default; \
  Class& operator=(const Class&) = default; \
  Class& operator=(Class&&) = default

#endif // RMF_UTILS__MACROS_HPP
