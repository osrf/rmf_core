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

#include <rmf_utils/impl_ptr.hpp>
#include "TestClass.hpp"

#include <iostream>

int main()
{
  // TODO(MXG): Make this a proper test
  TestClass test("here is my test string");
  std::cout << test.get_test_text() << std::endl;

  TestClass copy{test};
  std::cout << copy.get_test_text() << std::endl;

  TestClass moved = std::move(test);
  std::cout << moved.get_test_text() << std::endl;

  if(test._pimpl)
    std::cout << "impl_ptr returning true" << std::endl;
  else
    std::cout << "impl_ptr returning false" << std::endl;

  test = TestClass("replacing the text");
  std::cout << test.get_test_text() << std::endl;
}
