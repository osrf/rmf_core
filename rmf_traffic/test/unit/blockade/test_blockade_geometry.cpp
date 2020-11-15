/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <rmf_utils/catch.hpp>

#include <src/rmf_traffic/blockade/geometry.hpp>

SCENARIO("Test blockade geometry calculations for direct conflicts")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;
  const double max_angle = 1.0*M_PI/180.0;

  std::array<Eigen::Vector2d, 9> A;
  A[0] = { 0, 10};
  A[1] = { 0,  7};
  A[2] = { 0,  5};
  A[3] = { 5,  5};
  A[4] = { 5,  0};
  A[5] = {10,  0};
  A[6] = {10,  5};
  A[7] = {15,  5};
  A[8] = {15, 10};

  std::array<Segment, 8> s_A;
  for (std::size_t i=0; i < A.size()-1; ++i)
    s_A[i] = Segment{A[i], A[i+1], radius};

  std::array<Eigen::Vector2d, 5> B;
  B[0] = { 15, 10};
  B[1] = { 15,  5};
  B[2] = {7.5,  5};
  B[3] = {  0,  5};
  B[4] = {  0,  0};

  std::array<Segment, 4> s_B;
  for (std::size_t i=0; i < B.size()-1; ++i)
    s_B[i] = Segment{B[i], B[i+1], radius};

  for (std::size_t i=0; i < s_B.size(); ++i)
    CHECK_FALSE(detect_conflict(s_A[0], s_B[i], max_angle).is_conflict());

  {
    const auto info = detect_conflict(s_A[1], s_B[2], max_angle);
    CHECK(info.is_conflict());
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  CHECK_FALSE(detect_conflict(s_A[1], s_B[3], max_angle).is_conflict());

  for (std::size_t i=0; i < 2; ++i)
    CHECK_FALSE(detect_conflict(s_A[1], s_B[i], max_angle).is_conflict());

  {
    const auto info = detect_conflict(s_A[2], s_B[2], max_angle);
    CHECK(info.is_conflict());
    CHECK(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  CHECK_FALSE(detect_conflict(s_A[2], s_B[3], max_angle).is_conflict());

  for (std::size_t i=0; i < 2; ++i)
    CHECK_FALSE(detect_conflict(s_A[2], s_B[i], max_angle).is_conflict());

  for (std::size_t i=0; i < s_B.size(); ++i)
  {
    CHECK_FALSE(detect_conflict(s_A[3], s_B[i], max_angle).is_conflict());
    CHECK_FALSE(detect_conflict(s_A[4], s_B[i], max_angle).is_conflict());
  }

  {
    const auto info = detect_conflict(s_A[5], s_B[1], max_angle);
    CHECK(info.is_conflict());
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
  }

  CHECK_FALSE(detect_conflict(s_A[5], s_B[0], max_angle).is_conflict());

  for (std::size_t i=2; i < s_B.size(); ++i)
    CHECK_FALSE(detect_conflict(s_A[5], s_B[i], max_angle).is_conflict());

  {
    const auto info = detect_conflict(s_A[6], s_B[1], max_angle);
    CHECK(info.is_conflict());
    CHECK(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK(info.include_cap_b[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
  }

  {
    const auto info = detect_conflict(s_A[6], s_B[0], max_angle);
    CHECK(info.is_conflict());
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  for (std::size_t i=2; i < s_B.size(); ++i)
    CHECK_FALSE(detect_conflict(s_A[6], s_B[i], max_angle).is_conflict());

  {
    const auto info = detect_conflict(s_A[7], s_B[0], max_angle);
    CHECK(info.is_conflict());
    CHECK(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  for (std::size_t i=1; i < s_B.size(); ++i)
    CHECK_FALSE(detect_conflict(s_A[7], s_B[i], max_angle).is_conflict());
}

SCENARIO("Test blockade geometry conflicts for skimming conflicts")
{
  using namespace rmf_traffic::blockade;

  const double radius = 0.1;
  const double half_r = radius/2.0;
  const double max_angle = 1.0*M_PI/180.0;

  std::array<Eigen::Vector2d, 6> A;
  A[0] = { 0,  0};
  A[1] = { 5,  0};
  A[2] = { 8,  5};
  A[3] = {16,  5};
  A[4] = {20,  0};
  A[5] = {25,  0};

  std::array<Segment, 5> s_A;
  for (std::size_t i=0; i < A.size()-1; ++i)
    s_A[i] = Segment{A[i], A[i+1], radius};

  const double b_height = 5 + half_r;
  std::array<Eigen::Vector2d, 6> B;
  for (std::size_t i=0; i < B.size(); ++i)
    B[i] = {5.0*i, b_height};

  WHEN("B goes forward")
  {
    std::array<Segment, 5> s_B;
    for (std::size_t i=0; i < B.size()-1; ++i)
      s_B[i] = Segment{B[i], B[i+1], radius};

    {
      const auto info = detect_conflict(s_A[1], s_B[1], max_angle);
      CHECK(info.is_conflict());
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
      CHECK(info.include_cap_a[ConflictInfo::Finish]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
    }

    for (std::size_t i=0; i < s_A.size(); ++i)
    {
      for (std::size_t j=0; j < s_B.size(); ++j)
      {
        if (i==1 && j==1)
          continue;

        CHECK_FALSE(detect_conflict(s_A[i], s_B[j], max_angle).is_conflict());
      }
    }
  }

  WHEN("B goes backward")
  {
    std::reverse(B.begin(), B.end());

    std::array<Segment, 5> s_B;
    for (std::size_t i=0; i < B.size()-1; ++i)
      s_B[i] = Segment{B[i], B[i+1], radius};

    for (std::size_t i=0; i < s_B.size(); ++i)
      CHECK_FALSE(detect_conflict(s_A[0], s_B[i], max_angle).is_conflict());

    {
      const auto info = detect_conflict(s_A[1], s_B[3], max_angle);
      CHECK(info.is_conflict());
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
      CHECK(info.include_cap_a[ConflictInfo::Finish]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
    }

    for (std::size_t i=0; i < s_B.size(); ++i)
    {
      if (i == 3)
        continue;

      CHECK_FALSE(detect_conflict(s_A[1], s_B[i], max_angle).is_conflict());
    }

    {
      const auto info = detect_conflict(s_A[2], s_B[3], max_angle);
      CHECK(info.is_conflict());
      CHECK(info.include_cap_a[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
      CHECK(info.include_cap_b[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
    }

    {
      const auto info = detect_conflict(s_A[2], s_B[2], max_angle);
      CHECK(info.is_conflict());
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
      CHECK(info.include_cap_b[ConflictInfo::Start]);
      CHECK(info.include_cap_b[ConflictInfo::Finish]);
    }

    {
      const auto info = detect_conflict(s_A[2], s_B[1], max_angle);
      CHECK(info.is_conflict());
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
      CHECK(info.include_cap_a[ConflictInfo::Finish]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
      CHECK(info.include_cap_b[ConflictInfo::Finish]);
    }

    for (std::size_t i : {0, 4})
      CHECK_FALSE(detect_conflict(s_A[2], s_B[i], max_angle).is_conflict());

    {
      const auto info = detect_conflict(s_A[3], s_B[1], max_angle);
      CHECK(info.is_conflict());
      CHECK(info.include_cap_a[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
    }

    for (std::size_t i=0; i < s_B.size(); ++i)
    {
      if (i==1)
        continue;

      CHECK_FALSE(detect_conflict(s_A[3], s_B[i], max_angle).is_conflict());
    }

    for (std::size_t i=0; i < s_B.size(); ++i)
      CHECK_FALSE(detect_conflict(s_A[4], s_B[i], max_angle).is_conflict());
  }
}

SCENARIO("Simple blockade geometry tests")
{
  using namespace rmf_traffic::blockade;
  const double radius = 0.1;
  const double max_angle = 1.0*M_PI/180.0;

  GIVEN("Simple cross-over")
  {
    const Segment A{{-5, 0}, {5, 0}, radius};
    const Segment B{{0, -5}, {0, 5}, radius};

    const auto info = detect_conflict(A, B, max_angle);
    CHECK(info.is_conflict());
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
  }

  GIVEN("Tight initial cross-over")
  {
    const Segment A{{-0.01, 0}, {5, 0}, radius};
    const Segment B{{0, -0.01}, {0, 5}, radius};

    const auto info = detect_conflict(A, B, max_angle);
    CHECK(info.is_conflict());
    CHECK(info.include_cap_a[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
    CHECK(info.include_cap_b[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
  }

  GIVEN("Tight final cross-over")
  {
    const Segment A{{5, 0}, {-0.01, 0}, radius};
    const Segment B{{0, 5}, {0, -0.01}, radius};

    const auto info = detect_conflict(A, B, max_angle);
    CHECK(info.is_conflict());
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  GIVEN("Tight initial-final cross-over")
  {
    const Segment A{{-0.01, 0}, {5, 0}, radius};
    const Segment B{{0, 5}, {0, -0.01}, radius};

    const auto info = detect_conflict(A, B, max_angle);
    CHECK(info.is_conflict());
    CHECK(info.include_cap_a[ConflictInfo::Start]);
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  GIVEN("Acute angle cross-over")
  {
    const double length = 20.0;
    const double separation = 4.0;
    const double half_s = 0.5*separation;
    const Segment A{{0,  half_s}, {length, -half_s}, radius};
    const Segment B{{0, -half_s}, {length,  half_s}, radius};

    {
      const auto info = detect_conflict(A, B, max_angle);
      CHECK(info.is_conflict());
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
    }

    const double angle = 2.0*atan2(separation, length);
    // This is a conflict because the endpoints are not included in the brackets
    // even though the angle is within the bound.
    CHECK(detect_conflict(A, B, angle + 1e-6).is_conflict());

    const Segment C{{length, half_s}, {0, -half_s}, radius};
    for (const double a : {max_angle, angle})
    {
      const auto info = detect_conflict(A, C, a);
      CHECK(info.is_conflict());
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_a[ConflictInfo::Finish]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
      CHECK_FALSE(info.include_cap_b[ConflictInfo::Finish]);
    }
  }

  GIVEN("Corner convergence")
  {
    const Segment A{{5, 0}, {0, 0}, radius};
    const Segment B{{0, 5}, {0, 0}, radius};

    const auto info = detect_conflict(A, B, max_angle);
    CHECK(info.is_conflict());
    CHECK_FALSE(info.include_cap_a[ConflictInfo::Start]);
    CHECK(info.include_cap_a[ConflictInfo::Finish]);
    CHECK_FALSE(info.include_cap_b[ConflictInfo::Start]);
    CHECK(info.include_cap_b[ConflictInfo::Finish]);
  }

  GIVEN("Corner divergence")
  {
    const Segment A{{0, 0}, {5, 0}, radius};
    const Segment B{{0, 0}, {0, 5}, radius};
    CHECK_FALSE(detect_conflict(A, B, max_angle).is_conflict());
  }

  GIVEN("Near miss")
  {
    const double h = 2*radius + 1e-4;
    const Segment A{{0, 0}, {5, 0}, radius};
    const Segment B{{5, h}, {0, h}, radius};
    CHECK_FALSE(detect_conflict(A, B, max_angle).is_conflict());
  }
}
