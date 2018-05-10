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
#include "gtest/gtest.h"

#include "internal/types.h"

MYNTEYE_USE_NAMESPACE

class VersionTest : public ::testing::Test {
 protected:
  VersionTest()
      : ver_1_0(Version(1, 0)),
        ver_1_2(Version(1, 2)),
        ver_2_0(Version(2, 0)) {}

  virtual ~VersionTest() {}

  virtual void SetUp() {}

  virtual void TearDown() {}

  Version ver_1_0;
  Version ver_1_2;
  Version ver_2_0;
};

TEST_F(VersionTest, Properties) {
  EXPECT_EQ(1, ver_1_0.major());
  EXPECT_EQ(0, ver_1_0.minor());
}

TEST_F(VersionTest, Operators) {
  EXPECT_TRUE(ver_1_0 == ver_1_0);
  EXPECT_TRUE(ver_1_0 <= ver_1_2);
  EXPECT_TRUE(ver_1_0 != ver_1_2);
  EXPECT_TRUE(ver_1_0 < ver_1_2);
  EXPECT_TRUE(ver_1_2 > ver_1_0);
  EXPECT_TRUE(ver_1_2 >= ver_1_0);

  EXPECT_TRUE(ver_1_0 <= ver_1_0);
  EXPECT_TRUE(ver_1_2 >= ver_1_2);

  EXPECT_TRUE(ver_1_2 == ver_1_2);
  EXPECT_TRUE(ver_1_2 <= ver_2_0);
  EXPECT_TRUE(ver_1_2 != ver_2_0);
  EXPECT_TRUE(ver_1_2 < ver_2_0);
  EXPECT_TRUE(ver_2_0 > ver_1_2);
  EXPECT_TRUE(ver_2_0 >= ver_1_2);

  EXPECT_TRUE(ver_1_2 <= ver_1_2);
  EXPECT_TRUE(ver_2_0 >= ver_2_0);
}

TEST_F(VersionTest, Between) {
  EXPECT_TRUE(ver_1_0.is_between(ver_1_0, ver_2_0));
  EXPECT_TRUE(ver_1_2.is_between(ver_1_0, ver_2_0));
  EXPECT_TRUE(ver_2_0.is_between(ver_1_0, ver_2_0));

  EXPECT_FALSE(ver_1_0.is_between(ver_1_2, ver_2_0));
  EXPECT_FALSE(ver_2_0.is_between(ver_1_0, ver_1_2));
}

TEST_F(VersionTest, Strings) {
  EXPECT_EQ("1.0", ver_1_0.to_string());
  EXPECT_EQ("1.2", ver_1_2.to_string());
  EXPECT_EQ("2.0", ver_2_0.to_string());

  EXPECT_EQ("1.0", Version("1.0").to_string());
  EXPECT_EQ("1.2", Version("1.2").to_string());
  EXPECT_EQ("2.0", Version("2.0").to_string());
}

class HardwareVersionTest : public ::testing::Test {
 protected:
  HardwareVersionTest() : ver_1_0(HardwareVersion(1, 0)) {}

  HardwareVersion ver_1_0;
};

TEST_F(HardwareVersionTest, Properties) {
  EXPECT_EQ(1, ver_1_0.major());
  EXPECT_EQ(0, ver_1_0.minor());
}

TEST_F(HardwareVersionTest, Strings) {
  EXPECT_EQ("1.0", ver_1_0.to_string());
  EXPECT_EQ("1.0", HardwareVersion("1.0").to_string());
}

TEST_F(HardwareVersionTest, Flags) {
  auto flag = ver_1_0.flag();
  EXPECT_EQ("00000000", flag.to_string());

  flag[0] = true;
  flag[flag.size() - 1] = true;
  EXPECT_EQ("10000001", flag.to_string());
}

class TypeTest : public ::testing::Test {
 protected:
  TypeTest() : type_1_2a(Type(1, 0x2A)) {}

  Type type_1_2a;
};

TEST_F(TypeTest, Properties) {
  EXPECT_EQ(1, type_1_2a.vendor());
  EXPECT_EQ(0x2A, type_1_2a.product());
}

TEST_F(TypeTest, Strings) {
  EXPECT_EQ("012A", type_1_2a.to_string());
  EXPECT_EQ("010A", Type("010A").to_string());
}
