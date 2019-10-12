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
#include <memory>
#include "mynteye/api/version_checker.h"
#include "mynteye/device/utils.h"
#include "mynteye/logger.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

typedef struct {
  const std::string device_type;
  const std::string sdk_version;
  const std::string firmware_version;
  const std::string status;
}firmware_version_match_table_unit;

const char* ERRO_DESCRIPTION_F =
    "Please update the firmware at first;\n"
    "Read the doc (https://mynt-eye-s-sdk.readthedocs.io/en/latest/src/firmware/applicable.html) "  // NOLINT
    "to learn more.";
const char* ERRO_DESCRIPTION_S =
    "Please update the SDK at first;\n"
    "Read the doc (https://mynt-eye-s-sdk.readthedocs.io/en/latest/src/firmware/applicable.html) "  // NOLINT
    "to learn more.";
const char* WARN_DESCRIPTION_F =
    "We suggest that you should update the firmware;\n"
    "Read the doc (https://mynt-eye-s-sdk.readthedocs.io/en/latest/src/firmware/applicable.html) "  // NOLINT
    "to learn more.";
const char* WARN_DESCRIPTION_S =
    "We suggest that you should update the SDK;\n"
    "Read the doc (https://mynt-eye-s-sdk.readthedocs.io/en/latest/src/firmware/applicable.html) "  // NOLINT
    "to learn more.";
const char* PASS_DESCRIPTION = "pass";
const char* PASS_OUTOF_TABLE_WARNING = "You're using a custom mynteye device";

/** firmware/sdk version matched table */
/**----device type-----sdk version---firmware version-----pass tag-----*/
static const firmware_version_match_table_unit FSVM_TABLE[] ={
/** S1030 */
  {"MYNT-EYE-S1030",    ">2.3.3",         ">2.3.0",   PASS_DESCRIPTION},
  {"MYNT-EYE-S1030",    ">2.3.3",         "<2.4.0",   WARN_DESCRIPTION_F},
  {"MYNT-EYE-S1030",    ">0.0.0",         ">2.2.0",   PASS_DESCRIPTION},
  {"MYNT-EYE-S1030",    ">0.0.0",         "<2.3.0",   WARN_DESCRIPTION_F},
/** S2110 */
  {"MYNT-EYE-S2110",    ">2.4.1",         "2.1",       PASS_DESCRIPTION},
  {"MYNT-EYE-S2110",    ">2.4.1",         "1.5",       PASS_DESCRIPTION},
  {"MYNT-EYE-S2110",    ">2.4.1",         "1.4",       PASS_DESCRIPTION},
  {"MYNT-EYE-S2110",    "2.4.1",          "1.4",       PASS_DESCRIPTION},
  {"MYNT-EYE-S2110",    "2.4.1",          "<1.4",      WARN_DESCRIPTION_F},
  {"MYNT-EYE-S2110",    "<2.4.1",         "<1.4",      WARN_DESCRIPTION_S},
/** S2100 */
  {"MYNT-EYE-S2100",    "2.3.5",          "<1.2",      WARN_DESCRIPTION_F},
  {"MYNT-EYE-S2100",    ">2.3.3",         ">1.0",      PASS_DESCRIPTION},
  {"MYNT-EYE-S2100",    ">2.3.3",         "<1.1",      WARN_DESCRIPTION_F},
  {"MYNT-EYE-S2100",    ">2.3.0",         ">0.9",      PASS_DESCRIPTION},
  {"MYNT-EYE-S2100",    ">0.0.0",         ">0.9",      WARN_DESCRIPTION_S},
  {"MYNT-EYE-S2100",    ">0.0.0",         "<1.0",      WARN_DESCRIPTION_F},
/** S210A */
  {"MYNT-EYE-S210A",    "2.3.5",          "<1.2",      WARN_DESCRIPTION_F},
  {"MYNT-EYE-S210A",    ">2.3.3",         ">1.0",      PASS_DESCRIPTION},
  {"MYNT-EYE-S210A",    ">2.3.3",         "<1.1",      WARN_DESCRIPTION_F},
  {"MYNT-EYE-S210A",    ">2.3.0",         ">0.9",      PASS_DESCRIPTION},
  {"MYNT-EYE-S210A",    ">0.0.0",         ">0.9",      WARN_DESCRIPTION_S},
  {"MYNT-EYE-S210A",    ">0.0.0",         "<1.0",      WARN_DESCRIPTION_F},
};

void getVersion(const std::string &str, char *version) {
  std::string st1("");
  int j = 0;
  for (size_t i = 0; i < str.size(); i++) {
    if (str[i] == '.') {
      version[j++] = atoi(st1.c_str());
      st1 = "";
    } else {
      st1 += str[i];
    }
  }
  version[j++] = atoi(st1.c_str());
}

bool conditionMatch(const std::string& condition, const std::string& target) {
  char version[4] = {0};
  char version_c[4] = {0};
  getVersion(target, version);
  int tag_c = 0;
  std::string condition_c;
  if (condition[0] == '>') {
    tag_c = 1;
    condition_c = condition.substr(1);
  } else if (condition[0] == '<') {
    tag_c = -1;
    condition_c = condition.substr(1);
  } else {
    tag_c = 0;
    condition_c = condition;
  }
  getVersion(condition_c, version_c);
  int tag_big = memcmp(version, version_c, 4);
  if (tag_big * tag_c > 0 || (tag_big == 0 && tag_c == 0)) return true;
  return false;
}

enum STATUS_UNIT {
  ST_PASS,
  ST_ERRO_F,
  ST_ERRO_S,
  ST_NOT_PASS
};

STATUS_UNIT checkUnit(const std::string& sdkv,
    const std::string& devn,
    const std::string& firmv,
    const firmware_version_match_table_unit& condition) {
    if (condition.device_type == devn &&
        conditionMatch(condition.sdk_version, sdkv) &&
        conditionMatch(condition.firmware_version, firmv)) {
      if (condition.status == ERRO_DESCRIPTION_F) return ST_ERRO_F;
      if (condition.status == ERRO_DESCRIPTION_S) return ST_ERRO_S;
      if (condition.status == WARN_DESCRIPTION_F ||
          condition.status == WARN_DESCRIPTION_S) {
        LOG(WARNING) << condition.status;
      }
      return ST_PASS;
    }
  return ST_NOT_PASS;
}

bool checkIfDeviceInTable(const std::string& devn) {
  for (size_t i =0;
      i < sizeof(FSVM_TABLE)/sizeof(firmware_version_match_table_unit);
      i++) {
    if (FSVM_TABLE[i].device_type == devn) {
      return true;
    }
  }
  return false;
}

bool checkFirmwareVersion(const std::shared_ptr<API> api) {
  auto sdkv = api->GetSDKVersion();
  std::cout << sdkv;
  auto devn = api->GetInfo(Info::DEVICE_NAME);
  auto firmv = api->GetInfo(Info::FIRMWARE_VERSION);

  if (!checkIfDeviceInTable(devn)) {
    LOG(WARNING) << PASS_OUTOF_TABLE_WARNING;
    return true;
  }

  for (size_t i =0;
      i < sizeof(FSVM_TABLE)/sizeof(firmware_version_match_table_unit);
      i++) {
    auto res = checkUnit(sdkv, devn, firmv, FSVM_TABLE[i]);
    if (res == ST_PASS) {
      return true;
    } else if (res == ST_ERRO_S || res == ST_ERRO_F) {
      LOG(ERROR) << FSVM_TABLE[i].status;
      return false;
    }
  }
  LOG(ERROR) << ERRO_DESCRIPTION_S;
  return false;
}

MYNTEYE_END_NAMESPACE

