// Copyright 2026 Gabriel-Ning
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

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "marvin/FxRtCSDef.h"
#include "marvin/MarvinSDK.h"

namespace
{

bool parse_ip(const std::string & ip, unsigned char octets[4])
{
  int a, b, c, d;
  if (std::sscanf(ip.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4) {
    return false;
  }
  if (a < 0 || a > 255 || b < 0 || b > 255 || c < 0 || c > 255 || d < 0 || d > 255) {
    return false;
  }
  octets[0] = static_cast<unsigned char>(a);
  octets[1] = static_cast<unsigned char>(b);
  octets[2] = static_cast<unsigned char>(c);
  octets[3] = static_cast<unsigned char>(d);
  return true;
}

void print_errors(const char * arm_name, const long errs[7])
{
  std::printf("  %s Servo Errors: [", arm_name);
  for (int i = 0; i < 7; ++i) {
    std::printf("%ld%s", errs[i], (i == 6 ? "" : ", "));
  }
  std::printf("]\n");
}

bool has_active_errors(const long errs[7])
{
  for (int i = 0; i < 7; ++i) {
    if (errs[i] != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string target_ip = "10.19.0.191";

  for (int i = 1; i < argc; ++i) {
    if ((std::strcmp(argv[i], "--ip") == 0 || std::strcmp(argv[i], "-i") == 0) && i + 1 < argc) {
      target_ip = argv[++i];
    } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      std::printf("Usage: ros2 run marvin_manipulation_rt_launch clear_errors [--ip <controller_ip>]\n");
      std::printf("Default IP: 10.19.0.191\n");
      return 0;
    }
  }

  unsigned char octets[4] = {0};
  if (!parse_ip(target_ip, octets)) {
    std::fprintf(stderr, "[X] Error: Invalid IP address: %s\n", target_ip.c_str());
    return 1;
  }

  std::printf("\n=======================================================\n");
  std::printf("  Marvin Hardware Interface: Clear Servo Errors\n");
  std::printf("  Target Controller IP: %s\n", target_ip.c_str());
  std::printf("=======================================================\n\n");

  std::printf("[1/4] Connecting to Marvin controller at %s...\n", target_ip.c_str());
  if (!OnLinkTo(octets[0], octets[1], octets[2], octets[3])) {
    std::fprintf(stderr, "[X] Connection failed! Check Ethernet cable, subnet, and controller power.\n");
    return 1;
  }

  long version = OnGetSDKVersion();
  std::printf("  [✓] Connected! Native Marvin SDK Version: %ld\n", version);

  // Wait for initial telemetry frame
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::printf("\n[2/4] Reading active servo error status...\n");
  long err_a[7] = {0};
  long err_b[7] = {0};
  OnGetServoErr_A(err_a);
  OnGetServoErr_B(err_b);

  print_errors("Arm A (Left) ", err_a);
  print_errors("Arm B (Right)", err_b);

  bool need_clear = has_active_errors(err_a) || has_active_errors(err_b);
  if (!need_clear) {
    std::printf("  -> No active servo errors reported on either arm.\n");
  }

  std::printf("\n[3/4] Dispatching Clear Error commands to Arm A & Arm B...\n");
  if (!OnClearSet()) {
    std::fprintf(stderr, "[!] Warning: OnClearSet returned false, attempting direct clear...\n");
  }
  OnClearErr_A();
  OnClearErr_B();
  if (!OnSetSend()) {
    std::fprintf(stderr, "[!] Warning: OnSetSend returned false.\n");
  }

  // Allow controller to process reset
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  std::printf("\n[4/4] Verifying post-clear hardware status...\n");
  long post_err_a[7] = {0};
  long post_err_b[7] = {0};
  OnGetServoErr_A(post_err_a);
  OnGetServoErr_B(post_err_b);

  print_errors("Arm A (Left) ", post_err_a);
  print_errors("Arm B (Right)", post_err_b);

  bool all_clear = !has_active_errors(post_err_a) && !has_active_errors(post_err_b);

  // Disconnect cleanly
  OnRelease();
  std::printf("  [✓] SDK connection released.\n");

  if (all_clear) {
    std::printf("\n[✓] All servo errors successfully cleared! Marvin is ready for RT bringup.\n\n");
    return 0;
  } else {
    std::fprintf(stderr, "\n[!] Warning: Non-zero servo errors remain. Check E-stop and hardware interlocks.\n\n");
    return 1;
  }
}
