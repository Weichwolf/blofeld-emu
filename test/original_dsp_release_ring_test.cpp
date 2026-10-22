// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_voice_fixture.hpp"

int main(const int count, char **arguments) {
  return blofeld::test::run_voice_test(
      count, arguments, false, true, false, {}, blofeld::test::VoiceInput::stream_release);
}
