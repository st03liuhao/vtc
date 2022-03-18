/*
C++ Virtualization Library of Traffic Controller Cabinet
Copyright (C) 2022  Wuping Xin

GNU GPL v3 License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or at
your option any later version.

This program is distributed in the hope that it will be useful,  but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANT-
ABILITY or FITNESS FOR A PARTICULAR PURPOSE.

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>
#include <virtual_cabinet.hpp>

using namespace atc;

TEST_CASE("io::output::NotActive can be set") {
  using namespace io;

  io::variable<output::NotActive>.value = Bit::off;
  CHECK(io::variable<output::NotActive>.value == Bit::off);

  io::variable<output::NotActive>.value = Bit::on;
  CHECK(io::variable<output::NotActive>.value == Bit::on);

  CHECK(std::is_same_v<decltype(io::variable<output::NotActive>.value), std::atomic<Bit>>);
  CHECK(std::is_same_v<output::NotActive::value_t, Bit>);
  CHECK(std::is_same_v<output::NotActive::type, IOVariableType>);
}

TEST_CASE("io::output::ChannelGreenWalkDriver can be set") {
  using namespace io;

  io::variable<output::ChannelGreenWalkDriver<1>>.value = Bit::off;
  CHECK(io::variable<output::ChannelGreenWalkDriver<1>>.value == Bit::off);

  io::variable<output::ChannelGreenWalkDriver<1>>.value = Bit::on;
  CHECK(io::variable<output::ChannelGreenWalkDriver<1>>.value == Bit::on);
}

TEST_CASE("mmu::LoadSwitchFlash can be set") {
  using namespace mmu;

  mmu::variable<LoadSwitchFlash>.value = Bit::off;
  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::off);

  mmu::variable<LoadSwitchFlash>.value = Bit::on;
  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::on);
}

TEST_CASE("FrameBit can be instantiated") {
  using namespace mmu;

  FrameBit<LoadSwitchFlash, 127> l_framebit;
  CHECK(l_framebit.pos == 127);
}

TEST_CASE("mmu::LoadSwitchDriverFrame can be parsed") {
  using namespace mmu;

  LoadSwitchDriverFrame l_frame;
  std::array<Byte, 16>
      l_data = {0x10, 0x83, 0x00, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
  l_frame << l_data;

  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::on);

  CHECK(mmu::variable<ChannelGreenWalkDriver<1>>.value == Bit::on);
  CHECK(mmu::variable<ChannelGreenWalkDriver<2>>.value == Bit::off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<3>>.value == Bit::off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<4>>.value == Bit::on);
}

TEST_CASE("mmu::InputStatusRequestFrame can be parsed") {
  using namespace mmu;

  InputStatusRequestFrame l_frame;
  std::array<Byte, 3> l_data = {0x10, 0x83, 0x01};
  l_frame << l_data;

  CHECK(l_frame.frame_id == 0x01);
}

TEST_CASE("mmu::MMUProgrammingRequestFrame can be parsed") {
  using namespace mmu;

  MMUProgrammingRequestFrame l_frame;
  std::array<Byte, 3> l_data = {0x10, 0x83, 0x03};
  l_frame << l_data;

  CHECK(l_frame.frame_id == 0x03);
}

TEST_CASE("MMU can receive Date and Time Broadcast Command Frame Type 0") {
  using namespace atc::mmu;

  DateTimeBroadcastFrame l_frame;
  std::array<Byte, 12>     // M     D     Y     H     M     S     0.1   TF    DET
  l_data = {0xFF, 0x83, 0x09, 0x03, 0x12, 0x16, 0x11, 0x20, 0x00, 0x00, 0x01, 0x02}; // 03/18/2022, 17:32:00.0
  l_frame << l_data;

  CHECK(l_frame.frame_id == 0x09);
  CHECK(mmu::variable<CUReportedDay>.value == 18);
  CHECK(mmu::variable<CUReportedMonth>.value == 3);
  CHECK(mmu::variable<CUReportedYear>.value == 22);
  CHECK(mmu::variable<CUReportedHour>.value == 17);
  CHECK(mmu::variable<CUReportedMinutes>.value == 32);
  CHECK(mmu::variable<CUReportedSeconds>.value == 0);
  CHECK(mmu::variable<CUReportedTenthsOfSeconds>.value == 0);
  CHECK(mmu::variable<TFBIUPresent<1>>.value == Bit::on);
  CHECK(mmu::variable<DETBIUPresent<2>>.value == Bit::on);
}