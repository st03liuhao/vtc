/*
C++ Virtualization Library of Traffic Control Cabinet
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
#include <vtc.hpp>

using namespace vtc;

TEST_CASE("Channel composition channel IDs can be encoded as single index")
{
  Byte a = 1;
  Byte b = 2;

  Index I = a << 8 | b;
  CHECK(I == 0x0102);
}

TEST_CASE("Compile-time internal variable name can be retrieved")
{
  auto l_name_1 = variable_type_name<mmu::_24VoltMonitor_I>();
  mmu::_24VoltMonitor_I l_var;
  CHECK(l_var.value == Bit::Off);
  auto l_name_2 = variable_type_name<decltype(l_var)>();
  CHECK(l_name_1 == l_name_2);
}

TEST_CASE("FrameBit can be instantiated")
{
  serial::FrameBit<mmu::LoadSwitchFlash, 127> l_framebit;
  CHECK(l_framebit.pos == 127);
}

TEST_CASE("Output variable NotActive can be set")
{
  using namespace io;

  io::variable<output::NotActive>.value = Bit::Off;
  CHECK(io::variable<output::NotActive>.value == Bit::Off);

  io::variable<output::NotActive>.value = Bit::On;
  CHECK(io::variable<output::NotActive>.value == Bit::On);

  CHECK(std::is_same_v<decltype(io::variable<output::NotActive>.value), std::atomic<Bit>>);
  CHECK(std::is_same_v<output::NotActive::value_t, Bit>);
  CHECK(std::is_same_v<output::NotActive::type, IoVariableType>);
}

TEST_CASE("Output variable ChannelGreenWalkDriver can be set")
{
  using namespace io;

  io::variable<output::ChannelGreenWalkDriver<1>>.value = Bit::Off;
  CHECK(io::variable<output::ChannelGreenWalkDriver<1>>.value == Bit::Off);

  io::variable<output::ChannelGreenWalkDriver<1>>.value = Bit::On;
  CHECK(io::variable<output::ChannelGreenWalkDriver<1>>.value == Bit::On);
}

TEST_CASE("MMU variable LoadSwitchFlash can be set")
{
  using namespace mmu;

  mmu::variable<LoadSwitchFlash>.value = Bit::Off;
  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::Off);

  mmu::variable<LoadSwitchFlash>.value = Bit::On;
  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::On);
}

TEST_CASE("MMU variable LoadSwitchDriverFrame can be parsed")
{
  using namespace mmu;

  serial::LoadSwitchDriversFrame l_frame_1;
  std::array<Byte, 16> l_data = {0x10, 0x83, 0x00};
  l_data[0x03] = 0xC3;
  l_data[0x0F] = 0x80;
  l_frame_1 << l_data;

  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::On);

  CHECK(mmu::variable<ChannelGreenWalkDriver<1>>.value == Bit::On);
  CHECK(mmu::variable<ChannelGreenWalkDriver<2>>.value == Bit::Off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<3>>.value == Bit::Off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<4>>.value == Bit::On);

  l_data = {0x10, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  serial::FrameType<0>::type l_frame_2;
  l_frame_2 << l_data;

  CHECK(mmu::variable<LoadSwitchFlash>.value == Bit::Off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<1>>.value == Bit::Off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<2>>.value == Bit::Off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<3>>.value == Bit::Off);
  CHECK(mmu::variable<ChannelGreenWalkDriver<4>>.value == Bit::Off);
}

TEST_CASE("Type 1 serial frame InputStatusRequestFrame can be parsed")
{
  serial::MMUInputStatusRequestFrame l_frame;
  std::array<Byte, 3> l_data = {0x10, 0x83, 0x01};
  l_frame << l_data;

  CHECK(l_frame.id == 0x01);
}

TEST_CASE("Type 3 serial frame MMUProgrammingRequestFrame can be parsed")
{
  serial::MMUProgrammingRequestFrame l_frame;
  std::array<Byte, 3> l_data = {0x10, 0x83, 0x03};
  l_frame << l_data;

  CHECK(l_frame.id == 0x03);
}

TEST_CASE("Type 3 serial frame MMUProgrammingRequestFrame can be dispatched")
{
  // Type 3 Command Frame bytes
  std::array<Byte, 3> l_data_in = {0x10, 0x83, 0x03};
  mmu::variable<mmu::ChannelCompatibilityStatus<0x01, 0x02>>.value = Bit::On;
  auto result = serial::Dispatch(l_data_in);
  CHECK(std::get<0>(result));
  // Tye 131 Response Frame bytes
  CHECK(std::get<1>(result).size() == serial::FrameType<131>::type::bytesize);
  CHECK(std::get<1>(result)[3] == 0x01); // Byte 3 represents CH1-CH2 compatibility
  CHECK(std::get<1>(result)[4] == 0x00);
}

TEST_CASE("Type 9 serial frame DateTimeBroadcastFrame can be parsed")
{
  serial::DateTimeBroadcastFrame l_frame;
  std::array<Byte, 12>     // M     D     Y     H     M     S     0.1   TF    DET
  l_data = {0xFF, 0x83, 0x09, 0x03, 0x12, 0x16, 0x11, 0x20, 0x00, 0x00, 0x01, 0x02}; // 03/18/2022, 17:32:00.0
  l_frame << l_data;

  using namespace broadcast;

  CHECK(l_frame.id == 0x09);
  CHECK(broadcast::variable<CuReportedDay>.value == 18);
  CHECK(broadcast::variable<CuReportedMonth>.value == 3);
  CHECK(broadcast::variable<CuReportedYear>.value == 22);
  CHECK(broadcast::variable<CuReportedHour>.value == 17);
  CHECK(broadcast::variable<CuReportedMinutes>.value == 32);
  CHECK(broadcast::variable<CuReportedSeconds>.value == 0);
  CHECK(broadcast::variable<CuReportedTenthsOfSeconds>.value == 0);
  CHECK(broadcast::variable<CuReportedTfBiuPresence<1>>.value == Bit::On);
  CHECK(broadcast::variable<CuReportedDrBiuPresence<2>>.value == Bit::On);
}