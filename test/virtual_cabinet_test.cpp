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

using namespace atc::io;
using namespace atc;

TEST_CASE("io::output::NotActive can be set") {
  io::variable<output::NotActive>.value = Bit::off;
  CHECK(io::variable<output::NotActive>.value == Bit::off);

  io::variable<output::NotActive>.value = Bit::on;
  CHECK(io::variable<output::NotActive>.value == Bit::on);

  CHECK(std::is_same_v<decltype(io::variable<output::NotActive>.value), std::atomic<Bit>>);
  CHECK(std::is_same_v<output::NotActive::value_t, Bit>);
  CHECK(std::is_same_v<output::NotActive::type, IOVariableType>);
}

TEST_CASE("io::output::ChannelGreenWalkDriver can be set") {
  io::variable<output::ChannelGreenWalkDriver<1>>.value = Bit::off;
  CHECK(io::variable<output::ChannelGreenWalkDriver<1>>.value == Bit::off);

  io::variable<output::ChannelGreenWalkDriver<1>>.value = Bit::on;
  CHECK(io::variable<output::ChannelGreenWalkDriver<1>>.value == Bit::on);
}

TEST_CASE("mmu::LoadSwitchFlash can be set") {
  mmu::variable<mmu::LoadSwitchFlash>.value = Bit::off;
  CHECK(mmu::variable<mmu::LoadSwitchFlash>.value == Bit::off);

  mmu::variable<mmu::LoadSwitchFlash>.value = Bit::on;
  CHECK(mmu::variable<mmu::LoadSwitchFlash>.value == Bit::on);
}

TEST_CASE("FrameBit can be instantiated") {
  FrameBit<mmu::LoadSwitchFlash, 127> l_framebit;
  CHECK(l_framebit.pos == 127);
}

TEST_CASE("mmu::LoadSwitchDriverFrame can be parsed") {
  mmu::LoadSwitchDriverFrame l_frame;
  std::array<Byte, 16>
      l_data = {0x10, 0x83, 0x00, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
  l_frame << l_data;

  CHECK(mmu::variable<mmu::LoadSwitchFlash>.value == Bit::on);

  CHECK(mmu::variable<mmu::ChannelGreenWalkDriver<1>>.value == Bit::on);
  CHECK(mmu::variable<mmu::ChannelGreenWalkDriver<2>>.value == Bit::off);
  CHECK(mmu::variable<mmu::ChannelGreenWalkDriver<3>>.value == Bit::off);
  CHECK(mmu::variable<mmu::ChannelGreenWalkDriver<4>>.value == Bit::on);
}

TEST_CASE("mmu::InputStatusRequestFrame can be parsed") {
  mmu::InputStatusRequestFrame  l_frame;
  std::array<Byte, 3> l_data = {0x10, 0x83, 0x01};
  l_frame << l_data;

  CHECK(l_frame.frame_id == 0x01);
}
