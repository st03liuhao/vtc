/*
C++ Virtualization Library of NEMA-TS2 Traffic Controller Assemblies
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

TEST_CASE("NotActive output can be changed") {
    io::variable<output::NotActive>.value = Bit::off;
    CHECK(io::variable<output::NotActive>.value == Bit::off);

    io::variable<output::NotActive>.value = Bit::on;
    CHECK(io::variable<output::NotActive>.value == Bit::on);

    CHECK(std::is_same_v<decltype(io::variable<output::NotActive>.value), std::atomic<Bit>>);
    CHECK(std::is_same_v<output::NotActive::value_t , Bit>);
    CHECK(std::is_same_v<output::NotActive::type, io::IOVariableType>);
}