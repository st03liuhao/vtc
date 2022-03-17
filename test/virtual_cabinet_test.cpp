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

#include <iostream>
#include <virtual_cabinet.hpp>

using namespace atc::io::output;
using namespace atc;

int main() {
    std::cout << "max phases: " << atc::cu::phase::max_phase_groups << "\n";
    auto& b = io::io<NotActive>.value;
    std::cout << "expect false: " << static_cast<bool>(Bit(b)) << "\n";
    io::io<NotActive>.value = Bit::on;
    std::cout << "expect true: " << static_cast<bool>(Bit(b)) << "\n";
    std::cout << "expected false: " << std::is_same_v<io::IOVariableType, decltype(io::io<NotActive>.value)> << "\n";
    std::cout << "expected true: " << std::is_same_v<io::IOVariableType, NotActive::type> << "\n";
    return 0;
}