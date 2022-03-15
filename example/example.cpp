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

using namespace atc::fio::output;
using namespace atc::fio;
using namespace atc;

int main() {
    std::cout << atc::cu::phase::max_phase_groups << "\n";
    auto& b = fio::fio<NotActive>.state;
    std::cout << uint16_t(State(b)) << "\n";
    fio::fio<NotActive>.state = State::on;
    std::cout << uint16_t(State(b)) << "\n";
    return 0;
}