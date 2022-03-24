# Virtual Traffic Cabinet
A modern C++ library (C++20) for the virtualization of traffic controller cabinet in adherence with NEMA-TS1, NEMA-TS2, 170, Caltrans 232 and ATC Standards. 

The library provides software models of Field IOs, MMU/CMU, CU, BIU/CIU/SIU and SDLC serial bus, thus enables running tests of various cabinet devices in a flexible software-defined virtual environment. 

The "beauty" - well, in my eyes of this lib, is that it allows a functional/declarative programming style for composing various signal control elements. It is essentially is a C++ meta type library for traffic engineering devices. For example, we can define existing and any future (that yet to be defined) I/O modules, and the library infrastructure will automatically handle the data frame generation and parsing the binary stream automatically. In other words, the use can "compose" (i.e., define) I/O modules in a declarative fashion, without worrying how the related serial frames will be parsed - the latter will be taken care of by the framework automatically.

It can also be used as the basis to further develop Controller Interface Device (CID) for hardware-in-loop-simulation (HILS).

## Compiler
Requires a C++ compiler that supports C++20.

## Road Map

## Example

## Further Usage

Examine the [test cases](https://github.com/wxinix/virtual_cabinet/blob/master/test/virtual_cabinet_test.cpp) to learn how to use this library.