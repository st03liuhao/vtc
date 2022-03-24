# Virtual Traffic Cabinet
A modern C++ library (C++20) for the virtualization of traffic controller cabinet in adherence with NEMA-TS1, NEMA-TS2, 170, Caltrans 232 and ATC Standards. 

## What does it do?
The library provides software models of Field IOs, MMU/CMU, CU, BIU/CIU/SIU and SDLC serial bus, thus enables running tests of various cabinet devices in a flexible software-defined virtual environment. 

## What is the catch?
The "beauty" - well, in my eyes, is that it allows a functional/declarative programming style for composing various signal control elements. It is essentially is a C++ meta type library for traffic engineering devices. 
    
For example, we can define existing and any future (that yet to be defined) I/O modules, and the library infrastructure will automatically handle the data frame generation and parsing the binary stream automatically. In other words, the user can "compose" (i.e., define) I/O modules in a declarative fashion, without worrying how the related serial frames will be parsed - the latter will be taken care of by the framework automatically.

Other features of this library:

- It serves as the basis to further develop Controller Interface Device (CID) for hardware-in-loop-simulation (HILS).
- Fast and simple
- Modern C++ concept driven, and test driven; fully tested with unit testing cases

## Compiler
Requires a C++ compiler that supports C++20.

## Example
```
// l_data_in comes from a serial adaptor's read data
// in this example, it is a Type 3 Command Frame asking for MMU programmed
// channel compatibility
std::array<Byte, 3> l_data_in = {0x10, 0x83, 0x03};

// We simply pass along the Type 3 Command Frame to 
// serial::Dispatch, which will automatically
// obtain relevant MMU channel compatibility status data
// and converted to Type 131 response frame.
auto result = serial::Dispatch(l_data_in);

// Tye 131 Response Frame bytes
CHECK(std::get<1>(result).size() == serial::FrameType<131>::type::bytesize);
CHECK(std::get<1>(result)[3] == 0x01); // Byte 3 represents CH1-CH2 compatibility
CHECK(std::get<1>(result)[4] == 0x00);
```

## Further Usage

Examine the [test cases](https://github.com/wxinix/vtc/blob/master/test/vtc_test.cpp) to learn how to use this library.