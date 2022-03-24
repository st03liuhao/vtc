# Virtual Traffic Cabinet
A modern C++ library (C++20) for the virtualization of traffic cabinet in adherence with NEMA-TS1, NEMA-TS2, 170, Caltrans 232 and ATC Standards. 

## What does it do?
The library provides software models of Field IOs, MMU/CMU, CU, BIU/CIU/SIU and SDLC serial bus, thus enables running tests of various cabinet devices in a flexible software-defined virtual environment. 

## What is the catch?
The "beauty" - well, in my eyes, is that it allows a functional/declarative programming style for composing various signal control elements. It is essentially a C++ meta type library for traffic engineering devices. 
    
For example, we can define existing and any future (that yet to be defined) I/O modules, and the library infrastructure will automatically handle the data frame generation and parsing the binary stream automatically. In other words, the user can "compose" (i.e., define) I/O modules in a declarative fashion, without worrying how the related serial frames will be parsed - the latter will be taken care of by the framework automatically.

Other features of this library:

- It serves as the foundation to further develop Controller Interface Device (CID) for hardware-in-loop-simulation (HILS).
- Fast and simple
- Modern C++ concept driven, and test driven development
- Fully tested with unit testing cases

## Compiler
Requires a C++ compiler that supports C++20.

## Example

The following code illustrates a use case that an incoming SDLC Type 3 command frame is received, and a corresponding Type 131 response frame is generated based on the MMU channel compatibility status data.

First, the incoming data - as read from the serial bus by the serial adapter. Type 3 command frame is for MMU programmed compatibility request. 
```
// l_data_in read from serial bus by the serial adapter
std::array<Byte, 3> l_data_in = {0x10, 0x83, 0x03};
```
The received Type 3 command frame is dispatched as follows, in order to get a properly generated Type 131 response frame

```
// MMU channel compatibility status data converted to Type 131 response frame.
auto result = serial::Dispatch(l_data_in);
```
The returned ```result``` is a tuple of ```<bool, std::span<Byte>>```.  The span is the generated response frame, which can be transmitted back to the CU via the serial bus.
```
// Check the frame size is correct
assert(std::get<1>(result).size() == serial::FrameType<131>::type::bytesize);

// Check Byte 3 for Channel 1 and Channel 2 compatibility
assert(std::get<1>(result)[3] == 0x01); // Byte 3 represents CH1-CH2 compatibility
```


## Further Usage

Examine the [test cases](https://github.com/wxinix/vtc/blob/master/test/vtc_test.cpp) to learn how to use this library.