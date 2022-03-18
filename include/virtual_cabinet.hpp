/*
C++ Virtualization Library of Traffic Controller Cabinet
Copyright (C) 2022  Wuping Xin

GNU GPL v3 License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or at
your option any later version.

This program is  distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANT-
ABILITY or FITNESS FOR A PARTICULAR PURPOSE.

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef _VIRTUAL_CABINET
#define _VIRTUAL_CABINET

#include <atomic>
#include <cassert>
#include <span>

namespace atc {

// A dirty trick to generate template specialization tag.
#define AUTO_TAG_ID __LINE__

enum class Bit : bool {
  off = false, on = true
};

using Byte = uint8_t;
using Integer = uint32_t;
using Word = uint16_t;

using index_t = uint16_t;
using size_t = uint16_t;
using tag_t = int;

constexpr bool IsValidIndex(index_t I, size_t N) {
  return (I >= 1) && (I <= N);
};

template<typename T>
concept is_valid_variable_value_t
= std::is_same_v<T, Bit> || std::is_same_v<T, Byte> || std::is_same_v<T, Word> || std::is_same_v<T, Integer>;

template<typename ValueT, index_t I> requires is_valid_variable_value_t<ValueT>
struct Variable {
  using value_t = ValueT;

  Variable() = default;
  Variable(Variable &) = delete;
  Variable(Variable &&) = delete;
  Variable &operator=(Variable &) = delete;
  Variable &operator=(Variable &&) = delete;

  static constexpr index_t index{I};
  std::atomic<ValueT> value{};
};

template<typename T>
using ValueType = typename T::Variable::value_t;

template<typename T>
concept is_variable = requires {
  T::Variable::value_t;
};

namespace cu {

struct ControllerUnitVariableType {
};

template<typename T>
concept is_cu_variable = std::is_same_v<typename T::type, ControllerUnitVariableType>;

struct Phase {
  static constexpr size_t max_Phases{40};
  static constexpr size_t max_Phase_groups{5};
};

struct Detector {
  static constexpr size_t max_vehicle_detectors{128};
  static constexpr size_t max_vehicle_detector_status_groups{40};
  static constexpr size_t max_pedestrian_detectors{72};
};

struct Ring {
  static constexpr size_t max_rings{16};
  static constexpr size_t max_sequences{20};
  static constexpr size_t max_ring_control_groups{2};
};

struct Channel {
  static constexpr size_t max_Channels{32};
  static constexpr size_t max_Channel_status_groups{4};
};

struct Overlap {
  static constexpr size_t max_Overlaps{32};
  static constexpr size_t max_Overlap_status_groups{4};
};

struct Preempt {
  static constexpr size_t max_preempts{40};
};

struct Unit {
  static constexpr size_t max_alarm_groups{1};
  static constexpr size_t max_special_function_outputs{16};
};

struct Coord {
  static constexpr size_t max_patterns{128};
  static constexpr size_t max_splits{128};
};

struct TimebaseAsc {
  static constexpr size_t max_timebase_asc_actions{64};
};

struct Prioritor {
  static constexpr size_t max_prioritors{16};
  static constexpr size_t max_prioritor_groups{9};
};

template<tag_t, typename ValueT, index_t I = 0>
struct ControllerUnitVariable : Variable<ValueT, I> {
  using type = ControllerUnitVariableType;

  ControllerUnitVariable() = default;
  ControllerUnitVariable(ControllerUnitVariable &) = delete;
  ControllerUnitVariable(ControllerUnitVariable &&) = delete;

  ControllerUnitVariable &operator=(ControllerUnitVariable &) = delete;
  ControllerUnitVariable &operator=(ControllerUnitVariable &&) = delete;
};

template<typename T> requires is_cu_variable<T>
T variable{};
} // end of namespace atc::cu

namespace io {

struct IOVariableType {
};

template<typename T>
concept is_io_variable = std::is_same_v<typename T::type, IOVariableType>;

template<tag_t, typename ValueT, index_t I = 0>
struct IOVariable : Variable<ValueT, I> {
  using type = IOVariableType;

  IOVariable() = default;
  IOVariable(IOVariable &) = delete;
  IOVariable(IOVariable &&) = delete;
  IOVariable &operator=(IOVariable &) = delete;
  IOVariable &operator=(IOVariable &&) = delete;
};

template<typename T> requires is_io_variable<T>
T variable{};

namespace output {

using AltFlashState
= IOVariable<AUTO_TAG_ID, Bit>;

using AuxFunctionState
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I> requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelGreenWalkDriver
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelRedDoNotWalkDriver
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelYellowPedClearDriver
= IOVariable<AUTO_TAG_ID, Bit, I>;

using CustomAlarm
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I> requires (I >= 1)
using DetectorReset
= IOVariable<AUTO_TAG_ID, Bit, I>;

using FlashState
= IOVariable<AUTO_TAG_ID, Bit>;

using GlobalVariable
= IOVariable<AUTO_TAG_ID, Bit>;

using NotActive
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I> requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapGreen
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapProtectedGreen
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapRed
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapYellow
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PedCall
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseAdvWarning
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseCheck
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseDoNotWalk
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseGreen
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseNext
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseOn
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePedClearance
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePreClear
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePreClear2
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseRed
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseWalk
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseYellow
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptStatus
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptStatusFlash
= IOVariable<AUTO_TAG_ID, Bit, I>;

using RingStatusBitA
= IOVariable<AUTO_TAG_ID, Bit>;

using RingStatusBitB
= IOVariable<AUTO_TAG_ID, Bit>;

using RingStatusBitC
= IOVariable<AUTO_TAG_ID, Bit>;

using RingStatusBitD
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I> requires (IsValidIndex(I, cu::Unit::max_special_function_outputs))
using SpecialFunction
= IOVariable<AUTO_TAG_ID, Bit, I>;

using UnitAutomaticFlash
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitFaultMonitor
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitFreeCoordStatus
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_3
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTBCAux_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTBCAux_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTBCAux_3
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanA
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanB
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanC
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanD
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitVoltageMonitor
= IOVariable<AUTO_TAG_ID, Bit>;

using Watchdog
= IOVariable<AUTO_TAG_ID, Bit>;

} // end of namespace atc::io::output

namespace input {

template<index_t I> requires (IsValidIndex(I, cu::Detector::max_vehicle_detectors))
using ChannelFaultStatus
= IOVariable<AUTO_TAG_ID, Bit, I>;

using CoordFreeSwitch
= IOVariable<AUTO_TAG_ID, Bit>;

using CustomAlarm
= IOVariable<AUTO_TAG_ID, Bit>;

using DoorAjor
= IOVariable<AUTO_TAG_ID, Bit>;

using ManualControlGroupAction
= IOVariable<AUTO_TAG_ID, Bit>;

using MinGreen_2
= IOVariable<AUTO_TAG_ID, Bit>;

using NotActive
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I> requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Coord::max_patterns))
using PatternInput
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Detector::max_pedestrian_detectors))
using PedDetCall
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseForceOff
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseHold
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePedOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePhaseOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptGateDown
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptGateUp
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptHighPrioritorLow
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInput
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInputCRC
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInputNormalOff
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInputNormalOn
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Prioritor::max_prioritors))
using PrioritorCheckIn
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Prioritor::max_prioritors))
using PrioritorCheckOut
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (I >= 1)
using PrioritorPreemptDetector
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingForceOff
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingInhibitMaxTermination
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingMax2Selection
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingMax3Selection
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingOmitRedClearance
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingPedestrianRecycle
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingRedRest
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using RingStopTiming
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Ring::max_rings))
using SpecialFunctionInput
= IOVariable<AUTO_TAG_ID, Bit, I>;

using UnitAlarm_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitAlarm_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceA
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceB
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceC
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceD
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitAutomaticFlash
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitCallPedNAPlus
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitCallToNonActuated_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitCallToNonActuated_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitClockReset
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitCMUMMUFlashStatus
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitDimming
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitExternWatchDog
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitExternalMinRecall
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitExternalStart
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitIndicatorLampControl
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitIntervalAdvance
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_0
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_3
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitITSLocalFlashSense
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitLocalFlash
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitLocalFlashSense
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitManualControlEnable
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_3
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSignalPlanA
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSignalPlanB
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitStopTIme
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_0
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_1
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_2
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_3
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_4
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTBCHoldOnline
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTBCOnline
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTestInputA
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTestInputB
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTestInputC
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanA
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanB
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanC
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanD
= IOVariable<AUTO_TAG_ID, Bit>;

using UnitWalkRestModifier
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I> requires (IsValidIndex(I, cu::Detector::max_vehicle_detectors))
using VehicleDetCall
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (I >= 1)
using VehicleDetReset
= IOVariable<AUTO_TAG_ID, Bit, I>;

} // end of namespace atc::io::input

} // end of namespace atc::io

namespace mmu {

struct MMUVariableType {
};

template<typename T>
concept is_mmu_variable = std::is_same_v<typename T::type, MMUVariableType>;

template<tag_t, typename ValueT, index_t I = 0>
struct MMUVariable : Variable<ValueT, I> {
  using type = MMUVariableType;

  MMUVariable() = default;
  MMUVariable(MMUVariable &) = delete;
  MMUVariable(MMUVariable &&) = delete;
  MMUVariable &operator=(MMUVariable &) = delete;
  MMUVariable &operator=(MMUVariable &&) = delete;
};

template<index_t I> requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelGreenWalkDriver
= MMUVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelRedDoNotWalkDriver
= MMUVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I> requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelYellowPedClearDriver
= MMUVariable<AUTO_TAG_ID, Bit, I>;

using LoadSwitchFlash
= MMUVariable<AUTO_TAG_ID, Bit>;

template<typename T> requires is_mmu_variable<T>
T variable{};

} // end of namespace atc::mmu

template<typename T>
requires cu::is_cu_variable<T>
constexpr T &variable() { return cu::variable<T>; }

template<typename T>
requires mmu::is_mmu_variable<T>
constexpr T &variable() { return mmu::variable<T>; }

template<typename T>
requires io::is_io_variable<T>
constexpr T &variable() { return io::variable<T>; }

struct FrameElementType {
};

template<typename T>
concept is_valid_frame_elem = std::is_same_v<FrameElementType, typename T::type>;

template<typename T, size_t BitPos> requires std::is_same_v<ValueType<T>, Bit>
struct FrameBit {
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> a_data_in) {
    static auto l_bytepos = pos / 8;
    static auto l_nbits_to_shift = pos % 8;
    auto l_value = (a_data_in[l_bytepos] & (0x01 << l_nbits_to_shift)) != 0;
    ref_var.value = static_cast<Bit>(l_value);
  }

  void operator>>(const std::span<Byte> a_data_out) {
    static auto l_byte_pos = pos / 8;
    static auto l_num_of_bits_to_shift = pos % 8;
    Byte i = (ref_var.value = Bit::on) ? 1 : 0;
    a_data_out[l_byte_pos] = a_data_out[l_byte_pos] | (i << l_num_of_bits_to_shift);
  }

  size_t pos{BitPos};
  T &ref_var{variable<T>()};
};

template<typename T, size_t BytePos> requires std::is_same_v<ValueType<T>, Byte>
struct FrameByte {
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> a_data_in) {
    ref_var.value = a_data_in[pos];
  }

  void operator>>(const std::span<Byte> a_data_out) {
    a_data_out[pos] = ref_var.value;
  }

  size_t pos{BytePos};
  T &ref_var{variable<T>()};
};

template<typename T, size_t BytePos> requires std::is_same_v<ValueType<T>, Word>
struct FrameWord {
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> a_data_in) {
    ref_var.value = (a_data_in[pos] & 0x00FF) | (a_data_in[pos + 1] << 8); // LByte | HByte
  }

  void operator>>(const std::span<Byte> a_data_out) {
    a_data_out[pos] = ref_var.value & 0x00FF; // LByte
    a_data_out[pos + 1] = ref_var.value >> 8; // HByte
  }

  size_t pos{BytePos};
  T &ref_var{variable<T>()};
};

// Primary Station Generated Command Frame
struct PSG_CommandFrameType {
};

// Primary Station Received Response Frame
struct PSR_ResponseFrameType {
};

// Secondary Station Received Command Frame
struct SSR_CommandFrameType {
};

// Secondary Station Generated Response Frame
struct SSG_ResponseFrameType {
};

template<typename T>
concept is_valid_primary_station_frame
= std::is_same_v<T, PSG_CommandFrameType> || std::is_same_v<T, PSR_ResponseFrameType>;

template<typename T>
concept is_valid_secondary_station_frame
= std::is_same_v<T, SSR_CommandFrameType> || std::is_same_v<T, SSG_ResponseFrameType>;

template<typename T>
concept is_valid_frame
= is_valid_primary_station_frame<T> || is_valid_secondary_station_frame<T>;

template<typename T, typename ...Ts>
concept is_valid_frame_and_elem
= is_valid_frame<T> && (is_valid_frame_elem<Ts> &&...);

template<typename T>
concept is_generative_frame
= std::is_same_v<T, PSG_CommandFrameType> || std::is_same_v<T, SSG_ResponseFrameType>;

template<typename T>
concept is_receivable_frame
= std::is_same_v<T, PSR_ResponseFrameType> || std::is_same_v<T, SSR_CommandFrameType>;

template<Byte FrameID, size_t FrameByteSize, typename T, typename ...Ts> requires is_valid_frame_and_elem<T, Ts...>
class Frame {
public:
  using type = T;

  Frame() = default;
  Frame(Frame &) = delete;
  Frame(Frame &&) = delete;
  Frame &operator=(Frame &) = delete;
  Frame &operator=(Frame &&) = delete;

  template<typename = T>
  requires is_receivable_frame<T>
  void operator<<(const std::span<const Byte, FrameByteSize> a_data_in) {
    // [0]: address; [1]: SDLC control code, always 0x83; [2]: frame type id.
    // Last two byte of the SDLC payload is CRC; stripped already, not in the data passed in.
    assert(a_data_in[2] == FrameID);
    assign(a_data_in);
  }

  template<typename = T>
  requires is_generative_frame<T>
  void operator>>(std::span<const Byte, FrameByteSize> a_data_out) {
    std::fill(a_data_out, a_data_out + FrameByteSize, 0);
    a_data_out[0] = FrameID;
    generate(a_data_out);
  }

  static constexpr size_t frame_id{FrameID};
  static constexpr size_t frame_byte_size{FrameByteSize};
private:
  template<size_t I = 0>
  inline void assign(const std::span<const Byte, FrameByteSize> a_data_in) {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(m_frame_elements) << a_data_in;
      assign<I + 1>(a_data_in);
    }
  }

  template<size_t I = 0>
  inline void generate(const std::span<Byte, FrameByteSize> a_data_out) {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(m_frame_elements) >> a_data_out;
      generate < I + 1 > (a_data_out);
    }
  }

  std::tuple<Ts...> m_frame_elements;
};

namespace mmu {

/* MMU LoadSwitchDriverFrame (TYPE 0 Command Frame)
   For each channel, there are two bits for dimming purpose
   ----------------------------------------------------
   LS+ Bit   LS- Bit    Function
   ----------------------------------------------------
   0         0          OFF
   1         0          Dimmed by eliminating + halfwave
   0         1          Dimmed by eliminating - halfwave
   1         1          ON
   ----------------------------------------------------

   In the frame definition, we use the same MMU variable to
   back-up both the positive and negative bit, thus it will
   be having a state of either 00 (OFF) or 11 (ON), without
   considering the dimming.
 */
using LoadSwitchDriverFrame
= Frame<
    0x00, // FrameID
    16,   // Total Byte Size of the frame.
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 0 - Address, 0x10 for MMU
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 0x00 for Type 0 Command Frame
    // ----------------------------------------------
    // Byte 3 - Channel Green Driver
    //-----------------------------------------------
    FrameBit<ChannelGreenWalkDriver<0x01>, 0x18>,
    FrameBit<ChannelGreenWalkDriver<0x01>, 0x19>,
    FrameBit<ChannelGreenWalkDriver<0x02>, 0x1A>,
    FrameBit<ChannelGreenWalkDriver<0x02>, 0x1B>,
    FrameBit<ChannelGreenWalkDriver<0x03>, 0x1C>,
    FrameBit<ChannelGreenWalkDriver<0x03>, 0x1D>,
    FrameBit<ChannelGreenWalkDriver<0x04>, 0x1E>,
    FrameBit<ChannelGreenWalkDriver<0x04>, 0x1F>,
    // Byte 4
    FrameBit<ChannelGreenWalkDriver<0x05>, 0x20>,
    FrameBit<ChannelGreenWalkDriver<0x05>, 0x21>,
    FrameBit<ChannelGreenWalkDriver<0x06>, 0x22>,
    FrameBit<ChannelGreenWalkDriver<0x06>, 0x23>,
    FrameBit<ChannelGreenWalkDriver<0x07>, 0x24>,
    FrameBit<ChannelGreenWalkDriver<0x07>, 0x25>,
    FrameBit<ChannelGreenWalkDriver<0x08>, 0x26>,
    FrameBit<ChannelGreenWalkDriver<0x08>, 0x27>,
    // Byte 5
    FrameBit<ChannelGreenWalkDriver<0x09>, 0x28>,
    FrameBit<ChannelGreenWalkDriver<0x09>, 0x29>,
    FrameBit<ChannelGreenWalkDriver<0x0A>, 0x2A>,
    FrameBit<ChannelGreenWalkDriver<0x0A>, 0x2B>,
    FrameBit<ChannelGreenWalkDriver<0x0B>, 0x2C>,
    FrameBit<ChannelGreenWalkDriver<0x0B>, 0x2D>,
    FrameBit<ChannelGreenWalkDriver<0x0C>, 0x2E>,
    FrameBit<ChannelGreenWalkDriver<0x0C>, 0x2F>,
    // Byte 6
    FrameBit<ChannelGreenWalkDriver<0x0D>, 0x30>,
    FrameBit<ChannelGreenWalkDriver<0x0D>, 0x31>,
    FrameBit<ChannelGreenWalkDriver<0x0E>, 0x32>,
    FrameBit<ChannelGreenWalkDriver<0x0E>, 0x33>,
    FrameBit<ChannelGreenWalkDriver<0x0F>, 0x34>,
    FrameBit<ChannelGreenWalkDriver<0x0F>, 0x35>,
    FrameBit<ChannelGreenWalkDriver<0x10>, 0x36>,
    FrameBit<ChannelGreenWalkDriver<0x10>, 0x37>,
    // ----------------------------------------------
    // Byte 7 - Channel Yellow Driver
    // ----------------------------------------------
    FrameBit<ChannelYellowPedClearDriver<0x01>, 0x38>,
    FrameBit<ChannelYellowPedClearDriver<0x01>, 0x39>,
    FrameBit<ChannelYellowPedClearDriver<0x02>, 0x3A>,
    FrameBit<ChannelYellowPedClearDriver<0x02>, 0x3B>,
    FrameBit<ChannelYellowPedClearDriver<0x03>, 0x3C>,
    FrameBit<ChannelYellowPedClearDriver<0x03>, 0x3D>,
    FrameBit<ChannelYellowPedClearDriver<0x04>, 0x3E>,
    FrameBit<ChannelYellowPedClearDriver<0x04>, 0x3F>,
    // Byte 8
    FrameBit<ChannelYellowPedClearDriver<0x05>, 0x40>,
    FrameBit<ChannelYellowPedClearDriver<0x05>, 0x41>,
    FrameBit<ChannelYellowPedClearDriver<0x06>, 0x42>,
    FrameBit<ChannelYellowPedClearDriver<0x06>, 0x43>,
    FrameBit<ChannelYellowPedClearDriver<0x07>, 0x44>,
    FrameBit<ChannelYellowPedClearDriver<0x07>, 0x45>,
    FrameBit<ChannelYellowPedClearDriver<0x08>, 0x46>,
    FrameBit<ChannelYellowPedClearDriver<0x08>, 0x47>,
    // Byte 9
    FrameBit<ChannelYellowPedClearDriver<0x09>, 0x48>,
    FrameBit<ChannelYellowPedClearDriver<0x09>, 0x49>,
    FrameBit<ChannelYellowPedClearDriver<0x0A>, 0x4A>,
    FrameBit<ChannelYellowPedClearDriver<0x0A>, 0x4B>,
    FrameBit<ChannelYellowPedClearDriver<0x0B>, 0x4C>,
    FrameBit<ChannelYellowPedClearDriver<0x0B>, 0x4D>,
    FrameBit<ChannelYellowPedClearDriver<0x0C>, 0x4E>,
    FrameBit<ChannelYellowPedClearDriver<0x0C>, 0x4F>,
    // Byte 10
    FrameBit<ChannelYellowPedClearDriver<0x0D>, 0x50>,
    FrameBit<ChannelYellowPedClearDriver<0x0D>, 0x51>,
    FrameBit<ChannelYellowPedClearDriver<0x0E>, 0x52>,
    FrameBit<ChannelYellowPedClearDriver<0x0E>, 0x53>,
    FrameBit<ChannelYellowPedClearDriver<0x0F>, 0x54>,
    FrameBit<ChannelYellowPedClearDriver<0x0F>, 0x55>,
    FrameBit<ChannelYellowPedClearDriver<0x10>, 0x56>,
    FrameBit<ChannelYellowPedClearDriver<0x10>, 0x57>,
    // ----------------------------------------------
    // Byte 11 - Channel Red Driver
    // ----------------------------------------------
    FrameBit<ChannelRedDoNotWalkDriver<0x01>, 0x58>,
    FrameBit<ChannelRedDoNotWalkDriver<0x01>, 0x59>,
    FrameBit<ChannelRedDoNotWalkDriver<0x02>, 0x5A>,
    FrameBit<ChannelRedDoNotWalkDriver<0x02>, 0x5B>,
    FrameBit<ChannelRedDoNotWalkDriver<0x03>, 0x5C>,
    FrameBit<ChannelRedDoNotWalkDriver<0x03>, 0x5D>,
    FrameBit<ChannelRedDoNotWalkDriver<0x04>, 0x5E>,
    FrameBit<ChannelRedDoNotWalkDriver<0x04>, 0x5F>,
    // Byte 12
    FrameBit<ChannelRedDoNotWalkDriver<0x05>, 0x60>,
    FrameBit<ChannelRedDoNotWalkDriver<0x05>, 0x61>,
    FrameBit<ChannelRedDoNotWalkDriver<0x06>, 0x62>,
    FrameBit<ChannelRedDoNotWalkDriver<0x06>, 0x63>,
    FrameBit<ChannelRedDoNotWalkDriver<0x07>, 0x64>,
    FrameBit<ChannelRedDoNotWalkDriver<0x07>, 0x65>,
    FrameBit<ChannelRedDoNotWalkDriver<0x08>, 0x66>,
    FrameBit<ChannelRedDoNotWalkDriver<0x08>, 0x67>,
    // Byte 13
    FrameBit<ChannelRedDoNotWalkDriver<0x09>, 0x68>,
    FrameBit<ChannelRedDoNotWalkDriver<0x09>, 0x69>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0A>, 0x6A>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0A>, 0x6B>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0B>, 0x6C>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0B>, 0x6D>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0C>, 0x6E>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0C>, 0x6F>,
    // Byte 14
    FrameBit<ChannelRedDoNotWalkDriver<0x0D>, 0x70>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0D>, 0x71>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0E>, 0x72>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0E>, 0x73>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0F>, 0x74>,
    FrameBit<ChannelRedDoNotWalkDriver<0x0F>, 0x75>,
    FrameBit<ChannelRedDoNotWalkDriver<0x10>, 0x76>,
    FrameBit<ChannelRedDoNotWalkDriver<0x10>, 0x77>,
    // ----------------------------------------------
    // Byte 15 : Bit 0x78 ~ 0x7E are reserved bits.
    // ----------------------------------------------
    FrameBit<LoadSwitchFlash, 0x7F>
>;

using InputStatusRequestFrame
= Frame<
    0x01, // FrameID
    3,
    SSR_CommandFrameType
>;

using MMUProgrammingRequestFrame
= Frame<
    0x03, // FrameID
    3,
    SSR_CommandFrameType
>;

} // end of namespace atc::mmu

namespace biu {

} // end of namespace atc::biu


} // end of namespace atc

#endif