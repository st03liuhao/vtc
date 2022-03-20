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
#include <span>

namespace atc {

// A dirty trick to generate template specialization tag.
#define AUTO_TAG_ID __LINE__

enum class Bit : bool
{
  Off = false,
  On = true
};

using Byte = uint8_t;
using Integer = uint32_t;
using Word = uint16_t;

using index_t = uint16_t;
using size_t = uint16_t;
using tag_t = int;

constexpr bool IsValidIndex(index_t I, size_t N)
{
  return (I >= 1) && (I <= N);
};

template<typename T>
concept is_valid_variable_value_t
= std::is_same_v<T, Bit> || std::is_same_v<T, Byte> || std::is_same_v<T, Word> || std::is_same_v<T, Integer>;

template<typename ValueT, index_t I> /* */
requires is_valid_variable_value_t<ValueT>
struct Variable
{
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

namespace broadcast {

struct BroadcastVariableType
{
};

template<typename T>
concept is_broadcast_variable
= std::is_same_v<typename T::type, BroadcastVariableType>;

template<tag_t, typename ValueT, index_t I = 0>
struct BroadcastVariable : Variable<ValueT, I>
{
  using type = BroadcastVariableType;

  BroadcastVariable() = default;
  BroadcastVariable(BroadcastVariable &) = delete;
  BroadcastVariable(BroadcastVariable &&) = delete;
  BroadcastVariable &operator=(BroadcastVariable &) = delete;
  BroadcastVariable &operator=(BroadcastVariable &&) = delete;
};

using CUReportedMonth
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CUReportedDay
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CUReportedYear
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CUReportedHour
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CUReportedMinutes
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CUReportedSeconds
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CUReportedTenthsOfSeconds
= BroadcastVariable<AUTO_TAG_ID, Byte>;

template<index_t I>/* */
requires (IsValidIndex(I, 8))
using CUReportedTFBIUPresence
= BroadcastVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, 8))
using CUReportedDETBIUPresence
= BroadcastVariable<AUTO_TAG_ID, Bit, I>;

template<typename T>/* */
requires is_broadcast_variable<T>
T variable{};

} // end of namespace broadcast

namespace cu {

struct ControllerUnitVariableType
{
};

template<typename T>
concept is_cu_variable
= std::is_same_v<typename T::type, ControllerUnitVariableType>;

struct Phase
{
  static constexpr size_t max_Phases{40};
  static constexpr size_t max_Phase_groups{5};
};

struct Detector
{
  static constexpr size_t max_vehicle_detectors{128};
  static constexpr size_t max_vehicle_detector_status_groups{40};
  static constexpr size_t max_pedestrian_detectors{72};
};

struct Ring
{
  static constexpr size_t max_rings{16};
  static constexpr size_t max_sequences{20};
  static constexpr size_t max_ring_control_groups{2};
};

struct Channel
{
  static constexpr size_t max_Channels{32};
  static constexpr size_t max_Channel_status_groups{4};
};

struct Overlap
{
  static constexpr size_t max_Overlaps{32};
  static constexpr size_t max_Overlap_status_groups{4};
};

struct Preempt
{
  static constexpr size_t max_preempts{40};
};

struct Unit
{
  static constexpr size_t max_alarm_groups{1};
  static constexpr size_t max_special_function_outputs{16};
};

struct Coord
{
  static constexpr size_t max_patterns{128};
  static constexpr size_t max_splits{128};
};

struct TimebaseAsc
{
  static constexpr size_t max_timebase_asc_actions{64};
};

struct Prioritor
{
  static constexpr size_t max_prioritors{16};
  static constexpr size_t max_prioritor_groups{9};
};

template<tag_t, typename ValueT, index_t I = 0>
struct ControllerUnitVariable : Variable<ValueT, I>
{
  using type = ControllerUnitVariableType;

  ControllerUnitVariable() = default;
  ControllerUnitVariable(ControllerUnitVariable &) = delete;
  ControllerUnitVariable(ControllerUnitVariable &&) = delete;

  ControllerUnitVariable &operator=(ControllerUnitVariable &) = delete;
  ControllerUnitVariable &operator=(ControllerUnitVariable &&) = delete;
};

template<typename T>/* */
requires is_cu_variable<T>
T variable{};

} // end of namespace atc::cu

namespace io {

struct IOVariableType
{
};

template<typename T>
concept is_io_variable
= std::is_same_v<typename T::type, IOVariableType>;

template<tag_t, typename ValueT, index_t I = 0>
struct IOVariable : Variable<ValueT, I>
{
  using type = IOVariableType;

  IOVariable() = default;
  IOVariable(IOVariable &) = delete;
  IOVariable(IOVariable &&) = delete;
  IOVariable &operator=(IOVariable &) = delete;
  IOVariable &operator=(IOVariable &&) = delete;
};

template<typename T>/* */
requires is_io_variable<T>
T variable{};

namespace output {

using AltFlashState
= IOVariable<AUTO_TAG_ID, Bit>;

using AuxFunctionState
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelGreenWalkDriver
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelRedDoNotWalkDriver
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelYellowPedClearDriver
= IOVariable<AUTO_TAG_ID, Bit, I>;

using CustomAlarm
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I>/* */
requires (I >= 1)
using DetectorReset
= IOVariable<AUTO_TAG_ID, Bit, I>;

using FlashState
= IOVariable<AUTO_TAG_ID, Bit>;

using GlobalVariable
= IOVariable<AUTO_TAG_ID, Bit>;

using NotActive
= IOVariable<AUTO_TAG_ID, Bit>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapGreen
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapProtectedGreen
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapRed
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapYellow
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PedCall
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseAdvWarning
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseCheck
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseDoNotWalk
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseGreen
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseNext
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseOn
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePedClearance
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePreClear
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePreClear2
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseRed
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseWalk
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseYellow
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptStatus
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
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

template<index_t I>/* */
requires (IsValidIndex(I, cu::Unit::max_special_function_outputs))
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

template<index_t I>/* */
requires (IsValidIndex(I, cu::Detector::max_vehicle_detectors))
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

template<index_t I>/* */
requires (IsValidIndex(I, cu::Overlap::max_Overlaps))
using OverlapOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Coord::max_patterns))
using PatternInput
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Detector::max_pedestrian_detectors))
using PedDetCall
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseForceOff
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhaseHold
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePedOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Phase::max_Phases))
using PhasePhaseOmit
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptGateDown
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptGateUp
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptHighPrioritorLow
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInput
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInputCRC
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInputNormalOff
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Preempt::max_preempts))
using PreemptInputNormalOn
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Prioritor::max_prioritors))
using PrioritorCheckIn
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Prioritor::max_prioritors))
using PrioritorCheckOut
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (I >= 1)
using PrioritorPreemptDetector
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingForceOff
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingInhibitMaxTermination
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingMax2Selection
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingMax3Selection
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingOmitRedClearance
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingPedestrianRecycle
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingRedRest
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
using RingStopTiming
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Ring::max_rings))
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

template<index_t I>/* */
requires (IsValidIndex(I, cu::Detector::max_vehicle_detectors))
using VehicleDetCall
= IOVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (I >= 1)
using VehicleDetReset
= IOVariable<AUTO_TAG_ID, Bit, I>;

} // end of namespace atc::io::input

} // end of namespace atc::io

namespace mmu {

struct MMUVariableType
{
};

template<typename T>
concept is_mmu_variable
= std::is_same_v<typename T::type, MMUVariableType>;

template<tag_t, typename ValueT, index_t I = 0>
struct MMUVariable : Variable<ValueT, I>
{
  using type = MMUVariableType;

  MMUVariable() = default;
  MMUVariable(MMUVariable &) = delete;
  MMUVariable(MMUVariable &&) = delete;
  MMUVariable &operator=(MMUVariable &) = delete;
  MMUVariable &operator=(MMUVariable &&) = delete;
};

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelGreenWalkStatus
= MMUVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelRedDoNotWalkStatus
= MMUVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelYellowPedClearStatus
= MMUVariable<AUTO_TAG_ID, Bit, I>;

using ControllerVoltMonitor
= MMUVariable<AUTO_TAG_ID, Bit>;

using _24VoltMonitor_I
= MMUVariable<AUTO_TAG_ID, Bit>;

using _24VoltMonitor_II
= MMUVariable<AUTO_TAG_ID, Bit>;

using _24VoltMonitorInhibit
= MMUVariable<AUTO_TAG_ID, Bit>;

using Reset
= MMUVariable<AUTO_TAG_ID, Bit>;

using RedEnable
= MMUVariable<AUTO_TAG_ID, Bit>;

using Conflict
= MMUVariable<AUTO_TAG_ID, Bit>;

using RedFailure
= MMUVariable<AUTO_TAG_ID, Bit>;

using DiagnosticFailure
= MMUVariable<AUTO_TAG_ID, Bit>;

using MinimumClearanceFailure
= MMUVariable<AUTO_TAG_ID, Bit>;

using Port1TimeoutFailure
= MMUVariable<AUTO_TAG_ID, Bit>;

using FailedAndOutputRelayTransferred
= MMUVariable<AUTO_TAG_ID, Bit>;

using FailedAndImmediateResponse
= MMUVariable<AUTO_TAG_ID, Bit>;

using LocalFlashStatus
= MMUVariable<AUTO_TAG_ID, Bit>;

using StartupFlashCall
= MMUVariable<AUTO_TAG_ID, Bit>;

using FYAFlashRateFailure
= MMUVariable<AUTO_TAG_ID, Bit>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using MinimumYellowChangeDisable
= MMUVariable<AUTO_TAG_ID, Bit, I>;

using MinimumFlashTimeBit_0
= MMUVariable<AUTO_TAG_ID, Bit>;

using MinimumFlashTimeBit_1
= MMUVariable<AUTO_TAG_ID, Bit>;

using MinimumFlashTimeBit_2
= MMUVariable<AUTO_TAG_ID, Bit>;

using MinimumFlashTimeBit_3
= MMUVariable<AUTO_TAG_ID, Bit>;

using _24VoltLatch
= MMUVariable<AUTO_TAG_ID, Bit>;

using CVMFaultMonitorLatch
= MMUVariable<AUTO_TAG_ID, Bit>;

/*  The two channel IDs are encoded as the index.
    index = left-hand-side channel as high-byte, right-hand-size channel as low-byte
 */
template<Byte Ix, Byte Iy>/* */
requires (IsValidIndex(Ix, cu::Channel::max_Channels) && IsValidIndex(Iy, cu::Channel::max_Channels))
using ChannelCompatibilityStatus
= MMUVariable<AUTO_TAG_ID, Bit, (Ix << 8) | Iy>;

template<index_t I>/* */
requires (IsValidIndex(I, cu/* */::Channel::max_Channels))
using ChannelGreenWalkDriver
= MMUVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelRedDoNotWalkDriver
= MMUVariable<AUTO_TAG_ID, Bit, I>;

template<index_t I>/* */
requires (IsValidIndex(I, cu::Channel::max_Channels))
using ChannelYellowPedClearDriver
= MMUVariable<AUTO_TAG_ID, Bit, I>;

using LoadSwitchFlash
= MMUVariable<AUTO_TAG_ID, Bit>;

template<typename T>/* */
requires is_mmu_variable<T>
T variable{};

} // end of namespace atc::mmu

//-----------------------------------------------------
template<typename T>
requires broadcast::is_broadcast_variable<T>
constexpr T &variable()
{
  return broadcast::variable<T>;
}

template<typename T>
requires cu::is_cu_variable<T>
constexpr T &variable()
{
  return cu::variable<T>;
}

template<typename T>
requires mmu::is_mmu_variable<T>
constexpr T &variable()
{
  return mmu::variable<T>;
}

template<typename T>
requires io::is_io_variable<T>
constexpr T &variable()
{
  return io::variable<T>;
}
//---------------------------------------------------------

namespace serialframe {

struct FrameElementType
{
};

template<typename T>
concept is_valid_frame_elem
= std::is_same_v<FrameElementType, typename T::type>;

template<typename T, size_t BitPos>/* */
requires std::is_same_v<ValueType<T>, Bit>
struct FrameBit
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> a_data_in)
  {
    static auto l_bytepos = pos / 8;
    static auto l_nbits_to_shift = pos % 8;
    auto l_value = (a_data_in[l_bytepos] & (0x01 << l_nbits_to_shift)) != 0;
    ref_var.value = static_cast<Bit>(l_value);
  }

  void operator>>(const std::span<Byte> a_data_out)
  {
    static auto l_byte_pos = pos / 8;
    static auto l_num_of_bits_to_shift = pos % 8;
    Byte i = (ref_var.value = Bit::On) ? 1 : 0;
    a_data_out[l_byte_pos] = a_data_out[l_byte_pos] | (i << l_num_of_bits_to_shift);
  }

  size_t pos{BitPos};
  T &ref_var{variable<T>()};
};

template<typename T, size_t BytePos>/* */
requires std::is_same_v<ValueType<T>, Byte>
struct FrameByte
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> a_data_in)
  {
    ref_var.value = a_data_in[pos];
  }

  void operator>>(const std::span<Byte> a_data_out)
  {
    a_data_out[pos] = ref_var.value;
  }

  size_t pos{BytePos};
  T &ref_var{variable<T>()};
};

template<typename T, size_t BytePos>/* */
requires std::is_same_v<ValueType<T>, Word>
struct FrameWord
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> a_data_in)
  {
    ref_var.value = (a_data_in[pos] & 0x00FF) | (a_data_in[pos + 1] << 8); // LByte | HByte
  }

  void operator>>(const std::span<Byte> a_data_out)
  {
    a_data_out[pos] = ref_var.value & 0x00FF; // LByte
    a_data_out[pos + 1] = ref_var.value >> 8; // HByte
  }

  size_t pos{BytePos};
  T &ref_var{variable<T>()};
};

// Primary Station Generated Command Frame
struct PSG_CommandFrameType
{
};

// Primary Station Received Response Frame
struct PSR_ResponseFrameType
{
};

// Secondary Station Received Command Frame
struct SSR_CommandFrameType
{
};

// Secondary Station Generated Response Frame
struct SSG_ResponseFrameType
{
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

/* SDLC encoding using non-return-to-zero NRZ encoding, high = 1, low = 0.
 * Reserved bits will be set to low. Spare bits are vendor specific, but since
 * we don't deal with vendor specific features, spare bits will also be set to 0.
 * */
constexpr int max_sdlc_frame_bytesize = 64; // max byte size = 64 byte
constexpr int sdlc_clock_speed = 153600;    // clock frequency

template<Byte Address, Byte FrameID, size_t FrameByteSize, typename T, typename ...Ts>/* */
requires is_valid_frame_and_elem<T, Ts...>
class Frame
{
public:
  using type = T;

  Frame() = default;
  Frame(Frame &) = delete;
  Frame(Frame &&) = delete;
  Frame &operator=(Frame &) = delete;
  Frame &operator=(Frame &&) = delete;

  template<typename = T>
  requires is_receivable_frame<T>
  void operator<<(const std::span<const Byte, FrameByteSize> a_data_in)
  {
    // [0]: Address; [1]: SDLC control code 0x83; [2]: FrameID.
    // Last 16 bits CCITT-CRC of the SDLC payload stripped away, not part of a_data_in.
    // assert(a_data_in[0] == address;
    // assert(a_data_in[1] == 0x83;
    // assert(a_data_in[2] == id);
    assign(a_data_in);
  }

  template<typename = T>
  requires is_generative_frame<T>
  void operator>>(std::span<const Byte, FrameByteSize> a_data_out)
  {
    std::fill(a_data_out, a_data_out + FrameByteSize, 0);
    a_data_out[0] = address;
    a_data_out[1] = 0x83;
    a_data_out[2] = id;
    generate(a_data_out);
  }

  static constexpr size_t address{Address};
  static constexpr size_t id{FrameID};
  static constexpr size_t bytesize{FrameByteSize};
private:
  template<size_t I = 0>
  inline void assign(const std::span<const Byte, FrameByteSize> a_data_in)
  {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(m_frame_elements) << a_data_in;
      assign<I + 1>(a_data_in);
    }
  }

  template<size_t I = 0>
  inline void generate(const std::span<Byte, FrameByteSize> a_data_out)
  {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(m_frame_elements) >> a_data_out;
      generate < I + 1 > (a_data_out);
    }
  }

  std::tuple<Ts...> m_frame_elements;
};

template<Byte FrameID>
struct FrameType
{
};

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
   back-up both the positive and negative bits, thus having
   a state of either 00 (OFF) or 11 (ON), ignoring dimming,
   which is obsolete for modern traffic controllers.
 */
using LoadSwitchDriversFrame
= Frame<
    0x10, // MMU Address = 16
    0x00, // FrameID = 0
    16,   // Total Byte Size of the frame.
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 0 - Address, 0x10 for MMU
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 0x00 for Type 0 Command Frame
    // ----------------------------------------------
    // Byte 3 - Channel Green Driver
    //-----------------------------------------------
    FrameBit<mmu::ChannelGreenWalkDriver<0x01>, 0x18>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x01>, 0x19>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x02>, 0x1A>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x02>, 0x1B>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x03>, 0x1C>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x03>, 0x1D>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x04>, 0x1E>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x04>, 0x1F>,
    // Byte 4
    FrameBit<mmu::ChannelGreenWalkDriver<0x05>, 0x20>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x05>, 0x21>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x06>, 0x22>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x06>, 0x23>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x07>, 0x24>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x07>, 0x25>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x08>, 0x26>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x08>, 0x27>,
    // Byte 5
    FrameBit<mmu::ChannelGreenWalkDriver<0x09>, 0x28>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x09>, 0x29>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0A>, 0x2A>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0A>, 0x2B>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0B>, 0x2C>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0B>, 0x2D>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0C>, 0x2E>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0C>, 0x2F>,
    // Byte 6
    FrameBit<mmu::ChannelGreenWalkDriver<0x0D>, 0x30>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0D>, 0x31>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0E>, 0x32>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0E>, 0x33>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0F>, 0x34>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x0F>, 0x35>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x10>, 0x36>,
    FrameBit<mmu::ChannelGreenWalkDriver<0x10>, 0x37>,
    // ----------------------------------------------
    // Byte 7 - Channel Yellow Driver
    // ----------------------------------------------
    FrameBit<mmu::ChannelYellowPedClearDriver<0x01>, 0x38>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x01>, 0x39>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x02>, 0x3A>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x02>, 0x3B>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x03>, 0x3C>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x03>, 0x3D>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x04>, 0x3E>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x04>, 0x3F>,
    // Byte 8
    FrameBit<mmu::ChannelYellowPedClearDriver<0x05>, 0x40>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x05>, 0x41>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x06>, 0x42>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x06>, 0x43>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x07>, 0x44>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x07>, 0x45>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x08>, 0x46>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x08>, 0x47>,
    // Byte 9
    FrameBit<mmu::ChannelYellowPedClearDriver<0x09>, 0x48>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x09>, 0x49>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0A>, 0x4A>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0A>, 0x4B>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0B>, 0x4C>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0B>, 0x4D>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0C>, 0x4E>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0C>, 0x4F>,
    // Byte 10
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0D>, 0x50>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0D>, 0x51>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0E>, 0x52>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0E>, 0x53>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0F>, 0x54>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x0F>, 0x55>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x10>, 0x56>,
    FrameBit<mmu::ChannelYellowPedClearDriver<0x10>, 0x57>,
    // ----------------------------------------------
    // Byte 11 - Channel Red Driver
    // ----------------------------------------------
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x01>, 0x58>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x01>, 0x59>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x02>, 0x5A>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x02>, 0x5B>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x03>, 0x5C>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x03>, 0x5D>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x04>, 0x5E>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x04>, 0x5F>,
    // Byte 12
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x05>, 0x60>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x05>, 0x61>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x06>, 0x62>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x06>, 0x63>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x07>, 0x64>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x07>, 0x65>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x08>, 0x66>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x08>, 0x67>,
    // Byte 13
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x09>, 0x68>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x09>, 0x69>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0A>, 0x6A>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0A>, 0x6B>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0B>, 0x6C>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0B>, 0x6D>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0C>, 0x6E>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0C>, 0x6F>,
    // Byte 14
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0D>, 0x70>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0D>, 0x71>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0E>, 0x72>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0E>, 0x73>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0F>, 0x74>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x0F>, 0x75>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x10>, 0x76>,
    FrameBit<mmu::ChannelRedDoNotWalkDriver<0x10>, 0x77>,
    // ----------------------------------------------
    // Byte 15 : Bit 0x78 ~ 0x7E are reserved bits.
    // ----------------------------------------------
    FrameBit<mmu::LoadSwitchFlash, 0x7F>
>;

template<>
struct FrameType<0>
{
  using type = LoadSwitchDriversFrame;
};

using LoadSwitchDriversAckFrame
= Frame<
    0x10, // MMU Address = 16
    0x80, // FrameID = 128, Type 0 ACK
    3,
    SSG_ResponseFrameType
>;

template<>
struct FrameType<128>
{
  using type = LoadSwitchDriversAckFrame;
};

using MMUInputStatusRequestFrame
= Frame<
    0x10, // MMU Address = 16
    0x01, // FrameID = 1
    3,
    SSR_CommandFrameType
>;

template<>
struct FrameType<1>
{
  using type = MMUInputStatusRequestFrame;
};

using MMUInputStatusRequestAckFrame
= Frame<
    0x10, // MMU Address = 16
    0x81, // FrameID = 129
    13,
    SSG_ResponseFrameType,
    // ----------------------------------------------
    // Byte 0 - Address, 0x10 for MMU
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 0x81 for Type 129 Response Frame
    // ----------------------------------------------
    // Byte 3 - Channel Green Status 1 ~ 8
    //-----------------------------------------------
    FrameBit<mmu::ChannelGreenWalkStatus<0x01>, 0x18>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x02>, 0x19>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x03>, 0x1A>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x04>, 0x1B>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x05>, 0x1C>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x06>, 0x1D>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x07>, 0x1E>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x08>, 0x1F>,
    // ----------------------------------------------
    // Byte 4 - Channel Green Status 9 ~ 16
    // ----------------------------------------------
    FrameBit<mmu::ChannelGreenWalkStatus<0x09>, 0x20>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x0A>, 0x21>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x0B>, 0x22>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x0C>, 0x23>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x0D>, 0x24>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x0E>, 0x25>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x0E>, 0x26>,
    FrameBit<mmu::ChannelGreenWalkStatus<0x10>, 0x27>,
    // ----------------------------------------------
    // Byte 5 - Channel Yellow Status 1 ~ 8
    // ----------------------------------------------
    FrameBit<mmu::ChannelYellowPedClearStatus<0x01>, 0x28>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x02>, 0x29>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x03>, 0x2A>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x04>, 0x2B>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x05>, 0x2C>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x06>, 0x2D>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x07>, 0x2E>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x08>, 0x2F>,
    // ----------------------------------------------
    // Byte 6 - Channel Yellow Status 9 ~ 16
    // ----------------------------------------------
    FrameBit<mmu::ChannelYellowPedClearStatus<0x09>, 0x30>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x0A>, 0x31>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x0B>, 0x32>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x0C>, 0x33>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x0D>, 0x34>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x0E>, 0x35>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x0F>, 0x36>,
    FrameBit<mmu::ChannelYellowPedClearStatus<0x10>, 0x37>,
    // ----------------------------------------------
    // Byte 7 - Channel Red Status 1 ~ 8
    // ----------------------------------------------
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x01>, 0x38>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x02>, 0x39>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x03>, 0x3A>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x04>, 0x3B>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x05>, 0x3C>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x06>, 0x3D>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x07>, 0x3E>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x08>, 0x3F>,
    // ----------------------------------------------
    // Byte 8 - Channel Red Status 9 ~ 16
    // ----------------------------------------------
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x09>, 0x40>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x0A>, 0x41>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x0B>, 0x42>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x0C>, 0x43>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x0D>, 0x44>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x0E>, 0x45>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x0F>, 0x46>,
    FrameBit<mmu::ChannelRedDoNotWalkStatus<0x10>, 0x47>,
    // ----------------------------------------------
    // Byte 9
    // ----------------------------------------------
    FrameBit<
        mmu::ControllerVoltMonitor,
        0x48>,
    FrameBit<
        mmu::_24VoltMonitor_I,
        0x49>,
    FrameBit<
        mmu::_24VoltMonitor_II,
        0x4A>,
    FrameBit<
        mmu::_24VoltMonitorInhibit,
        0x4B>,
    FrameBit<
        mmu::Reset,
        0x4C>,
    FrameBit<
        mmu::RedEnable,
        0x4D>,
    //  0x4E Reserved
    //  0x4F Reserved
    // ----------------------------------------------
    // Byte 10
    // ----------------------------------------------
    FrameBit<
        mmu::Conflict,
        0x50>,
    FrameBit<
        mmu::RedFailure,
        0x51>,
    //  0x52 Spare
    //  0x53 Spare
    //  0x54 Spare
    //  0x55 Spare
    //  0x56 Spare
    //  0x57 Spare
    // ----------------------------------------------
    // Byte 11
    // ----------------------------------------------
    FrameBit<
        mmu::DiagnosticFailure,
        0x58>,
    FrameBit<
        mmu::MinimumClearanceFailure,
        0x59>,
    FrameBit<
        mmu::Port1TimeoutFailure,
        0x5A>,
    FrameBit<
        mmu::FailedAndOutputRelayTransferred,
        0x5B>,
    FrameBit<
        mmu::FailedAndImmediateResponse,
        0x5C>,
    //  0x5D Reserved
    FrameBit<
        mmu::LocalFlashStatus,
        0x5E>,
    FrameBit<
        mmu::StartupFlashCall,
        0x5F>,
    // ----------------------------------------------
    // Byte 12
    // ----------------------------------------------
    FrameBit<
        mmu::FYAFlashRateFailure,
        0x60>
    //  0x61 Reserved
    //  0x62 Reserved
    //  0x63 Reserved
    //  0x64 Reserved
    //  0x65 Reserved
    //  0x66 Reserved
    //  0x67 Reserved
>;

template<>
struct FrameType<129>
{
  using type = MMUInputStatusRequestAckFrame;
};

using MMUProgrammingRequestFrame
= Frame<
    0x10, // MMU Address = 16
    0x03, // FrameID = 3
    3,
    SSR_CommandFrameType
>;

template<>
struct FrameType<3>
{
  using type = MMUProgrammingRequestFrame;
};

using MMUProgrammingRequestAckFrame
= Frame<
    0x10, // MMU Address = 16
    0x83, // FrameID = 131
    23,
    SSG_ResponseFrameType,
    // ----------------------------------------------
    // Byte 0 - Address, 0x10 for MMU
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 0x83 for Type 131 Response Frame
    // ----------------------------------------------
    // Byte 3
    //-----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x02>, 0x18>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x03>, 0x19>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x04>, 0x1A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x05>, 0x1B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x06>, 0x1C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x07>, 0x1D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x08>, 0x1E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x09>, 0x1F>,
    // ----------------------------------------------
    // Byte 4
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x0A>, 0x20>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x0B>, 0x21>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x0C>, 0x22>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x0D>, 0x23>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x0E>, 0x24>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x0F>, 0x25>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x01, 0x10>, 0x26>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x03>, 0x27>,
    // ----------------------------------------------
    // Byte 5
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x04>, 0x28>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x05>, 0x29>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x06>, 0x2A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x07>, 0x2B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x08>, 0x2C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x09>, 0x2D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x0A>, 0x2E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x0B>, 0x2F>,
    // ----------------------------------------------
    // Byte 6
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x0C>, 0x30>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x0D>, 0x31>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x0E>, 0x32>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x0F>, 0x33>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x02, 0x10>, 0x34>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x04>, 0x35>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x05>, 0x36>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x06>, 0x37>,
    // ----------------------------------------------
    // Byte 7
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x07>, 0x38>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x08>, 0x39>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x09>, 0x3A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x0A>, 0x3B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x0B>, 0x3C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x0C>, 0x3D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x0D>, 0x3E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x0E>, 0x3F>,
    // ----------------------------------------------
    // Byte 8
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x0F>, 0x40>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x03, 0x10>, 0x41>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x05>, 0x42>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x06>, 0x43>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x07>, 0x44>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x08>, 0x45>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x09>, 0x46>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x0A>, 0x47>,
    // ----------------------------------------------
    // Byte 9
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x0B>, 0x48>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x0C>, 0x49>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x0D>, 0x4A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x0E>, 0x4B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x0F>, 0x4C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x04, 0x10>, 0x4D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x06>, 0x4E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x07>, 0x4F>,
    // ----------------------------------------------
    // Byte 10
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x08>, 0x50>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x09>, 0x51>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x0A>, 0x52>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x0B>, 0x53>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x0C>, 0x54>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x0D>, 0x55>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x0E>, 0x56>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x0F>, 0x57>,
    // ----------------------------------------------
    // Byte 11
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x05, 0x10>, 0x58>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x07>, 0x59>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x08>, 0x5A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x09>, 0x5B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x0A>, 0x5C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x0B>, 0x5D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x0C>, 0x5E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x0D>, 0x5F>,
    // ----------------------------------------------
    // Byte 12
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x0E>, 0x60>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x0F>, 0x61>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x06, 0x10>, 0x62>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x08>, 0x63>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x09>, 0x64>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x0A>, 0x65>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x0B>, 0x66>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x0C>, 0x67>,
    // ----------------------------------------------
    // Byte 13
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x0D>, 0x68>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x0E>, 0x69>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x0F>, 0x6A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x07, 0x10>, 0x6B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x09>, 0x6C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x0A>, 0x6D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x0B>, 0x6E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x0C>, 0x6F>,
    // ----------------------------------------------
    // Byte 14
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x0D>, 0x70>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x0E>, 0x71>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x0F>, 0x72>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x08, 0x10>, 0x73>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x0A>, 0x74>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x0B>, 0x75>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x0C>, 0x76>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x0D>, 0x77>,
    // ----------------------------------------------
    // Byte 15
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x0E>, 0x78>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x0F>, 0x79>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x09, 0x10>, 0x7A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0A, 0x0B>, 0x7B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0A, 0x0C>, 0x7C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0A, 0x0D>, 0x7D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0A, 0x0E>, 0x7E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0A, 0x0F>, 0x7F>,
    // ----------------------------------------------
    // Byte 16
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x0A, 0x10>, 0x80>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0B, 0x0C>, 0x81>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0B, 0x0D>, 0x82>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0B, 0x0E>, 0x83>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0B, 0x0F>, 0x84>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0B, 0x10>, 0x85>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0C, 0x0D>, 0x86>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0C, 0x0E>, 0x87>,
    // ----------------------------------------------
    // Byte 17
    // ----------------------------------------------
    FrameBit<mmu::ChannelCompatibilityStatus<0x0C, 0x0F>, 0x88>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0C, 0x10>, 0x89>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0D, 0x0E>, 0x8A>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0D, 0x0F>, 0x8B>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0D, 0x10>, 0x8C>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0E, 0x0F>, 0x8D>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0E, 0x10>, 0x8E>,
    FrameBit<mmu::ChannelCompatibilityStatus<0x0F, 0x10>, 0x8F>,
    // ----------------------------------------------
    // Byte 18
    // ----------------------------------------------
    FrameBit<mmu::MinimumYellowChangeDisable<0x01>, 0x90>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x02>, 0x91>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x03>, 0x92>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x04>, 0x93>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x05>, 0x94>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x06>, 0x95>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x07>, 0x96>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x08>, 0x97>,
    // ----------------------------------------------
    // Byte 19
    // ----------------------------------------------
    FrameBit<mmu::MinimumYellowChangeDisable<0x09>, 0x98>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x0A>, 0x99>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x0B>, 0x9A>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x0C>, 0x9B>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x0D>, 0x9C>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x0E>, 0x9D>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x0F>, 0x9E>,
    FrameBit<mmu::MinimumYellowChangeDisable<0x10>, 0x9F>,
    // ----------------------------------------------
    // Byte 20
    // ----------------------------------------------
    FrameBit<mmu::MinimumFlashTimeBit_0, 0xA0>,
    FrameBit<mmu::MinimumFlashTimeBit_1, 0xA1>,
    FrameBit<mmu::MinimumFlashTimeBit_2, 0xA2>,
    FrameBit<mmu::MinimumFlashTimeBit_3, 0xA3>,

    FrameBit<
        mmu::_24VoltLatch,
        0xA4>,
    FrameBit<
        mmu::CVMFaultMonitorLatch,
        0xA5>
    // 0xA6 - Reserved
    // 0xA7 - Reserved
    // ----------------------------------------------
    // Byte 21 - Reserved
    // ----------------------------------------------
    // ----------------------------------------------
    // Byte 22 - Reserved
    // ----------------------------------------------      
>;

template<>
struct FrameType<131>
{
  using type = MMUProgrammingRequestAckFrame;
};

using DateTimeBroadcastFrame
= Frame<
    0xFF, // Broadcast Address = 255
    0x09, // FrameID
    12,   // 12 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 0 - Address, 0xFF for broadcast message
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 0x09 for Type 9 Command Frame
    // ----------------------------------------------

    //-----------------------------------------------
    // Byte 3~9: Mon/Day/Year/Hour/Min/Sec/TenthSec
    //-----------------------------------------------
    FrameByte<
        broadcast::CUReportedMonth,                  // 1 ~ 12
        3>,
    FrameByte<
        broadcast::CUReportedDay,                    // 1 ~ 31
        4>,
    FrameByte<
        broadcast::CUReportedYear,                   // 0 ~ 99
        5>,
    FrameByte<
        broadcast::CUReportedHour,                   // 0 ~ 23
        6>,
    FrameByte<
        broadcast::CUReportedMinutes,                // 0 ~ 59
        7>,
    FrameByte<
        broadcast::CUReportedSeconds,                // 0 ~ 59
        8>,
    FrameByte<
        broadcast::CUReportedTenthsOfSeconds,        // 0 ~ 9
        9>,

    //-----------------------------------------------
    // Byte 10 - TF BIU # 1 ~ 8 Present State
    //-----------------------------------------------
    FrameBit<broadcast::CUReportedTFBIUPresence<0x01>, 0x50>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x02>, 0x51>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x03>, 0x52>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x04>, 0x53>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x05>, 0x54>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x06>, 0x55>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x07>, 0x56>,
    FrameBit<broadcast::CUReportedTFBIUPresence<0x08>, 0x57>,
    //-----------------------------------------------
    // Byte 11 - DET BIU # 1 ~ 8 Present State
    //-----------------------------------------------
    FrameBit<broadcast::CUReportedDETBIUPresence<0x01>, 0x58>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x02>, 0x59>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x03>, 0x5A>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x04>, 0x5B>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x05>, 0x5C>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x06>, 0x5D>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x07>, 0x5E>,
    FrameBit<broadcast::CUReportedDETBIUPresence<0x08>, 0x5F>
>;

template<>
struct FrameType<9>
{
  using type = DateTimeBroadcastFrame;
};

} // end of namespace atc::serialframe


} // end of namespace atc

#endif