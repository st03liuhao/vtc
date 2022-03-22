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

#include <array>
#include <atomic>
#include <functional>
#include <span>

namespace vc {

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

using Index = uint16_t;
using Size = uint16_t;
using Tag = int;

template<Index _I, Size _N>
concept ValidIndex
= (_I >= 1) && (_I <= _N);

template<typename T>
concept ValidValueType
= std::is_same_v<T, Bit> || std::is_same_v<T, Byte> || std::is_same_v<T, Word> || std::is_same_v<T, Integer>;

template<typename T, Index _I> /* */
requires ValidValueType<T>
struct Variable
{
  using value_t = T;

  Variable() = default;
  Variable(Variable &) = delete;
  Variable(Variable &&) = delete;
  Variable &operator=(Variable &) = delete;
  Variable &operator=(Variable &&) = delete;

  static constexpr Index index{_I};
  std::atomic<T> value{};
};

template<typename T>
using ValueType = typename T::Variable::value_t;

namespace cu {

struct CuVariableType
{
};

template<typename T>
concept ValidCuVariable
= std::is_same_v<typename T::type, CuVariableType>;

template<Tag, typename ValueT, Index _I = 0>
struct CuVariable : Variable<ValueT, _I>
{
  using type = CuVariableType;

  CuVariable() = default;
  CuVariable(CuVariable &) = delete;
  CuVariable(CuVariable &&) = delete;
  CuVariable &operator=(CuVariable &) = delete;
  CuVariable &operator=(CuVariable &&) = delete;
};

namespace phase {

constexpr Size max_phases{40};
constexpr Size max_phase_groups{5};

};

namespace detector {

constexpr Size max_vehicle_detectors{128};
constexpr Size max_vehicle_detector_status_groups{40};
constexpr Size max_pedestrian_detectors{72};

};

namespace ring {

constexpr Size max_rings{16};
constexpr Size max_sequences{20};
constexpr Size max_ring_control_groups{2};

};

namespace channel {

constexpr Size max_channels{32};
constexpr Size max_Channel_status_groups{4};

};

namespace overlap {

constexpr Size max_overlaps{32};
constexpr Size max_overlap_status_groups{4};

};

namespace preempt {
constexpr Size max_preempts{40};
};

namespace unit {

constexpr Size max_alarm_groups{1};
constexpr Size max_special_function_outputs{16};

};

namespace coord {

constexpr Size max_patterns{128};
constexpr Size max_splits{128};

};

namespace timebase_asc {

constexpr Size max_timebase_asc_actions{64};

};

namespace prioritor {

constexpr Size max_prioritors{16};
constexpr Size max_prioritor_groups{9};

};

template<typename T>/* */
requires ValidCuVariable<T>
T variable{};

} // end of namespace vc::cu

namespace io {

struct IoVariableType
{
};

template<typename T>
concept ValidIoVariable
= std::is_same_v<typename T::type, IoVariableType>;

template<Tag, typename ValueT, Index _I = 0>
struct IoVariable : Variable<ValueT, _I>
{
  using type = IoVariableType;

  IoVariable() = default;
  IoVariable(IoVariable &) = delete;
  IoVariable(IoVariable &&) = delete;
  IoVariable &operator=(IoVariable &) = delete;
  IoVariable &operator=(IoVariable &&) = delete;
};

template<typename T>/* */
requires ValidIoVariable<T>
T variable{};

namespace output {

using AltFlashState
= IoVariable<AUTO_TAG_ID, Bit>;

using AuxFunctionState
= IoVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelGreenWalkDriver
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelRedDoNotWalkDriver
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelYellowPedClearDriver
= IoVariable<AUTO_TAG_ID, Bit, _I>;

using CustomAlarm
= IoVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires (_I >= 1)
using DetectorReset
= IoVariable<AUTO_TAG_ID, Bit, _I>;

using FlashState
= IoVariable<AUTO_TAG_ID, Bit>;

using GlobalVariable
= IoVariable<AUTO_TAG_ID, Bit>;

using NotActive
= IoVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires ValidIndex<_I, cu::overlap::max_overlaps>
using OverlapGreen
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::overlap::max_overlaps>
using OverlapProtectedGreen
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::overlap::max_overlaps>
using OverlapRed
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::overlap::max_overlaps>
using OverlapYellow
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PedCall
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseAdvWarning
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseCheck
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseDoNotWalk
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseGreen
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseNext
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseOmit
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseOn
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhasePedClearance
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhasePreClear
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhasePreClear2
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseRed
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseWalk
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseYellow
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptStatus
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptStatusFlash
= IoVariable<AUTO_TAG_ID, Bit, _I>;

using StatusBitA_Ring_1
= IoVariable<AUTO_TAG_ID, Bit>;

using StatusBitB_Ring_1
= IoVariable<AUTO_TAG_ID, Bit>;

using StatusBitC_Ring_1
= IoVariable<AUTO_TAG_ID, Bit>;

using StatusBitA_Ring_2
= IoVariable<AUTO_TAG_ID, Bit>;

using StatusBitB_Ring_2
= IoVariable<AUTO_TAG_ID, Bit>;

using StatusBitC_Ring_2
= IoVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires ValidIndex<_I, cu::unit::max_special_function_outputs>
using SpecialFunction
= IoVariable<AUTO_TAG_ID, Bit, _I>;

using UnitAutomaticFlash
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitFaultMonitor
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitFreeCoordStatus
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_3
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTBCAux_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTBCAux_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTBCAux_3
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanA
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanB
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanC
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanD
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitVoltageMonitor
= IoVariable<AUTO_TAG_ID, Bit>;

using Watchdog
= IoVariable<AUTO_TAG_ID, Bit>;

} // end of namespace vc::io::output

namespace input {

template<Index _I>/* */
requires ValidIndex<_I, cu::detector::max_vehicle_detectors>
using ChannelFaultStatus
= IoVariable<AUTO_TAG_ID, Bit, _I>;

using CoordFreeSwitch
= IoVariable<AUTO_TAG_ID, Bit>;

using CustomAlarm
= IoVariable<AUTO_TAG_ID, Bit>;

using DoorAjor
= IoVariable<AUTO_TAG_ID, Bit>;

using ManualControlGroupAction
= IoVariable<AUTO_TAG_ID, Bit>;

using MinGreen_2
= IoVariable<AUTO_TAG_ID, Bit>;

using NotActive
= IoVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires ValidIndex<_I, cu::overlap::max_overlaps>
using OverlapOmit
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::coord::max_patterns>
using PatternInput
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::detector::max_pedestrian_detectors>
using PedDetCall
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseForceOff
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhaseHold
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhasePedOmit
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::phase::max_phases>
using PhasePhaseOmit
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptGateDown
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptGateUp
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptHighPrioritorLow
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptInput
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptInputCRC
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptInputNormalOff
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::preempt::max_preempts>
using PreemptInputNormalOn
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::prioritor::max_prioritors>
using PrioritorCheckIn
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::prioritor::max_prioritors>
using PrioritorCheckOut
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires (_I >= 1)
using PrioritorPreemptDetector
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingForceOff
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingInhibitMaxTermination
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingMax2Selection
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingMax3Selection
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingOmitRedClearance
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingPedestrianRecycle
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingRedRest
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using RingStopTiming
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::ring::max_rings>
using SpecialFunctionInput
= IoVariable<AUTO_TAG_ID, Bit, _I>;

using UnitAlarm_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitAlarm_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceA
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceB
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceC
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitAlternateSequenceD
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitAutomaticFlash
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitCallPedNAPlus
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitCallToNonActuated_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitCallToNonActuated_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitClockReset
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitCMUMMUFlashStatus
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitDimming
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitExternWatchDog
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitExternalMinRecall
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitExternalStart
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitIndicatorLampControl
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitIntervalAdvance
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_0
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitIOModeBit_3
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitITSLocalFlashSense
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitLocalFlash
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitLocalFlashSense
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitManualControlEnable
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitOffset_3
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSignalPlanA
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSignalPlanB
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitStopTIme
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_0
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_1
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_2
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_3
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitSystemAddressBit_4
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTBCHoldOnline
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTBCOnline
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTestInputA
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTestInputB
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTestInputC
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanA
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanB
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanC
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitTimingPlanD
= IoVariable<AUTO_TAG_ID, Bit>;

using UnitWalkRestModifier
= IoVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires ValidIndex<_I, cu::detector::max_vehicle_detectors>
using VehicleDetCall
= IoVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::detector::max_vehicle_detectors>
using VehicleDetReset
= IoVariable<AUTO_TAG_ID, Bit, _I>;

} // end of namespace vc::io::input

namespace biu {

constexpr Size max_det_bius{8};
constexpr Size max_tf_bius{8};

} // end of namespace vc::io::biu

} // end of namespace vc::io

namespace mmu {

struct MmuVariableType
{
};

template<typename T>
concept ValidMmuVariable
= std::is_same_v<typename T::type, MmuVariableType>;

template<Tag, typename ValueT, Index _I = 0>
struct MmuVariable : Variable<ValueT, _I>
{
  using type = MmuVariableType;

  MmuVariable() = default;
  MmuVariable(MmuVariable &) = delete;
  MmuVariable(MmuVariable &&) = delete;
  MmuVariable &operator=(MmuVariable &) = delete;
  MmuVariable &operator=(MmuVariable &&) = delete;
};

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelGreenWalkStatus
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelRedDoNotWalkStatus
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelYellowPedClearStatus
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

using ControllerVoltMonitor
= MmuVariable<AUTO_TAG_ID, Bit>;

using _24VoltMonitor_I
= MmuVariable<AUTO_TAG_ID, Bit>;

using _24VoltMonitor_II
= MmuVariable<AUTO_TAG_ID, Bit>;

using _24VoltMonitorInhibit
= MmuVariable<AUTO_TAG_ID, Bit>;

using Reset
= MmuVariable<AUTO_TAG_ID, Bit>;

using RedEnable
= MmuVariable<AUTO_TAG_ID, Bit>;

using Conflict
= MmuVariable<AUTO_TAG_ID, Bit>;

using RedFailure
= MmuVariable<AUTO_TAG_ID, Bit>;

using DiagnosticFailure
= MmuVariable<AUTO_TAG_ID, Bit>;

using MinimumClearanceFailure
= MmuVariable<AUTO_TAG_ID, Bit>;

using Port1TimeoutFailure
= MmuVariable<AUTO_TAG_ID, Bit>;

using FailedAndOutputRelayTransferred
= MmuVariable<AUTO_TAG_ID, Bit>;

using FailedAndImmediateResponse
= MmuVariable<AUTO_TAG_ID, Bit>;

using LocalFlashStatus
= MmuVariable<AUTO_TAG_ID, Bit>;

using StartupFlashCall
= MmuVariable<AUTO_TAG_ID, Bit>;

using FYAFlashRateFailure
= MmuVariable<AUTO_TAG_ID, Bit>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using MinimumYellowChangeDisable
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

using MinimumFlashTimeBit_0
= MmuVariable<AUTO_TAG_ID, Bit>;

using MinimumFlashTimeBit_1
= MmuVariable<AUTO_TAG_ID, Bit>;

using MinimumFlashTimeBit_2
= MmuVariable<AUTO_TAG_ID, Bit>;

using MinimumFlashTimeBit_3
= MmuVariable<AUTO_TAG_ID, Bit>;

using _24VoltLatch
= MmuVariable<AUTO_TAG_ID, Bit>;

using CVMFaultMonitorLatch
= MmuVariable<AUTO_TAG_ID, Bit>;

/*  The two channel IDs are encoded as the index.
    index = left-hand-side channel as high-byte, right-hand-size channel as low-byte
 */
template<Byte Ix, Byte Iy>/* */
requires ValidIndex<Ix, cu::channel::max_channels> && ValidIndex<Iy, cu::channel::max_channels>
using ChannelCompatibilityStatus
= MmuVariable<AUTO_TAG_ID, Bit, (Ix << 8) | Iy>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelGreenWalkDriver
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelRedDoNotWalkDriver
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, cu::channel::max_channels>
using ChannelYellowPedClearDriver
= MmuVariable<AUTO_TAG_ID, Bit, _I>;

using LoadSwitchFlash
= MmuVariable<AUTO_TAG_ID, Bit>;

template<typename T>/* */
requires ValidMmuVariable<T>
T variable{};

} // end of namespace vc::mmu

namespace broadcast {

struct BroadcastVariableType
{
};

template<typename T>
concept ValidBroadcastVariable
= std::is_same_v<typename T::type, BroadcastVariableType>;

template<Tag, typename ValueT, Index _I = 0>
struct BroadcastVariable : Variable<ValueT, _I>
{
  using type = BroadcastVariableType;

  BroadcastVariable() = default;
  BroadcastVariable(BroadcastVariable &) = delete;
  BroadcastVariable(BroadcastVariable &&) = delete;
  BroadcastVariable &operator=(BroadcastVariable &) = delete;
  BroadcastVariable &operator=(BroadcastVariable &&) = delete;
};

using CuReportedMonth
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CuReportedDay
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CuReportedYear
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CuReportedHour
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CuReportedMinutes
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CuReportedSeconds
= BroadcastVariable<AUTO_TAG_ID, Byte>;

using CuReportedTenthsOfSeconds
= BroadcastVariable<AUTO_TAG_ID, Byte>;

template<Index _I>/* */
requires ValidIndex<_I, io::biu::max_tf_bius>
using CuReportedTFBIUPresence
= BroadcastVariable<AUTO_TAG_ID, Bit, _I>;

template<Index _I>/* */
requires ValidIndex<_I, io::biu::max_det_bius>
using CuReportedDETBIUPresence
= BroadcastVariable<AUTO_TAG_ID, Bit, _I>;

template<typename T>/* */
requires ValidBroadcastVariable<T>
T variable{};

} // end of namespace broadcast

template<typename T>
requires broadcast::ValidBroadcastVariable<T>
constexpr T &variable()
{
  return broadcast::variable<T>;
}

template<typename T>
requires cu::ValidCuVariable<T>
constexpr T &variable()
{
  return cu::variable<T>;
}

template<typename T>
requires mmu::ValidMmuVariable<T>
constexpr T &variable()
{
  return mmu::variable<T>;
}

template<typename T>
requires io::ValidIoVariable<T>
constexpr T &variable()
{
  return io::variable<T>;
}

namespace serial {

/* SDLC encoding using non-return-to-zero NRZ encoding, high = 1, low = 0.
 * Reserved bits will be set to low. Spare bits are vendor specific, but since
 * we don't deal with vendor specific features, spare bits will also be set to 0.
 * */
constexpr int max_sdlc_frame_bytesize = 64; // max byte size = 64 byte
constexpr int sdlc_clock_speed = 153600;    // clock frequency

namespace {

struct FrameElementType
{
};

template<typename T, Size _BitPos>/* */
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
    Byte i = (ref_var.value == Bit::On) ? 1 : 0;
    a_data_out[l_byte_pos] = a_data_out[l_byte_pos] | (i << l_num_of_bits_to_shift);
  }

  Size pos{_BitPos};
  T &ref_var{variable<T>()};
};

template<typename T, Size _BytePos>/* */
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

  Size pos{_BytePos};
  T &ref_var{variable<T>()};
};

template<typename T, Size _BytePos>/* _BytePos: position of Word value low byte */
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

  Size pos{_BytePos};
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
concept ValidPrimaryStationFrame
= std::is_same_v<T, PSG_CommandFrameType> || std::is_same_v<T, PSR_ResponseFrameType>;

template<typename T>
concept ValidSecondaryStationFrame
= std::is_same_v<T, SSR_CommandFrameType> || std::is_same_v<T, SSG_ResponseFrameType>;

template<Size _N>
concept ValidFrameByteSize
= (_N >= 1) && (_N <= max_sdlc_frame_bytesize);

template<typename T>
concept ValidFrameElement
= std::is_same_v<typename T::type, FrameElementType>;

template<Size _ByteSize, typename T, typename ...Ts>
concept ValidFrame
= (ValidPrimaryStationFrame<T> || ValidSecondaryStationFrame<T>) && (ValidFrameElement<Ts> &&...)
    && ValidFrameByteSize<_ByteSize>;

template<typename T>
concept GenerativeFrame
= std::is_same_v<T, PSG_CommandFrameType> || std::is_same_v<T, SSG_ResponseFrameType>;

template<typename T>
concept ReceivableFrame
= std::is_same_v<T, PSR_ResponseFrameType> || std::is_same_v<T, SSR_CommandFrameType>;

template<Byte _Address, Byte _FrameID, Size _FrameByteSize, typename T, typename ...Ts>/* */
requires ValidFrame<_FrameByteSize, T, Ts...>
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
  requires ReceivableFrame<T>
  void operator<<(const std::span<const Byte> a_data_in)
  {
    // [0]: Address; [1]: SDLC control code 0x83; [2]: FrameID.
    // Last 16 bits CCITT-CRC of the SDLC payload stripped away, not part of a_data_in.
    // assert(a_data_in[0] == address;
    // assert(a_data_in[1] == 0x83;
    // assert(a_data_in[2] == id);
    Assign(a_data_in);
  }

  template<typename = T>
  requires GenerativeFrame<T>
  void operator>>(std::span<Byte> a_data_out)
  {
    std::fill(a_data_out.begin(), a_data_out.end(), 0);
    a_data_out[0] = address;
    a_data_out[1] = 0x83;
    a_data_out[2] = id;
    Generate(a_data_out);
  }

  static constexpr Byte address{_Address};
  static constexpr Byte id{_FrameID};
  static constexpr Size bytesize{_FrameByteSize};
private:
  template<Size I = 0>
  inline void Assign(const std::span<const Byte> a_data_in)
  {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(m_frame_elements) << a_data_in;
      // @formatter:off
      Assign<I+1>(a_data_in);
      // @formatter:on
    }
  }

  template<Size I = 0>
  inline void Generate(std::span<Byte> a_data_out)
  {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(m_frame_elements) >> a_data_out;
      // @formatter:off
      Generate <I+1> (a_data_out);
      // @formatter:on
    }
  }

  std::tuple<Ts...> m_frame_elements;
};

} // end of namespace anonymous

template<Byte _FrameID>
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
    FrameBit<mmu::ControllerVoltMonitor, 0x48>,
    FrameBit<mmu::_24VoltMonitor_I, 0x49>,
    FrameBit<mmu::_24VoltMonitor_II, 0x4A>,
    FrameBit<mmu::_24VoltMonitorInhibit, 0x4B>,
    FrameBit<mmu::Reset, 0x4C>,
    FrameBit<mmu::RedEnable, 0x4D>,
    //  0x4E Reserved
    //  0x4F Reserved
    // ----------------------------------------------
    // Byte 10
    // ----------------------------------------------
    FrameBit<mmu::Conflict, 0x50>,
    FrameBit<mmu::RedFailure, 0x51>,
    //  0x52 Spare
    //  0x53 Spare
    //  0x54 Spare
    //  0x55 Spare
    //  0x56 Spare
    //  0x57 Spare
    // ----------------------------------------------
    // Byte 11
    // ----------------------------------------------
    FrameBit<mmu::DiagnosticFailure, 0x58>,
    FrameBit<mmu::MinimumClearanceFailure, 0x59>,
    FrameBit<mmu::Port1TimeoutFailure, 0x5A>,
    FrameBit<mmu::FailedAndOutputRelayTransferred, 0x5B>,
    FrameBit<mmu::FailedAndImmediateResponse, 0x5C>,
    //  0x5D Reserved
    FrameBit<mmu::LocalFlashStatus, 0x5E>,
    FrameBit<mmu::StartupFlashCall, 0x5F>,
    // ----------------------------------------------
    // Byte 12
    // ----------------------------------------------
    FrameBit<mmu::FYAFlashRateFailure, 0x60>
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
    FrameBit<mmu::_24VoltLatch, 0xA4>,
    FrameBit<mmu::CVMFaultMonitorLatch, 0xA5>
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
    0x09, // FrameID = 9
    12,   // 12 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 0 - Address, 0xFF for broadcast message
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 0x09 for Type 9 Command Frame
    // ----------------------------------------------
    // Byte 3~9: Mon/Day/Year/Hour/Min/Sec/TenthSec
    //-----------------------------------------------
    FrameByte<broadcast::CuReportedMonth, 3>,
    FrameByte<broadcast::CuReportedDay, 4>,
    FrameByte<broadcast::CuReportedYear, 5>,
    FrameByte<broadcast::CuReportedHour, 6>,
    FrameByte<broadcast::CuReportedMinutes, 7>,
    FrameByte<broadcast::CuReportedSeconds, 8>,
    FrameByte<broadcast::CuReportedTenthsOfSeconds, 9>,
    //-----------------------------------------------
    // Byte 10 - TF BIU # 1 ~ 8 Present State
    //-----------------------------------------------
    FrameBit<broadcast::CuReportedTFBIUPresence<0x01>, 0x50>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x02>, 0x51>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x03>, 0x52>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x04>, 0x53>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x05>, 0x54>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x06>, 0x55>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x07>, 0x56>,
    FrameBit<broadcast::CuReportedTFBIUPresence<0x08>, 0x57>,
    //-----------------------------------------------
    // Byte 11 - DET BIU # 1 ~ 8 Present State
    //-----------------------------------------------
    FrameBit<broadcast::CuReportedDETBIUPresence<0x01>, 0x58>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x02>, 0x59>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x03>, 0x5A>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x04>, 0x5B>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x05>, 0x5C>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x06>, 0x5D>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x07>, 0x5E>,
    FrameBit<broadcast::CuReportedDETBIUPresence<0x08>, 0x5F>
>;

template<>
struct FrameType<9>
{
  using type = DateTimeBroadcastFrame;
};

using IoRequestTfBiu_1
= Frame<
    0x00, // TF BIU#1 Address = 0
    0x0A, // FrameID = 10
    11,   // 11 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 3
    // ---------------------------------------------- 
    FrameBit<io::output::ChannelRedDoNotWalkDriver<1>, 0x18>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<1>, 0x19>,
    FrameBit<io::output::ChannelYellowPedClearDriver<1>, 0x1A>,
    FrameBit<io::output::ChannelYellowPedClearDriver<1>, 0x1B>,
    FrameBit<io::output::ChannelGreenWalkDriver<1>, 0x1C>,
    FrameBit<io::output::ChannelGreenWalkDriver<1>, 0x1D>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<2>, 0x1E>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<2>, 0x1F>,
    // ----------------------------------------------
    // Byte 4
    // ----------------------------------------------
    FrameBit<io::output::ChannelYellowPedClearDriver<2>, 0x20>,
    FrameBit<io::output::ChannelYellowPedClearDriver<2>, 0x21>,
    FrameBit<io::output::ChannelGreenWalkDriver<2>, 0x22>,
    FrameBit<io::output::ChannelGreenWalkDriver<2>, 0x23>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<3>, 0x24>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<3>, 0x25>,
    FrameBit<io::output::ChannelYellowPedClearDriver<3>, 0x26>,
    FrameBit<io::output::ChannelYellowPedClearDriver<3>, 0x27>,
    // ----------------------------------------------
    // Byte 5
    // ----------------------------------------------
    FrameBit<io::output::ChannelGreenWalkDriver<3>, 0x28>,
    FrameBit<io::output::ChannelGreenWalkDriver<3>, 0x29>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<4>, 0x2A>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<4>, 0x2B>,
    FrameBit<io::output::ChannelYellowPedClearDriver<4>, 0x2C>,
    FrameBit<io::output::ChannelYellowPedClearDriver<4>, 0x2D>,
    FrameBit<io::output::ChannelGreenWalkDriver<4>, 0x2E>,
    FrameBit<io::output::ChannelGreenWalkDriver<4>, 0x2F>,
    // ----------------------------------------------
    // Byte 6
    // ----------------------------------------------
    FrameBit<io::output::ChannelRedDoNotWalkDriver<5>, 0x30>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<5>, 0x31>,
    FrameBit<io::output::ChannelYellowPedClearDriver<5>, 0x32>,
    FrameBit<io::output::ChannelYellowPedClearDriver<5>, 0x33>,
    FrameBit<io::output::ChannelGreenWalkDriver<5>, 0x34>,
    FrameBit<io::output::ChannelGreenWalkDriver<5>, 0x35>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<6>, 0x36>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<6>, 0x37>,
    // ----------------------------------------------
    // Byte 7
    // ----------------------------------------------
    FrameBit<io::output::ChannelYellowPedClearDriver<6>, 0x38>,
    FrameBit<io::output::ChannelYellowPedClearDriver<6>, 0x39>,
    FrameBit<io::output::ChannelGreenWalkDriver<6>, 0x3A>,
    FrameBit<io::output::ChannelGreenWalkDriver<6>, 0x3B>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<7>, 0x3C>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<7>, 0x3D>,
    FrameBit<io::output::ChannelYellowPedClearDriver<7>, 0x3E>,
    FrameBit<io::output::ChannelYellowPedClearDriver<7>, 0x3F>,
    // ----------------------------------------------
    // Byte 8
    // ----------------------------------------------
    FrameBit<io::output::ChannelGreenWalkDriver<7>, 0x40>,
    FrameBit<io::output::ChannelGreenWalkDriver<7>, 0x41>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<8>, 0x42>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<8>, 0x43>,
    FrameBit<io::output::ChannelYellowPedClearDriver<8>, 0x44>,
    FrameBit<io::output::ChannelYellowPedClearDriver<8>, 0x45>,
    FrameBit<io::output::ChannelGreenWalkDriver<8>, 0x46>,
    FrameBit<io::output::ChannelGreenWalkDriver<8>, 0x47>,
    // ----------------------------------------------
    // Byte 9
    // ----------------------------------------------
    FrameBit<io::output::UnitTBCAux_1, 0x48>,
    FrameBit<io::output::UnitTBCAux_2, 0x49>,
    FrameBit<io::output::PreemptStatus<1>, 0x4A>,
    FrameBit<io::output::PreemptStatus<2>, 0x4B>
    // Bit 0x4C - 0x4F designated as inputs, should be driven to logic 0 all times.
    // ----------------------------------------------
    // Byte 10
    // ----------------------------------------------
    // Bit 0x50 - 0x56 designated as inputs, should be driven to logic 0 all times.
    // Bit 0x57 Reserved.
>;

template<>
struct FrameType<10>
{
  using type = IoRequestTfBiu_1;
};

using IoRequestTfBiu_2
= Frame<
    0x01, // TF BIU#2 Address = 1
    0x0B, // FrameID = 11
    11,   // 11 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 3
    // ---------------------------------------------- 
    FrameBit<io::output::ChannelRedDoNotWalkDriver<9>, 0x18>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<9>, 0x19>,
    FrameBit<io::output::ChannelYellowPedClearDriver<9>, 0x1A>,
    FrameBit<io::output::ChannelYellowPedClearDriver<9>, 0x1B>,
    FrameBit<io::output::ChannelGreenWalkDriver<9>, 0x1C>,
    FrameBit<io::output::ChannelGreenWalkDriver<9>, 0x1D>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<10>, 0x1E>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<10>, 0x1F>,
    // ----------------------------------------------
    // Byte 4
    // ----------------------------------------------
    FrameBit<io::output::ChannelYellowPedClearDriver<10>, 0x20>,
    FrameBit<io::output::ChannelYellowPedClearDriver<10>, 0x21>,
    FrameBit<io::output::ChannelGreenWalkDriver<10>, 0x22>,
    FrameBit<io::output::ChannelGreenWalkDriver<10>, 0x23>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<11>, 0x24>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<11>, 0x25>,
    FrameBit<io::output::ChannelYellowPedClearDriver<11>, 0x26>,
    FrameBit<io::output::ChannelYellowPedClearDriver<11>, 0x27>,
    // ----------------------------------------------
    // Byte 5
    // ----------------------------------------------
    FrameBit<io::output::ChannelGreenWalkDriver<11>, 0x28>,
    FrameBit<io::output::ChannelGreenWalkDriver<11>, 0x29>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<12>, 0x2A>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<12>, 0x2B>,
    FrameBit<io::output::ChannelYellowPedClearDriver<12>, 0x2C>,
    FrameBit<io::output::ChannelYellowPedClearDriver<12>, 0x2D>,
    FrameBit<io::output::ChannelGreenWalkDriver<12>, 0x2E>,
    FrameBit<io::output::ChannelGreenWalkDriver<12>, 0x2F>,
    // ----------------------------------------------
    // Byte 6
    // ----------------------------------------------
    FrameBit<io::output::ChannelRedDoNotWalkDriver<13>, 0x30>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<13>, 0x31>,
    FrameBit<io::output::ChannelYellowPedClearDriver<13>, 0x32>,
    FrameBit<io::output::ChannelYellowPedClearDriver<13>, 0x33>,
    FrameBit<io::output::ChannelGreenWalkDriver<13>, 0x34>,
    FrameBit<io::output::ChannelGreenWalkDriver<13>, 0x35>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<14>, 0x36>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<14>, 0x37>,
    // ----------------------------------------------
    // Byte 7
    // ----------------------------------------------
    FrameBit<io::output::ChannelYellowPedClearDriver<14>, 0x38>,
    FrameBit<io::output::ChannelYellowPedClearDriver<14>, 0x39>,
    FrameBit<io::output::ChannelGreenWalkDriver<14>, 0x3A>,
    FrameBit<io::output::ChannelGreenWalkDriver<14>, 0x3B>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<15>, 0x3C>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<15>, 0x3D>,
    FrameBit<io::output::ChannelYellowPedClearDriver<15>, 0x3E>,
    FrameBit<io::output::ChannelYellowPedClearDriver<15>, 0x3F>,
    // ----------------------------------------------
    // Byte 8
    // ----------------------------------------------
    FrameBit<io::output::ChannelGreenWalkDriver<15>, 0x40>,
    FrameBit<io::output::ChannelGreenWalkDriver<15>, 0x41>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<16>, 0x42>,
    FrameBit<io::output::ChannelRedDoNotWalkDriver<16>, 0x43>,
    FrameBit<io::output::ChannelYellowPedClearDriver<16>, 0x44>,
    FrameBit<io::output::ChannelYellowPedClearDriver<16>, 0x45>,
    FrameBit<io::output::ChannelGreenWalkDriver<16>, 0x46>,
    FrameBit<io::output::ChannelGreenWalkDriver<16>, 0x47>,
    // ----------------------------------------------
    // Byte 9
    // ----------------------------------------------
    FrameBit<io::output::UnitTBCAux_3, 0x48>,
    FrameBit<io::output::UnitFreeCoordStatus, 0x49>,
    FrameBit<io::output::PreemptStatus<3>, 0x4A>,
    FrameBit<io::output::PreemptStatus<4>, 0x4B>,
    FrameBit<io::output::PreemptStatus<5>, 0x4C>,
    FrameBit<io::output::PreemptStatus<6>, 0x4D>
    // Bit 0x4E - 0x4F designated as inputs, should be driven to logic 0 all times.
    // ----------------------------------------------
    // Byte 10
    // ----------------------------------------------
    // Bit 0x50 - 0x52 designated as inputs, should be driven to logic 0 all times.
    // Bit 0x53 - 0x56 Spare, vendor specific (logic 0 or 1).
    // Bit 0x57 - Reserved.
>;

template<>
struct FrameType<11>
{
  using type = IoRequestTfBiu_2;
};

using IoRequestTfBiu_3
= Frame<
    0x02, // TF BIU#1 Address = 2
    0x0C, // FrameID = 12
    8,    // 8 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 3
    // ----------------------------------------------
    FrameBit<io::output::UnitTimingPlanA, 0x18>,
    FrameBit<io::output::UnitTimingPlanB, 0x19>,
    FrameBit<io::output::UnitTimingPlanC, 0x1A>,
    FrameBit<io::output::UnitTimingPlanD, 0x1B>,
    FrameBit<io::output::UnitOffset_1, 0x1C>,
    FrameBit<io::output::UnitOffset_2, 0x1D>,
    FrameBit<io::output::UnitOffset_3, 0x1E>,
    FrameBit<io::output::UnitAutomaticFlash, 0x1F>,
    // ----------------------------------------------
    // Byte 4
    // ----------------------------------------------
    FrameBit<io::output::SpecialFunction<1>, 0x20>,
    FrameBit<io::output::SpecialFunction<2>, 0x21>,
    FrameBit<io::output::SpecialFunction<3>, 0x22>,
    FrameBit<io::output::SpecialFunction<4>, 0x23>,
    // 0x24 - Reserved.
    // 0x25 - Reserved.
    // 0x26 - Reserved.
    // 0x27 - Reserved.
    // ----------------------------------------------
    // Byte 5
    // ----------------------------------------------
    FrameBit<io::output::StatusBitA_Ring_1, 0x28>,
    FrameBit<io::output::StatusBitB_Ring_1, 0x29>,
    FrameBit<io::output::StatusBitC_Ring_1, 0x2A>,
    FrameBit<io::output::StatusBitA_Ring_2, 0x2B>,
    FrameBit<io::output::StatusBitB_Ring_2, 0x2C>,
    FrameBit<io::output::StatusBitC_Ring_2, 0x2D>
    // 0x2E - Designated Input
    // 0x2F - Designated Input
    // ----------------------------------------------
    // Byte 6
    // ----------------------------------------------
    // 0x30 - Designated Input
    // 0x31 - Designated Input
    // 0x32 - Designated Input
    // 0x33 - Designated Input
    // 0x34 - Designated Input
    // 0x35 - Designated Input
    // 0x36 - Designated Input
    // 0x37 - Designated Input
    // ----------------------------------------------
    // Byte 7
    // ----------------------------------------------
    // 0x38 - Designated Input
    // 0x39 - Designated Input
    // 0x3A - Designated Input
    // 0x3B - Designated Input
    // 0x3C - Designated Input
    // 0x3D - Designated Input
    // 0x3E - Designated Input
    // 0x3F - Designated Input
>;

template<>
struct FrameType<12>
{
  using type = IoRequestTfBiu_3;
};

using IoRequestTfBiu_4
= Frame<
    0x03, // TF BIU#1 Address = 3
    0x0D, // FrameID = 13
    8,    // 8 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 3
    // ----------------------------------------------
    FrameBit<io::output::PhaseOn<1>, 0x18>,
    FrameBit<io::output::PhaseOn<2>, 0x19>,
    FrameBit<io::output::PhaseOn<3>, 0x1A>,
    FrameBit<io::output::PhaseOn<4>, 0x1B>,
    FrameBit<io::output::PhaseOn<5>, 0x1C>,
    FrameBit<io::output::PhaseOn<6>, 0x1D>,
    FrameBit<io::output::PhaseOn<7>, 0x1E>,
    FrameBit<io::output::PhaseOn<8>, 0x1F>,
    // ----------------------------------------------
    // Byte 4
    // ----------------------------------------------
    FrameBit<io::output::PhaseNext<1>, 0x20>,
    FrameBit<io::output::PhaseNext<2>, 0x21>,
    FrameBit<io::output::PhaseNext<3>, 0x22>,
    FrameBit<io::output::PhaseNext<4>, 0x23>,
    FrameBit<io::output::PhaseNext<5>, 0x24>,
    FrameBit<io::output::PhaseNext<6>, 0x25>,
    FrameBit<io::output::PhaseNext<7>, 0x26>,
    // 0x27 - Reserved.
    // ----------------------------------------------
    // Byte 5
    // ----------------------------------------------
    FrameBit<io::output::PhaseNext<8>, 0x28>,
    FrameBit<io::output::PhaseCheck<1>, 0x29>,
    FrameBit<io::output::PhaseCheck<2>, 0x2A>,
    FrameBit<io::output::PhaseCheck<3>, 0x2B>,
    FrameBit<io::output::PhaseCheck<4>, 0x2C>,
    FrameBit<io::output::PhaseCheck<5>, 0x2D>,
    FrameBit<io::output::PhaseCheck<6>, 0x2E>,
    FrameBit<io::output::PhaseCheck<7>, 0x2F>,
    // ----------------------------------------------
    // Byte 6
    // ----------------------------------------------
    FrameBit<io::output::PhaseCheck<8>, 0x30>
    // 0x31 - Designated Input
    // 0x32 - Designated Input
    // 0x33 - Designated Input
    // 0x34 - Designated Input
    // 0x35 - Designated Input
    // 0x36 - Spare
    // 0x37 - Spare
    // ----------------------------------------------
    // Byte 7
    // ----------------------------------------------
    // 0x38 - Spare
    // 0x39 - Spare
    // 0x3A - Spare
    // 0x3B - Designated Input
    // 0x3C - Designated Input
    // 0x3D - Designated Input
    // 0x3E - Designated Input
    // 0x3F - Designated Input
>;

template<>
struct FrameType<13>
{
  using type = IoRequestTfBiu_4;
};

using OutputTransferFrame
= Frame<
    0xFF, // Broadcast address 255
    0x12, // FrameID = 18
    3,
    SSR_CommandFrameType
>;

template<>
struct FrameType<18>
{
  using type = OutputTransferFrame;
};

namespace {

template<Byte _CommandFrameID, Byte _ResponseFrameID>
using FrameMap = std::tuple<typename FrameType<_CommandFrameID>::type, typename FrameType<_ResponseFrameID>::type>;

template<typename ...Ts>
using FrameMapsType = std::tuple<Ts...>;

using FrameMaps = FrameMapsType<
    FrameMap<0, 128>,
    FrameMap<1, 129>,
    FrameMap<3, 131>
>;

FrameMaps frame_maps;
constexpr auto frame_maps_size = std::tuple_size_v<decltype(frame_maps)>;
std::array<Byte, max_sdlc_frame_bytesize> buffer = {0};

} // end of namespace anonymous

std::function<void(Byte a_frame_id)> OnCommandFrameReceipt{nullptr};

template<int _I = 0>
std::tuple<bool, std::span<Byte>> Dispatch(std::span<const Byte> a_data_in)
{
  if constexpr (_I < frame_maps_size) {
    auto &cmd_frame = std::get<0>(std::get<_I>(frame_maps));
    auto &res_frame = std::get<1>(std::get<_I>(frame_maps));

    if (cmd_frame.id == a_data_in[2]) {
      cmd_frame << a_data_in;

      if (OnCommandFrameReceipt) {
        OnCommandFrameReceipt(cmd_frame.id);
      }

      res_frame >> buffer; // Note - the buffer will be emptied at the beginning of inside >>().
      return {true, {buffer.data(), res_frame.bytesize}};
    } else {
      // @formatter:off
      return Dispatch<_I+1> (a_data_in);
      // @formatter:on
    }
  } else {
    return {false, buffer};
  }
}

} // end of namespace vc::serialframe

} // end of namespace atc

#endif