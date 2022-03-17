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
#include <type_traits>
#include <span>

namespace atc {

// A dirty trick to generate template specialization tag.
#define TAG_ID __LINE__

    enum class Bit : bool
    {
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
    concept is_valid_variable_value_t = std::is_same_v<T, Bit>
                                        || std::is_same_v<T, Byte>
                                        || std::is_same_v<T, Word>
                                        || std::is_same_v<T, Integer>;

    template<typename ValueT, index_t I> requires is_valid_variable_value_t<ValueT>
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
    concept is_variable = requires {
        T::Variable::value_t;
    };

    template<typename T>
    using VariableValueType = typename T::Variable::value_t;

    namespace cu {
        namespace phase {
            constexpr size_t max_phases{40};
            constexpr size_t max_phase_groups{5};
        }

        namespace detector {
            constexpr size_t max_vehicle_detectors{128};
            constexpr size_t max_vehicle_detector_status_groups{40};
            constexpr size_t max_pedestrian_detectors{72};
        }

        namespace ring {
            constexpr size_t max_rings{16};
            constexpr size_t max_sequences{20};
            constexpr size_t max_ring_control_groups{2};

        }

        namespace channel {
            constexpr size_t max_channels{32};
            constexpr size_t max_channel_status_groups{4};
        }

        namespace overlap {
            constexpr size_t max_overlaps{32};
            constexpr size_t max_overlap_status_groups{4};
        }

        namespace preempt {
            constexpr size_t max_preempts{40};
        }

        namespace unit {
            constexpr size_t max_alarm_groups{1};
            constexpr size_t max_special_function_outputs{16};
        }

        namespace coord {
            constexpr size_t max_patterns{128};
            constexpr size_t max_splits{128};
        }

        namespace timebase_asc {
            constexpr size_t max_timebase_asc_actions{64};
        }

        namespace prioritor {
            constexpr size_t max_prioritors{16};
            constexpr size_t max_prioritor_groups{9};
        }

        struct ControllerUnitVariableType
        {
        };

        template<typename T>
        concept is_cu_variable = requires {
            typename T::Variable::value_t;
            std::is_same_v<typename T::type, ControllerUnitVariableType>;
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

        template<typename T> requires is_cu_variable<T>
        T variable{};
    } // end of namespace cu

    namespace io {

        struct IOVariableType
        {
        };

        template<typename T>
        concept is_io_variable = requires {
            typename T::Variable::value_t;
            std::is_same_v<typename T::type, IOVariableType>;
        };

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

        namespace output {

            using AltFlashState = IOVariable<TAG_ID, Bit>;

            using AuxFunctionState = IOVariable<TAG_ID, Bit>;

            template<index_t I> requires (IsValidIndex(I, cu::channel::max_channels))
            using ChannelGreenWalkDriver = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::channel::max_channels))
            using ChannelRedDoNotWalkDriver = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::channel::max_channels))
            using ChannelYellowRedClearDriver = IOVariable<TAG_ID, Bit, I>;

            using CustomAlarm = IOVariable<TAG_ID, Bit>;

            template<index_t I> requires (I >= 1)
            using DetectorReset = IOVariable<TAG_ID, Bit, I>;

            using FlashState = IOVariable<TAG_ID, Bit>;

            using GlobalVariable = IOVariable<TAG_ID, Bit>;

            using NotActive = IOVariable<TAG_ID, Bit>;

            template<index_t I> requires (IsValidIndex(I, cu::overlap::max_overlaps))
            using OverlapGreen = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::overlap::max_overlaps))
            using OverlapProtectedGreen = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::overlap::max_overlaps))
            using OverlapRed = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::overlap::max_overlaps))
            using OverlapYellow = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PedCall = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseAdvWarning = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseCheck = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseDoNotWalk = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseGreen = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseNext = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseOmit = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseOn = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhasePedClearance = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhasePreClear = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhasePreClear2 = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseRed = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseWalk = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseYellow = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptStatus = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptStatusFlash = IOVariable<TAG_ID, Bit, I>;

            using RingStatusBitA = IOVariable<TAG_ID, Bit>;

            using RingStatusBitB = IOVariable<TAG_ID, Bit>;

            using RingStatusBitC = IOVariable<TAG_ID, Bit>;

            using RingStatusBitD = IOVariable<TAG_ID, Bit>;

            template<index_t I> requires (IsValidIndex(I, cu::unit::max_special_function_outputs))
            using SpecialFunction = IOVariable<TAG_ID, Bit, I>;

            using UnitAutomaticFlash = IOVariable<TAG_ID, Bit>;

            using UnitFaultMonitor = IOVariable<TAG_ID, Bit>;

            using UnitFreeCoordStatus = IOVariable<TAG_ID, Bit>;

            using UnitOffset_1 = IOVariable<TAG_ID, Bit>;

            using UnitOffset_2 = IOVariable<TAG_ID, Bit>;

            using UnitOffset_3 = IOVariable<TAG_ID, Bit>;

            using UnitTBCAux_1 = IOVariable<TAG_ID, Bit>;

            using UnitTBCAux_2 = IOVariable<TAG_ID, Bit>;

            using UnitTBCAux_3 = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanA = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanB = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanC = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanD = IOVariable<TAG_ID, Bit>;

            using UnitVoltageMonitor = IOVariable<TAG_ID, Bit>;

            using Watchdog = IOVariable<TAG_ID, Bit>;

        } // end of namespace output

        namespace input {

            template<index_t I> requires (IsValidIndex(I, cu::detector::max_vehicle_detectors))
            using ChannelFaultStatus = IOVariable<TAG_ID, Bit, I>;

            using CoordFreeSwitch = IOVariable<TAG_ID, Bit>;

            using CustomAlarm = IOVariable<TAG_ID, Bit>;

            using DoorAjor = IOVariable<TAG_ID, Bit>;

            using ManualControlGroupAction = IOVariable<TAG_ID, Bit>;

            using MinGreen_2 = IOVariable<TAG_ID, Bit>;

            using NotActive = IOVariable<TAG_ID, Bit>;

            template<index_t I> requires (IsValidIndex(I, cu::overlap::max_overlaps))
            using OverlapOmit = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::coord::max_patterns))
            using PatternInput = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::detector::max_pedestrian_detectors))
            using PedDetCall = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseForceOff = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhaseHold = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhasePedOmit = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::phase::max_phases))
            using PhasePhaseOmit = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptGateDown = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptGateUp = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptHighPrioritorLow = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptInput = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptInputCRC = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptInputNormalOff = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::preempt::max_preempts))
            using PreemptInputNormalOn = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::prioritor::max_prioritors))
            using PrioritorCheckIn = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::prioritor::max_prioritors))
            using PrioritorCheckOut = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (I >= 1)
            using PrioritorPreemptDetector = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingForceOff = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingInhibitMaxTermination = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingMax2Selection = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingMax3Selection = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingOmitRedClearance = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingPedestrianRecycle = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingRedRest = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using RingStopTiming = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (IsValidIndex(I, cu::ring::max_rings))
            using SpecialFunctionInput = IOVariable<TAG_ID, Bit, I>;

            using UnitAlarm_1 = IOVariable<TAG_ID, Bit>;

            using UnitAlarm_2 = IOVariable<TAG_ID, Bit>;

            using UnitAlternateSequenceA = IOVariable<TAG_ID, Bit>;

            using UnitAlternateSequenceB = IOVariable<TAG_ID, Bit>;

            using UnitAlternateSequenceC = IOVariable<TAG_ID, Bit>;

            using UnitAlternateSequenceD = IOVariable<TAG_ID, Bit>;

            using UnitAutomaticFlash = IOVariable<TAG_ID, Bit>;

            using UnitCallPedNAPlus = IOVariable<TAG_ID, Bit>;

            using UnitCallToNonActuated_1 = IOVariable<TAG_ID, Bit>;

            using UnitCallToNonActuated_2 = IOVariable<TAG_ID, Bit>;

            using UnitClockReset = IOVariable<TAG_ID, Bit>;

            using UnitCMUMMUFlashStatus = IOVariable<TAG_ID, Bit>;

            using UnitDimming = IOVariable<TAG_ID, Bit>;

            using UnitExternWatchDog = IOVariable<TAG_ID, Bit>;

            using UnitExternalMinRecall = IOVariable<TAG_ID, Bit>;

            using UnitExternalStart = IOVariable<TAG_ID, Bit>;

            using UnitIndicatorLampControl = IOVariable<TAG_ID, Bit>;

            using UnitIntervalAdvance = IOVariable<TAG_ID, Bit>;

            using UnitIOModeBit_0 = IOVariable<TAG_ID, Bit>;

            using UnitIOModeBit_1 = IOVariable<TAG_ID, Bit>;

            using UnitIOModeBit_2 = IOVariable<TAG_ID, Bit>;

            using UnitIOModeBit_3 = IOVariable<TAG_ID, Bit>;

            using UnitITSLocalFlashSense = IOVariable<TAG_ID, Bit>;

            using UnitLocalFlash = IOVariable<TAG_ID, Bit>;

            using UnitLocalFlashSense = IOVariable<TAG_ID, Bit>;

            using UnitManualControlEnable = IOVariable<TAG_ID, Bit>;

            using UnitOffset_1 = IOVariable<TAG_ID, Bit>;

            using UnitOffset_2 = IOVariable<TAG_ID, Bit>;

            using UnitOffset_3 = IOVariable<TAG_ID, Bit>;

            using UnitSignalPlanA = IOVariable<TAG_ID, Bit>;

            using UnitSignalPlanB = IOVariable<TAG_ID, Bit>;

            using UnitStopTIme = IOVariable<TAG_ID, Bit>;

            using UnitSystemAddressBit_0 = IOVariable<TAG_ID, Bit>;

            using UnitSystemAddressBit_1 = IOVariable<TAG_ID, Bit>;

            using UnitSystemAddressBit_2 = IOVariable<TAG_ID, Bit>;

            using UnitSystemAddressBit_3 = IOVariable<TAG_ID, Bit>;

            using UnitSystemAddressBit_4 = IOVariable<TAG_ID, Bit>;

            using UnitTBCHoldOnline = IOVariable<TAG_ID, Bit>;

            using UnitTBCOnline = IOVariable<TAG_ID, Bit>;

            using UnitTestInputA = IOVariable<TAG_ID, Bit>;

            using UnitTestInputB = IOVariable<TAG_ID, Bit>;

            using UnitTestInputC = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanA = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanB = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanC = IOVariable<TAG_ID, Bit>;

            using UnitTimingPlanD = IOVariable<TAG_ID, Bit>;

            using UnitWalkRestModifier = IOVariable<TAG_ID, Bit>;

            template<index_t I> requires (IsValidIndex(I, cu::detector::max_vehicle_detectors))
            using VehicleDetCall = IOVariable<TAG_ID, Bit, I>;

            template<index_t I> requires (I >= 1)
            using VehicleDetReset = IOVariable<TAG_ID, Bit, I>;
        } // end of namespace input

        template<typename T> requires is_io_variable<T>
        T variable{};

    } // end of namespace io

    template<typename T>
    requires io::is_io_variable<T>
    constexpr T &variable() {
        return io::variable<T>;
    }

    template<typename T>
    requires cu::is_cu_variable<T>
    constexpr T &variable() {
        return cu::variable<T>;
    }

    namespace serialbus {

        struct DataFrameElementType
        {
        };

        template<typename T, size_t BitPos> requires std::is_same_v<VariableValueType<T>, Bit>
        struct DataframeBit
        {
            using type = DataFrameElementType;

            void operator<<(const std::span<const Byte> a_data_in) {
                static constexpr auto l_byte_pos = BitPos / sizeof(Byte);
                static constexpr auto l_num_of_bits_to_shift = BitPos % sizeof(Byte);
                Byte i = 1;
                auto l_value = (a_data_in[l_byte_pos] & (i << l_num_of_bits_to_shift)) != 0;
                ref_var.value = static_cast<Bit>(l_value);
            }

            void operator>>(const std::span<Byte> a_data_out) {
                static constexpr auto l_byte_pos = BitPos / sizeof(Byte);
                static constexpr auto l_num_of_bits_to_shift = BitPos % sizeof(Byte);
                Byte i = (ref_var.value = Bit::on) ? 1 : 0;
                a_data_out[l_byte_pos] = a_data_out[l_byte_pos] | (i << l_num_of_bits_to_shift);
            }

            size_t pos{BitPos};
            T &ref_var{variable<T>};
        };

        template<typename T, size_t BytePos> requires std::is_same_v<VariableValueType<T>, Byte>
        struct DataframeByte
        {
            using type = DataFrameElementType;

            void operator<<(const std::span<const Byte> a_data_in) {
                ref_var.value = a_data_in[BytePos];
            }

            void operator>>(const std::span<Byte> a_data_out) {
                a_data_out[BytePos] = ref_var.value;
            }

            size_t pos{BytePos};
            T &ref_var{variable<T>};
        };

        template<typename T, size_t BytePos> requires std::is_same_v<VariableValueType<T>, Word>
        struct DataframeWord
        {
            using type = DataFrameElementType;

            void operator<<(const std::span<const Byte> a_data_in) {
                // LowByte | HighByte
                ref_var.value = (a_data_in[BytePos] & 0x00FF) | (a_data_in[BytePos + 1] << 8);
            }

            void operator>>(const std::span<Byte> a_data_out) {
                a_data_out[BytePos] = ref_var.value & 0x00FF; // LowByte
                a_data_out[BytePos + 1] = ref_var.value >> 8; // HighByte
            }

            size_t pos{BytePos};
            T &ref_var{variable<T>};
        };

        template<typename T>
        concept is_dataframe_elem = std::is_same_v<DataFrameElementType, typename T::type>;

        // Primary Station Out
        struct PrimaryCommandFrameType
        {
        };

        // Primary Station In
        struct PrimaryResponseFrameType
        {
        };

        // Secondary Station In
        struct SecondaryCommandFrameType
        {
        };

        // Secondary Station Out
        struct SecondaryResponseFrameType
        {
        };

        template<typename T>
        concept is_dataframe = std::is_same_v<T, PrimaryCommandFrameType>
                               || std::is_same_v<T, PrimaryResponseFrameType>
                               || std::is_same_v<T, SecondaryCommandFrameType>
                               || std::is_same_v<T, SecondaryResponseFrameType>;

        template<Byte FrameID, size_t FrameByteSize, typename T, typename ...Ts> requires is_dataframe<T> && (is_dataframe_elem<Ts> &&...)
        class Dataframe
        {
        public:
            using type = T;

            Dataframe() = default;

            Dataframe(Dataframe &) = delete;

            Dataframe(Dataframe &&) = delete;

            Dataframe &operator=(Dataframe &) = delete;

            Dataframe &operator=(Dataframe &&) = delete;

            template<typename = T>
            requires (std::is_same_v<T, PrimaryResponseFrameType> || std::is_same_v<T, SecondaryCommandFrameType>)
            void operator<<(const std::span<const Byte, FrameByteSize> a_data_in) {
                assert(a_data_in[0] == FrameID);
                assign(a_data_in);
            }

            template<typename = T>
            requires (std::is_same_v<T, PrimaryCommandFrameType> || std::is_same_v<T, SecondaryResponseFrameType>)
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
                std::get<I>(m_frame_elements) << a_data_in;
                if constexpr (I + 1 != sizeof...(Ts)) {
                    assign < I + 1 > (a_data_in);
                }
            }

            template<size_t I = 0>
            inline void generate(const std::span<Byte, FrameByteSize> a_data_out) {
                std::get<I>(m_frame_elements) >> a_data_out;
                if constexpr (I + 1 != sizeof...(Ts)) {
                    generate < I + 1 > (a_data_out);
                }
            }

            std::tuple<Ts...> m_frame_elements;
        };
    } // end of namespace serialbus

    namespace biu {

    } // end of namespace biu

} // end of namespace atc


#endif