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
#include <type_traits>

namespace atc {

    namespace phase {
        constexpr uint16_t max_phases{40};
        constexpr uint16_t max_phase_groups{5};
    }

    namespace detector {
        constexpr uint16_t max_vehicle_detectors{128};
        constexpr uint16_t max_vehicle_detector_status_groups{40};
        constexpr uint16_t max_pedestrian_detectors{72};
    }

    namespace ring {
        constexpr uint16_t max_rings{16};
        constexpr uint16_t max_sequences{20};
        constexpr uint16_t max_ring_control_groups{2};

    }

    namespace channel {
        constexpr uint16_t max_channels{32};
        constexpr uint16_t max_channel_status_groups{4};
    }

    namespace overlap {
        constexpr uint16_t max_overlaps{32};
        constexpr uint16_t max_overlap_status_groups{4};
    }

    namespace preempt {
        constexpr uint16_t max_preempts{40};
    }

    namespace unit {
        constexpr uint16_t max_alarm_groups{1};
        constexpr uint16_t max_special_function_outputs{16};
    }

    namespace coord {
        constexpr uint16_t max_patterns{128};
        constexpr uint16_t max_splits{128};
    }

    namespace timebase_asc {
        constexpr uint16_t max_timebase_asc_actions{64};
    }

    namespace prioritor {
        constexpr uint16_t max_prioritors{16};
        constexpr uint16_t max_prioritor_groups{9};
    }

    namespace io {
        using io_index_t = uint16_t;

        enum class Bit : bool
        {
            off = false, on = true
        };

        using Byte = uint8_t;
        using Word = uint16_t;

        struct io_input_category
        {
        };

        struct io_output_category
        {
        };

        template<typename T>
        concept is_io_category = requires {
            std::is_same_v<T, io_input_category> || std::is_same_v<T, io_input_category>;
        };

        template<typename T>
        concept is_valid_io_value_t = requires {
            std::is_same_v<T, Bit> || std::is_same_v<T, Byte> || std::is_same_v<T, Word>;
        };

        template<typename ValueT, typename CategoryT, io_index_t I = 0> requires is_io_category<CategoryT> && is_valid_io_value_t<ValueT>
        struct IO
        {
            using io_catetory = CategoryT;
            using value_t = ValueT;

            IO() = default;

            IO(IO &) = delete;

            IO(IO &&) = delete;

            IO &operator=(IO &) = delete;

            IO &operator=(IO &&) = delete;

            static constexpr io_index_t fio_index{I};
            std::atomic<ValueT> value{};
        };

        template<typename T>
        concept is_io = requires {
            std::is_same_v<typename T::IO::io_catetory, io_input_category>
            || std::is_same_v<typename T::IO::io_catetory, io_output_category>;
        };

        template<typename T>
        concept is_bit_io = requires {
            std::is_same_v<typename T::IO::value_t, Bit>
            && (std::is_same_v<typename T::IO::io_catetory, io_input_category>
                || std::is_same_v<typename T::IO::io_catetory, io_output_category>);
        };

        template<typename T>
        concept is_byte_io = requires {
            std::is_same_v<typename T::IO::value_t, uint8_t>
            && (std::is_same_v<typename T::IO::io_catetory, io_input_category>
                || std::is_same_v<typename T::IO::io_catetory, io_output_category>);
        };

        template<typename T>
        concept is_word_io = requires {
            std::is_same_v<typename T::IO::value_t, uint16_t>
            && (std::is_same_v<typename T::IO::io_catetory, io_input_category>
                || std::is_same_v<typename T::IO::io_catetory, io_output_category>);
        };

        template<typename T> requires is_io<T>
        using IOCatetory = typename T::IO::io_catetory;

        constexpr bool is_valid_io_index(io_index_t I, uint16_t N) {
            return (I >= 1) && (I <= N);
        };

        namespace output {

            struct AltFlashState : public IO<Bit, io_output_category>
            {
            };

            struct AuxFunctionState : public IO<Bit, io_output_category>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, channel::max_channels))
            struct ChannelGreenWalkDriver : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, channel::max_channels))
            struct ChannelRedDoNotWalkDriver : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, channel::max_channels))
            struct ChannelYellowRedClearDriver : public IO<Bit, io_output_category, I>
            {
            };

            struct CustomAlarm : public IO<Bit, io_output_category>
            {
            };

            template<io_index_t I> requires (I >= 1)
            struct DetectorReset : public IO<Bit, io_output_category, I>
            {
            };

            struct FlashState : public IO<Bit, io_output_category>
            {
            };

            struct GlobalVariable : public IO<Bit, io_output_category>
            {
            };

            struct NotActive : public IO<Bit, io_output_category>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, overlap::max_overlaps))
            struct OverlapGreen : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, overlap::max_overlaps))
            struct OverlapProtectedGreen : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, overlap::max_overlaps))
            struct OverlapRed : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, overlap::max_overlaps))
            struct OverlapYellow : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PedCall : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseAdvWarning : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseCheck : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseDoNotWalk : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseGreen : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseNext : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseOmit : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseOn : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhasePedClearance : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhasePreClear : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhasePreClear2 : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseRed : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseWalk : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseYellow : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptStatus : public IO<Bit, io_output_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptStatusFlash : public IO<Bit, io_output_category, I>
            {
            };

            struct RingStatusBitA : public IO<Bit, io_output_category>
            {
            };

            struct RingStatusBitB : public IO<Bit, io_output_category>
            {
            };

            struct RingStatusBitC : public IO<Bit, io_output_category>
            {
            };

            struct RingStatusBitD : public IO<Bit, io_output_category>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, unit::max_special_function_outputs))
            struct SpecialFunction : public IO<Bit, io_output_category, I>
            {
            };

            struct UnitAutomaticFlash : public IO<Bit, io_output_category>
            {
            };

            struct UnitFaultMonitor : public IO<Bit, io_output_category>
            {
            };

            struct UnitFreeCoordStatus : public IO<Bit, io_output_category>
            {
            };

            struct UnitOffset_1 : public IO<Bit, io_output_category>
            {
            };

            struct UnitOffset_2 : public IO<Bit, io_output_category>
            {
            };

            struct UnitOffset_3 : public IO<Bit, io_output_category>
            {
            };

            struct UnitTBCAux_1 : public IO<Bit, io_output_category>
            {
            };

            struct UnitTBCAux_2 : public IO<Bit, io_output_category>
            {
            };

            struct UnitTBCAux_3 : public IO<Bit, io_output_category>
            {
            };

            struct UnitTimingPlanA : public IO<Bit, io_output_category>
            {
            };

            struct UnitTimingPlanB : public IO<Bit, io_output_category>
            {
            };

            struct UnitTimingPlanC : public IO<Bit, io_output_category>
            {
            };

            struct UnitTimingPlanD : public IO<Bit, io_output_category>
            {
            };

            struct UnitVoltageMonitor : public IO<Bit, io_output_category>
            {
            };

            struct Watchdog : public IO<Bit, io_output_category>
            {
            };

        } // end of namespace output

        namespace input {

            template<io_index_t I> requires (is_valid_io_index(I, detector::max_vehicle_detectors))
            struct ChannelFaultStatus : public IO<Bit, io_input_category, I>
            {
            };

            struct CoordFreeSwitch : public IO<Bit, io_input_category>
            {
            };

            struct CustomAlarm : public IO<Bit, io_input_category>
            {
            };

            struct DoorAjor : public IO<Bit, io_input_category>
            {
            };

            struct ManualControlGroupAction : public IO<Bit, io_input_category>
            {
            };

            struct MinGreen_2 : public IO<Bit, io_input_category>
            {
            };

            struct NotActive : public IO<Bit, io_input_category>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, overlap::max_overlaps))
            struct OverlapOmit : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, coord::max_patterns))
            struct PatternInput : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, detector::max_pedestrian_detectors))
            struct PedDetCall : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseForceOff : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhaseHold : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhasePedOmit : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, phase::max_phases))
            struct PhasePhaseOmit : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptGateDown : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptGateUp : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptHighPrioritorLow : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptInput : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptInputCRC : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptInputNormalOff : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, preempt::max_preempts))
            struct PreemptInputNormalOn : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, prioritor::max_prioritors))
            struct PrioritorCheckIn : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, prioritor::max_prioritors))
            struct PrioritorCheckOut : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (I >= 1)
            struct PrioritorPreemptDetector : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingForceOff : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingInhibitMaxTermination : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingMax2Selection : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingMax3Selection : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingOmitRedClearance : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingPedestrianRecycle : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingRedRest : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct RingStopTiming : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, ring::max_rings))
            struct SpecialFunctionInput : public IO<Bit, io_input_category, I>
            {
            };

            struct UnitAlarm_1 : public IO<Bit, io_input_category>
            {
            };

            struct UnitAlarm_2 : public IO<Bit, io_input_category>
            {
            };

            struct UnitAlternateSequenceA : public IO<Bit, io_input_category>
            {
            };

            struct UnitAlternateSequenceB : public IO<Bit, io_input_category>
            {
            };

            struct UnitAlternateSequenceC : public IO<Bit, io_input_category>
            {
            };

            struct UnitAlternateSequenceD : public IO<Bit, io_input_category>
            {
            };

            struct UnitAutomaticFlash : public IO<Bit, io_input_category>
            {
            };

            struct UnitCallPedNAPlus : public IO<Bit, io_input_category>
            {
            };

            struct UnitCallToNonActuated_1 : public IO<Bit, io_input_category>
            {
            };

            struct UnitCallToNonActuated_2 : public IO<Bit, io_input_category>
            {
            };

            struct UnitClockReset : public IO<Bit, io_input_category>
            {
            };

            struct UnitCMUMMUFlashStatus : public IO<Bit, io_input_category>
            {
            };

            struct UnitDimming : public IO<Bit, io_input_category>
            {
            };

            struct UnitExternWatchDog : public IO<Bit, io_input_category>
            {
            };

            struct UnitExternalMinRecall : public IO<Bit, io_input_category>
            {
            };

            struct UnitExternalStart : public IO<Bit, io_input_category>
            {
            };

            struct UnitIndicatorLampControl : public IO<Bit, io_input_category>
            {
            };

            struct UnitIntervalAdvance : public IO<Bit, io_input_category>
            {
            };

            struct UnitIOModeBit_0 : public IO<Bit, io_input_category>
            {
            };

            struct UnitIOModeBit_1 : public IO<Bit, io_input_category>
            {
            };

            struct UnitIOModeBit_2 : public IO<Bit, io_input_category>
            {
            };

            struct UnitIOModeBit_3 : public IO<Bit, io_input_category>
            {
            };

            struct UnitITSLocalFlashSense : public IO<Bit, io_input_category>
            {
            };

            struct UnitLocalFlash : public IO<Bit, io_input_category>
            {
            };

            struct UnitLocalFlashSense : public IO<Bit, io_input_category>
            {
            };

            struct UnitManualControlEnable : public IO<Bit, io_input_category>
            {
            };

            struct UnitOffset_1 : public IO<Bit, io_input_category>
            {
            };

            struct UnitOffset_2 : public IO<Bit, io_input_category>
            {
            };

            struct UnitOffset_3 : public IO<Bit, io_input_category>
            {
            };

            struct UnitSignalPlanA : public IO<Bit, io_input_category>
            {
            };

            struct UnitSignalPlanB : public IO<Bit, io_input_category>
            {
            };

            struct UnitStopTIme : public IO<Bit, io_input_category>
            {
            };

            struct UnitSystemAddressBit_0 : public IO<Bit, io_input_category>
            {
            };

            struct UnitSystemAddressBit_1 : public IO<Bit, io_input_category>
            {
            };

            struct UnitSystemAddressBit_2 : public IO<Bit, io_input_category>
            {
            };

            struct UnitSystemAddressBit_3 : public IO<Bit, io_input_category>
            {
            };

            struct UnitSystemAddressBit_4 : public IO<Bit, io_input_category>
            {
            };

            struct UnitTBCHoldOnline : public IO<Bit, io_input_category>
            {
            };

            struct UnitTBCOnline : public IO<Bit, io_input_category>
            {
            };

            struct UnitTestInputA : public IO<Bit, io_input_category>
            {
            };

            struct UnitTestInputB : public IO<Bit, io_input_category>
            {
            };

            struct UnitTestInputC : public IO<Bit, io_input_category>
            {
            };

            struct UnitTimingPlanA : public IO<Bit, io_input_category>
            {
            };

            struct UnitTimingPlanB : public IO<Bit, io_input_category>
            {
            };

            struct UnitTimingPlanC : public IO<Bit, io_input_category>
            {
            };

            struct UnitTimingPlanD : public IO<Bit, io_input_category>
            {
            };

            struct UnitWalkRestModifier : public IO<Bit, io_input_category>
            {
            };

            template<io_index_t I> requires (is_valid_io_index(I, detector::max_vehicle_detectors))
            struct VehicleDetCall : public IO<Bit, io_input_category, I>
            {
            };

            template<io_index_t I> requires (I >= 1)
            struct VehicleDetReset : public IO<Bit, io_input_category, I>
            {
            };
        } // end of namespace input

        template<typename T> requires is_io<T>
        T io{};

    } // end of namespace io

    namespace serial_com {
        // A bit backed-up by IO.
        struct dataframe_bit_t
        {
        };

        struct dataframe_byte_t
        {
        };

        struct dataframe_word_t
        {
        };

        template<typename T, uint16_t BitPos> requires io::is_bit_io<T>
        struct DataframeBit
        {
            using type = dataframe_bit_t;
            // io::bit_io<T> is the IO using a Bit to store its value. We create a ref here.
            T &r_bit_io{io::io<T>};
        };

        template<typename T, uint16_t BytePos> requires io::is_byte_io<T>
        struct DataframeByte
        {
            using type = dataframe_byte_t;
            // io::byte_io<T> is the IO using a Byte to store its value. We create a ref here.
            T &r_byte_io{io::io<T>};
        };

        template<typename T, uint16_t WordPos> requires io::is_word_io<T>
        struct DataframeWord
        {
            using type = dataframe_word_t;
            // io::word_io<T> is the IO using a Word to store its value. We create a ref here.
            T &r_word_io{io::io<T>};
        };

        template<typename T>
        concept is_valid_dataframe_element = requires {
            std::is_same_v<dataframe_bit_t, typename T::type>
            || std::is_same_v<dataframe_byte_t, typename T::type>
            || std::is_same_v<dataframe_word_t, typename T::type>;
        };

        template<int16_t FrameID, uint16_t FrameByteSize, typename ...Ts> requires(is_valid_dataframe_element<Ts> &&...)
        class Dataframe
        {
        public:
            Dataframe() = default;

            Dataframe(Dataframe &) = delete;

            Dataframe(Dataframe &&) = delete;

            Dataframe &operator=(Dataframe &) = delete;

            Dataframe &operator=(Dataframe &&) = delete;

            static constexpr int16_t frame_id{FrameID};
        private:
            std::tuple<Ts...> m_frame_definition;
        };

    } // end serial_com

} // end of namespace atc


#endif