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

    enum class State : uint16_t
    {
        off = 0, on = 1
    };
    
    namespace cu {
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
    } // end of cu

    namespace fio {
        using fio_index_t = uint16_t;

        struct fio_input_t
        {
        };

        struct fio_output_t
        {
        };

        template<typename T, fio_index_t I = 0> requires std::is_same_v<T, fio_input_t> || std::is_same_v<T, fio_output_t>
        class FieldIO
        {
        public:
            using io_catetory = T;

            FieldIO() = default;

            FieldIO(const State a_state) = delete;

            FieldIO(FieldIO &rhs) = delete;

            FieldIO(FieldIO &&rhs) = delete;

            FieldIO &operator=(FieldIO &rhs) = delete;

            FieldIO &operator=(FieldIO &&rhs) = delete;

            static constexpr fio_index_t fio_index{I};
            std::atomic<State> state{State::off};
        };

        template<typename T>
        concept is_fio_type = requires {
            std::is_same_v<typename T::FieldIO::io_catetory, fio_input_t> || std::is_same_v<typename T::FieldIO::io_catetory, fio_output_t>;
        };

        template<typename T> requires is_fio_type<T>
        using IOCatetory = typename T::FieldIO::io_catetory;

        constexpr bool is_valid_fio_index(fio_index_t I, uint16_t N) {
            return (I >= 1) && (I <= N);
        };

        namespace output {

            struct AltFlashState : public FieldIO<fio_output_t>
            {
            };

            struct AuxFunctionState : public FieldIO<fio_output_t>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::channel::max_channels))
            struct ChannelGreenWalkDriver : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::channel::max_channels))
            struct ChannelRedDoNotWalkDriver : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::channel::max_channels))
            struct ChannelYellowRedClearDriver : public FieldIO<fio_output_t, I>
            {
            };

            struct CustomAlarm : public FieldIO<fio_output_t>
            {
            };

            template<fio_index_t I> requires (I >= 1)
            struct DetectorReset : public FieldIO<fio_output_t, I>
            {
            };

            struct FlashState : public FieldIO<fio_output_t>
            {
            };

            struct GlobalVariable : public FieldIO<fio_output_t>
            {
            };

            struct NotActive : public FieldIO<fio_output_t>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::overlap::max_overlaps))
            struct OverlapGreen : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::overlap::max_overlaps))
            struct OverlapProtectedGreen : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::overlap::max_overlaps))
            struct OverlapRed : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::overlap::max_overlaps))
            struct OverlapYellow : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PedCall : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseAdvWarning : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseCheck : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseDoNotWalk : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseGreen : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseNext : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseOmit : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseOn : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhasePedClearance : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhasePreClear : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhasePreClear2 : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseRed : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseWalk : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseYellow : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptStatus : public FieldIO<fio_output_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptStatusFlash : public FieldIO<fio_output_t, I>
            {
            };

            struct RingStatusBitA : public FieldIO<fio_output_t>
            {
            };

            struct RingStatusBitB : public FieldIO<fio_output_t>
            {
            };

            struct RingStatusBitC : public FieldIO<fio_output_t>
            {
            };

            struct RingStatusBitD : public FieldIO<fio_output_t>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::unit::max_special_function_outputs))
            struct SpecialFunction : public FieldIO<fio_output_t, I>
            {
            };

            struct UnitAutomaticFlash : public FieldIO<fio_output_t>
            {
            };

            struct UnitFaultMonitor : public FieldIO<fio_output_t>
            {
            };

            struct UnitFreeCoordStatus : public FieldIO<fio_output_t>
            {
            };

            struct UnitOffset_1 : public FieldIO<fio_output_t>
            {
            };

            struct UnitOffset_2 : public FieldIO<fio_output_t>
            {
            };

            struct UnitOffset_3 : public FieldIO<fio_output_t>
            {
            };

            struct UnitTBCAux_1 : public FieldIO<fio_output_t>
            {
            };

            struct UnitTBCAux_2 : public FieldIO<fio_output_t>
            {
            };

            struct UnitTBCAux_3 : public FieldIO<fio_output_t>
            {
            };

            struct UnitTimingPlanA : public FieldIO<fio_output_t>
            {
            };

            struct UnitTimingPlanB : public FieldIO<fio_output_t>
            {
            };

            struct UnitTimingPlanC : public FieldIO<fio_output_t>
            {
            };

            struct UnitTimingPlanD : public FieldIO<fio_output_t>
            {
            };

            struct UnitVoltageMonitor : public FieldIO<fio_output_t>
            {
            };

            struct Watchdog : public FieldIO<fio_output_t>
            {
            };

        } // end of namespace fio::output

        namespace input {

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::detector::max_vehicle_detectors))
            struct ChannelFaultStatus : public FieldIO<fio_input_t, I>
            {
            };

            struct CoordFreeSwitch : public FieldIO<fio_input_t>
            {
            };

            struct CustomAlarm : public FieldIO<fio_input_t>
            {
            };

            struct DoorAjor : public FieldIO<fio_input_t>
            {
            };

            struct ManualControlGroupAction : public FieldIO<fio_input_t>
            {
            };

            struct MinGreen_2 : public FieldIO<fio_input_t>
            {
            };

            struct NotActive : public FieldIO<fio_input_t>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::overlap::max_overlaps))
            struct OverlapOmit : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::coord::max_patterns))
            struct PatternInput : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I,
                                                                 cu::detector::max_pedestrian_detectors))
            struct PedDetCall : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseForceOff : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhaseHold : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhasePedOmit : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::phase::max_phases))
            struct PhasePhaseOmit : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptGateDown : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptGateUp : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptHighPrioritorLow : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptInput : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptInputCRC : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptInputNormalOff : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::preempt::max_preempts))
            struct PreemptInputNormalOn : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::prioritor::max_prioritors))
            struct PrioritorCheckIn : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::prioritor::max_prioritors))
            struct PrioritorCheckOut : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (I >= 1)
            struct PrioritorPreemptDetector : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingForceOff : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingInhibitMaxTermination : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingMax2Selection : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingMax3Selection : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingOmitRedClearance : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingPedestrianRecycle : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingRedRest : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct RingStopTiming : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::ring::max_rings))
            struct SpecialFunctionInput : public FieldIO<fio_input_t, I>
            {
            };

            struct UnitAlarm_1 : public FieldIO<fio_input_t>
            {
            };

            struct UnitAlarm_2 : public FieldIO<fio_input_t>
            {
            };

            struct UnitAlternateSequenceA : public FieldIO<fio_input_t>
            {
            };

            struct UnitAlternateSequenceB : public FieldIO<fio_input_t>
            {
            };

            struct UnitAlternateSequenceC : public FieldIO<fio_input_t>
            {
            };

            struct UnitAlternateSequenceD : public FieldIO<fio_input_t>
            {
            };

            struct UnitAutomaticFlash : public FieldIO<fio_input_t>
            {
            };

            struct UnitCallPedNAPlus : public FieldIO<fio_input_t>
            {
            };

            struct UnitCallToNonActuated_1 : public FieldIO<fio_input_t>
            {
            };

            struct UnitCallToNonActuated_2 : public FieldIO<fio_input_t>
            {
            };

            struct UnitClockReset : public FieldIO<fio_input_t>
            {
            };

            struct UnitCMUMMUFlashStatus : public FieldIO<fio_input_t>
            {
            };

            struct UnitDimming : public FieldIO<fio_input_t>
            {
            };

            struct UnitExternWatchDog : public FieldIO<fio_input_t>
            {
            };

            struct UnitExternalMinRecall : public FieldIO<fio_input_t>
            {
            };

            struct UnitExternalStart : public FieldIO<fio_input_t>
            {
            };

            struct UnitIndicatorLampControl : public FieldIO<fio_input_t>
            {
            };

            struct UnitIntervalAdvance : public FieldIO<fio_input_t>
            {
            };

            struct UnitIOModeBit_0 : public FieldIO<fio_input_t>
            {
            };

            struct UnitIOModeBit_1 : public FieldIO<fio_input_t>
            {
            };

            struct UnitIOModeBit_2 : public FieldIO<fio_input_t>
            {
            };

            struct UnitIOModeBit_3 : public FieldIO<fio_input_t>
            {
            };

            struct UnitITSLocalFlashSense : public FieldIO<fio_input_t>
            {
            };

            struct UnitLocalFlash : public FieldIO<fio_input_t>
            {
            };

            struct UnitLocalFlashSense : public FieldIO<fio_input_t>
            {
            };

            struct UnitManualControlEnable : public FieldIO<fio_input_t>
            {
            };

            struct UnitOffset_1 : public FieldIO<fio_input_t>
            {
            };

            struct UnitOffset_2 : public FieldIO<fio_input_t>
            {
            };

            struct UnitOffset_3 : public FieldIO<fio_input_t>
            {
            };

            struct UnitSignalPlanA : public FieldIO<fio_input_t>
            {
            };

            struct UnitSignalPlanB : public FieldIO<fio_input_t>
            {
            };

            struct UnitStopTIme : public FieldIO<fio_input_t>
            {
            };

            struct UnitSystemAddressBit_0 : public FieldIO<fio_input_t>
            {
            };

            struct UnitSystemAddressBit_1 : public FieldIO<fio_input_t>
            {
            };

            struct UnitSystemAddressBit_2 : public FieldIO<fio_input_t>
            {
            };

            struct UnitSystemAddressBit_3 : public FieldIO<fio_input_t>
            {
            };

            struct UnitSystemAddressBit_4 : public FieldIO<fio_input_t>
            {
            };

            struct UnitTBCHoldOnline : public FieldIO<fio_input_t>
            {
            };

            struct UnitTBCOnline : public FieldIO<fio_input_t>
            {
            };

            struct UnitTestInputA : public FieldIO<fio_input_t>
            {
            };

            struct UnitTestInputB : public FieldIO<fio_input_t>
            {
            };

            struct UnitTestInputC : public FieldIO<fio_input_t>
            {
            };

            struct UnitTimingPlanA : public FieldIO<fio_input_t>
            {
            };

            struct UnitTimingPlanB : public FieldIO<fio_input_t>
            {
            };

            struct UnitTimingPlanC : public FieldIO<fio_input_t>
            {
            };

            struct UnitTimingPlanD : public FieldIO<fio_input_t>
            {
            };

            struct UnitWalkRestModifier : public FieldIO<fio_input_t>
            {
            };

            template<fio_index_t I> requires (is_valid_fio_index(I, cu::detector::max_vehicle_detectors))
            struct VehicleDetCall : public FieldIO<fio_input_t, I>
            {
            };

            template<fio_index_t I> requires (I >= 1)
            struct VehicleDetReset : public FieldIO<fio_input_t, I>
            {
            };
        } // end of namespace fio::input

        template<typename T> requires is_fio_type<T>
        T fio{};
    } // end of fio

    namespace serial_com {

        struct dataframe_bit_definition_t
        {
        };

        template<typename T, uint16_t BitPos> requires fio::is_fio_type<T>
        struct DataframeBitDefinition
        {
            using type = dataframe_bit_definition_t;
            T &state{atc::fio::fio<T>.state};
        };

        template<uint16_t FrameID, uint16_t FrameByteSize, typename ...Ts> requires(std::is_same_v<dataframe_bit_definition_t, typename Ts::type> &&...)
        struct Dataframe
        {
            std::tuple<Ts...> bit_definitions;
        };

    } // end serial_com




} // end of namespace atc


#endif