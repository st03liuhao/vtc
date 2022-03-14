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

namespace atc {

    using fioidx_t = unsigned short int;
    using uint16_t = unsigned short int;

    namespace controller_unit {
        namespace phase {
            constexpr uint16_t max_phases{40};
            [[maybe_unused]] constexpr uint16_t max_phase_groups{5};
        }

        namespace detector {
            constexpr uint16_t max_vehicle_detectors{128};
            [[maybe_unused]] constexpr uint16_t max_vehicle_detector_status_groups{40};
            constexpr uint16_t max_pedestrian_detectors{72};
        }

        namespace ring {
            constexpr uint16_t max_rings{16};
            [[maybe_unused]] constexpr uint16_t max_sequences{20};
            [[maybe_unused]] constexpr uint16_t max_ring_control_groups{2};

        }

        namespace channel {
            constexpr uint16_t max_channels{32};
            [[maybe_unused]] constexpr uint16_t max_channel_status_groups{4};
        }

        namespace overlap {
            constexpr uint16_t max_overlaps{32};
            [[maybe_unused]] constexpr uint16_t max_overlap_status_groups{4};
        }

        namespace preempt {
            constexpr uint16_t max_preempts{40};
        }

        namespace unit {
            [[maybe_unused]] constexpr uint16_t max_alarm_groups{1};
            constexpr uint16_t max_special_function_outputs{16};
        }

        namespace coord {
            constexpr uint16_t max_patterns{128};
            [[maybe_unused]] constexpr uint16_t max_splits{128};
        }

        namespace timebase_asc {
            [[maybe_unused]] constexpr uint16_t max_timebase_asc_actions{64};
        }

        namespace prioritor {
            constexpr uint16_t max_prioritors{16};
            [[maybe_unused]] constexpr uint16_t max_prioritor_groups{9};
        }
    }

    namespace field_io {

        enum class State : uint16_t
        {
            off = 0, on = 1
        };

        class FieldIO
        {
        public:
            FieldIO() = delete;

            State operator()() {
                return m_state;
            };

            FieldIO &operator=(const State a_state) {
                m_state = a_state;
                return *this;
            };
        private:
            std::atomic<State> m_state{State::off};
        };

        struct InputTag
        {
        };

        struct OutputTag
        {
        };

        // I = 0 means no siblings.
        template<fioidx_t I = 0>
        struct Input : FieldIO
        {
            using io_catetory [[maybe_unused]] = InputTag;

            Input() = delete;

            [[maybe_unused]] static constexpr fioidx_t io_index = I;
        };

        // I = 0 means no siblings.
        template<fioidx_t I = 0>
        struct Output : FieldIO
        {
            using io_category [[maybe_unused]] = OutputTag;

            Output() = delete;

            [[maybe_unused]] static constexpr fioidx_t io_index = I;
        };

        template<typename T> requires std::is_base_of_v<FieldIO, T> && T::io_category
        using IOCatetory [[maybe_unused]] = typename T::io_category;

        namespace output {

            struct [[maybe_unused]] AltFlashState : public Output<>
            {
            };

            struct [[maybe_unused]] AuxFunctionState : public Output<>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::channel::max_channels)
            struct [[maybe_unused]] ChannelGreenWalkDriver : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::channel::max_channels)
            struct [[maybe_unused]] ChannelRedDoNotWalkDriver : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::channel::max_channels)
            struct [[maybe_unused]] ChannelYellowRedClearDriver : public Output<I>
            {
            };

            struct [[maybe_unused]] CustomAlarm : public Output<>
            {
            };

            template<fioidx_t I> requires (I >= 1)
            struct [[maybe_unused]] DetectorReset : public Output<I>
            {
            };

            struct [[maybe_unused]] FlashState : public Output<>
            {
            };

            struct [[maybe_unused]] GlobalVariable : public Output<>
            {
            };

            struct [[maybe_unused]] NotActive : public Output<>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::overlap::max_overlaps)
            struct [[maybe_unused]] OverlapGreen : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::overlap::max_overlaps)
            struct [[maybe_unused]] OverlapProtectedGreen : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::overlap::max_overlaps)
            struct [[maybe_unused]] OverlapRed : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::overlap::max_overlaps)
            struct [[maybe_unused]] OverlapYellow : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PedCall : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseAdvWarning : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseCheck : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseDoNotWalk : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseGreen : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseNext : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseOmit : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseOn : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhasePedClearance : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhasePreClear : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhasePreClear2 : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseRed : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseWalk : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseYellow : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptStatus : public Output<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptStatusFlash : public Output<I>
            {
            };

            struct [[maybe_unused]] RingStatusBitA : public Output<>
            {
            };

            struct [[maybe_unused]] RingStatusBitB : public Output<>
            {
            };

            struct [[maybe_unused]] RingStatusBitC : public Output<>
            {
            };

            struct [[maybe_unused]] RingStatusBitD : public Output<>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::unit::max_special_function_outputs)
            struct [[maybe_unused]] SpecialFunction : public Output<I>
            {
            };

            struct [[maybe_unused]] UnitAutomaticFlash : public Output<>
            {
            };

            struct [[maybe_unused]] UnitFaultMonitor : public Output<>
            {
            };

            struct [[maybe_unused]] UnitFreeCoordStatus : public Output<>
            {
            };

            struct [[maybe_unused]] UnitOffset_1 : public Output<>
            {
            };

            struct [[maybe_unused]] UnitOffset_2 : public Output<>
            {
            };

            struct [[maybe_unused]] UnitOffset_3 : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTBCAux_1 : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTBCAux_2 : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTBCAux_3 : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanA : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanB : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanC : public Output<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanD : public Output<>
            {
            };

            struct [[maybe_unused]] UnitVoltageMonitor : public Output<>
            {
            };

            struct [[maybe_unused]] Watchdog : public Output<>
            {
            };

        } // end of namespace field_io::output

        namespace input {

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::detector::max_vehicle_detectors)
            struct [[maybe_unused]] ChannelFaultStatus : public Input<I>
            {
            };

            struct [[maybe_unused]] CoordFreeSwitch : public Input<>
            {
            };

            struct [[maybe_unused]] CustomAlarm : public Input<>
            {
            };

            struct [[maybe_unused]] DoorAjor : public Input<>
            {
            };

            struct [[maybe_unused]] ManualControlGroupAction : public Input<>
            {
            };

            struct [[maybe_unused]] MinGreen_2 : public Input<>
            {
            };

            struct [[maybe_unused]] NotActive : public Input<>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::overlap::max_overlaps)
            struct [[maybe_unused]] OverlapOmit : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::coord::max_patterns)
            struct [[maybe_unused]] PatternInput : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::detector::max_pedestrian_detectors)
            struct [[maybe_unused]] PedDetCall : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseForceOff : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhaseHold : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhasePedOmit : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::phase::max_phases)
            struct [[maybe_unused]] PhasePhaseOmit : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptGateDown : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptGateUp : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptHighPrioritorLow : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptInput : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptInputCRC : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptInputNormalOff : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::preempt::max_preempts)
            struct [[maybe_unused]] PreemptInputNormalOn : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::prioritor::max_prioritors)
            struct [[maybe_unused]] PrioritorCheckIn : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::prioritor::max_prioritors)
            struct [[maybe_unused]] PrioritorCheckOut : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1)
            struct [[maybe_unused]] PrioritorPreemptDetector : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingForceOff : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingInhibitMaxTermination : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingMax2Selection : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingMax3Selection : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingOmitRedClearance : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingPedestrianRecycle : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingRedRest : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] RingStopTiming : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::ring::max_rings)
            struct [[maybe_unused]] SpecialFunctionInput : public Input<I>
            {
            };

            struct [[maybe_unused]] UnitAlarm_1 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitAlarm_2 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitAlternateSequenceA : public Input<>
            {
            };

            struct [[maybe_unused]] UnitAlternateSequenceB : public Input<>
            {
            };

            struct [[maybe_unused]] UnitAlternateSequenceC : public Input<>
            {
            };

            struct [[maybe_unused]] UnitAlternateSequenceD : public Input<>
            {
            };

            struct [[maybe_unused]] UnitAutomaticFlash : public Input<>
            {
            };

            struct [[maybe_unused]] UnitCallPedNAPlus : public Input<>
            {
            };

            struct [[maybe_unused]] UnitCallToNonActuated_1 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitCallToNonActuated_2 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitClockReset : public Input<>
            {
            };

            struct [[maybe_unused]] UnitCMUMMUFlashStatus : public Input<>
            {
            };

            struct [[maybe_unused]] UnitDimming : public Input<>
            {
            };

            struct [[maybe_unused]] UnitExternWatchDog : public Input<>
            {
            };

            struct [[maybe_unused]] UnitExternalMinRecall : public Input<>
            {
            };

            struct [[maybe_unused]] UnitExternalStart : public Input<>
            {
            };

            struct [[maybe_unused]] UnitIndicatorLampControl : public Input<>
            {
            };

            struct [[maybe_unused]] UnitIntervalAdvance : public Input<>
            {
            };

            struct [[maybe_unused]] UnitIOModeBit_0 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitIOModeBit_1 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitIOModeBit_2 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitIOModeBit_3 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitITSLocalFlashSense : public Input<>
            {
            };

            struct [[maybe_unused]] UnitLocalFlash : public Input<>
            {
            };

            struct [[maybe_unused]] UnitLocalFlashSense : public Input<>
            {
            };

            struct [[maybe_unused]] UnitManualControlEnable : public Input<>
            {
            };

            struct [[maybe_unused]] UnitOffset_1 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitOffset_2 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitOffset_3 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSignalPlanA : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSignalPlanB : public Input<>
            {
            };

            struct [[maybe_unused]] UnitStopTIme : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_0 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_1 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_2 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_3 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_4 : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTBCHoldOnline : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTBCOnline : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTestInputA : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTestInputB : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTestInputC : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanA : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanB : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanC : public Input<>
            {
            };

            struct [[maybe_unused]] UnitTimingPlanD : public Input<>
            {
            };

            struct [[maybe_unused]] UnitWalkRestModifier : public Input<>
            {
            };

            template<fioidx_t I> requires (I >= 1) && (I <= controller_unit::detector::max_vehicle_detectors)
            struct [[maybe_unused]] VehicleDetCall : public Input<I>
            {
            };

            template<fioidx_t I> requires (I >= 1)
            struct [[maybe_unused]] VehicleDetReset : public Input<I>
            {
            };
        } // end of namespace field_io::input

    } // end of namespace field_io

    template<typename T> requires std::is_base_of_v<field_io::FieldIO, T> && T::io_category
    T fio;


} // end of namespace virtual_cabinet


#endif