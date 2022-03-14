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

namespace virtual_cabinet {

    namespace field_io {

        enum class State : unsigned char {
            off = 0, on = 1
        };

        class FieldIO {
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
            State m_state{State::off};
        };

        struct InputTag {
        };

        struct OutputTag {
        };

        template<unsigned char  = 0>
        struct Input : FieldIO {
            using io_catetory [[maybe_unused]] = InputTag;

            Input() = delete;
        };

        template<unsigned char index = 0>
        struct Output : FieldIO {
            using io_category = OutputTag;

            Output() = delete;
        };

        template<typename T> requires std::is_base_of_v<FieldIO, T> && T::io_category
        using IOCatetory = typename T::io_category;

        namespace output {

            struct [[maybe_unused]] AltFlashState : public Output<> {
            };

            struct [[maybe_unused]] AuxFunctionState : public Output<> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] ChannelGreenWalkDriver : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] ChannelRedDoNotWalkDriver : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] ChannelYellowRedClearDriver : public Output<index> {
            };

            struct [[maybe_unused]] CustomAlarm : public Output<> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] DetectorReset : public Output<index> {
            };

            struct [[maybe_unused]] FlashState : public Output<> {
            };

            struct [[maybe_unused]] GlobalVariable : public Output<> {
            };

            struct [[maybe_unused]] NotActive : public Output<> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] OverlapGreen : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] OverlapProtectedGreen : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] OverlapRed : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] OverlapYellow : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PedCall : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseAdvWarning : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseCheck : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseDoNotWalk : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseGreen : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseNext : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseOmit : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseOn : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhasePedClearance : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhasePreClear : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhasePreClear2 : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseRed : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseWalk : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseYellow : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptStatus : public Output<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptStatusFlash : public Output<index> {
            };

            struct [[maybe_unused]] RingStatusBitA : public Output<> {
            };

            struct [[maybe_unused]] RingStatusBitB : public Output<> {
            };

            struct [[maybe_unused]] RingStatusBitC : public Output<> {
            };

            struct [[maybe_unused]] RingStatusBitD : public Output<> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] SpecialFunction : public Output<index> {
            };

            struct [[maybe_unused]] UnitAutomaticFlash : public Output<> {
            };

            struct [[maybe_unused]] UnitFaultMonitor : public Output<> {
            };

            struct [[maybe_unused]] UnitFreeCoordStatus : public Output<> {
            };

            struct [[maybe_unused]] UnitOffset_1 : public Output<> {
            };

            struct [[maybe_unused]] UnitOffset_2 : public Output<> {
            };

            struct [[maybe_unused]] UnitOffset_3 : public Output<> {
            };

            struct [[maybe_unused]] UnitTBCAux_1 : public Output<> {
            };

            struct [[maybe_unused]] UnitTBCAux_2 : public Output<> {
            };

            struct [[maybe_unused]] UnitTBCAux_3 : public Output<> {
            };

            struct [[maybe_unused]] UnitTimingPlanA : public Output<> {
            };

            struct [[maybe_unused]] UnitTimingPlanB : public Output<> {
            };

            struct [[maybe_unused]] UnitTimingPlanC : public Output<> {
            };

            struct [[maybe_unused]] UnitTimingPlanD : public Output<> {
            };

            struct [[maybe_unused]] UnitVoltageMonitor : public Output<> {
            };

            struct [[maybe_unused]] Watchdog : public Output<> {
            };

        } // end of namespace field_io::output

        namespace input {

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] ChannelFaultStatus : public Input<index> {
            };

            struct [[maybe_unused]] CoordFreeSwitch : public Input<> {
            };

            struct [[maybe_unused]] CustomAlarm : public Input<> {
            };

            struct [[maybe_unused]] DoorAjor : public Input<> {
            };

            struct [[maybe_unused]] ManualControlGroupAction : public Input<> {
            };

            struct [[maybe_unused]] MinGreen_2 : public Input<> {
            };

            struct [[maybe_unused]] NotActive : public Input<> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] OverlapOmit : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PatternInput : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PedDetCall : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseForceOff : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhaseHold : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhasePedOmit : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PhasePhaseOmit : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptGateDown : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptGateUp : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptHighPrioritorLow : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptInput : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptInputCRC : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptInputNormalOff : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PreemptInputNormalOn : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PrioritorCheckIn : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PrioritorCheckOut : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] PrioritorPreemptDetector : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingForceOff : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingInhibitMaxTermination : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingMax2Selection : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingMax3Selection : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingOmitRedClearance : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingPedestrianRecycle : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingRedRest : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] RingStopTiming : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] SpecialFunctionInput : public Input<index> {
            };

            struct [[maybe_unused]] UnitAlarm_1 : public Input<> {
            };

            struct [[maybe_unused]] UnitAlarm_2 : public Input<> {
            };

            struct [[maybe_unused]] UnitAlternateSequenceA : public Input<> {
            };

            struct [[maybe_unused]] UnitAlternateSequenceB : public Input<> {
            };

            struct [[maybe_unused]] UnitAlternateSequenceC : public Input<> {
            };

            struct [[maybe_unused]] UnitAlternateSequenceD : public Input<> {
            };

            struct [[maybe_unused]] UnitAutomaticFlash : public Input<> {
            };

            struct [[maybe_unused]] UnitCallPedNAPlus : public Input<> {
            };

            struct [[maybe_unused]] UnitCallToNonActuated_1 : public Input<> {
            };

            struct [[maybe_unused]] UnitCallToNonActuated_2 : public Input<> {
            };

            struct [[maybe_unused]] UnitClockReset : public Input<> {
            };

            struct [[maybe_unused]] UnitCMUMMUFlashStatus : public Input<> {
            };

            struct [[maybe_unused]] UnitDimming : public Input<> {
            };

            struct [[maybe_unused]] UnitExternWatchDog : public Input<> {
            };

            struct [[maybe_unused]] UnitExternalMinRecall : public Input<> {
            };

            struct [[maybe_unused]] UnitExternalStart : public Input<> {
            };

            struct [[maybe_unused]] UnitIndicatorLampControl : public Input<> {
            };

            struct [[maybe_unused]] UnitIntervalAdvance : public Input<> {
            };

            struct [[maybe_unused]] UnitIOModeBit_0 : public Input<> {
            };

            struct [[maybe_unused]] UnitIOModeBit_1 : public Input<> {
            };

            struct [[maybe_unused]] UnitIOModeBit_2 : public Input<> {
            };

            struct [[maybe_unused]] UnitIOModeBit_3 : public Input<> {
            };

            struct [[maybe_unused]] UnitITSLocalFlashSense : public Input<> {
            };

            struct [[maybe_unused]] UnitLocalFlash : public Input<> {
            };

            struct [[maybe_unused]] UnitLocalFlashSense : public Input<> {
            };

            struct [[maybe_unused]] UnitManualControlEnable : public Input<> {
            };

            struct [[maybe_unused]] UnitOffset_1 : public Input<> {
            };

            struct [[maybe_unused]] UnitOffset_2 : public Input<> {
            };

            struct [[maybe_unused]] UnitOffset_3 : public Input<> {
            };

            struct [[maybe_unused]] UnitSignalPlanA : public Input<> {
            };

            struct [[maybe_unused]] UnitSignalPlanB : public Input<> {
            };

            struct [[maybe_unused]] UnitStopTIme : public Input<> {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_0 : public Input<> {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_1 : public Input<> {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_2 : public Input<> {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_3 : public Input<> {
            };

            struct [[maybe_unused]] UnitSystemAddressBit_4 : public Input<> {
            };

            struct [[maybe_unused]] UnitTBCHoldOnline : public Input<> {
            };

            struct [[maybe_unused]] UnitTBCOnline : public Input<> {
            };

            struct [[maybe_unused]] UnitTestInputA : public Input<> {
            };

            struct [[maybe_unused]] UnitTestInputB : public Input<> {
            };

            struct [[maybe_unused]] UnitTestInputC : public Input<> {
            };

            struct [[maybe_unused]] UnitTimingPlanA : public Input<> {
            };

            struct [[maybe_unused]] UnitTimingPlanB : public Input<> {
            };

            struct [[maybe_unused]] UnitTimingPlanC : public Input<> {
            };

            struct [[maybe_unused]] UnitTimingPlanD : public Input<> {
            };

            struct [[maybe_unused]] UnitWalkRestModifier : public Input<> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] VehicleDetCall : public Input<index> {
            };

            template<unsigned char index> requires (index > 0)
            struct [[maybe_unused]] VehicleDetReset : public Input<index> {
            };
        } // end of namespace field_io::input

    } // end of namespace field_io

} // end of namespace virtual_cabinet


#endif