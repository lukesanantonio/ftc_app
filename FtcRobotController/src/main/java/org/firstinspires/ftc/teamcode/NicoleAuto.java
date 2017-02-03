package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by luke on 1/31/17.
 */

public class NicoleAuto extends OpMode implements StateInjector {

    VVHardware hardware;
    VVAutoSensors sensors;
    VVTweaks tweaks;

    RobotFSM fsm;

    @Override
    public void onNewState(State state)
    {
        VVState vvState = (VVState) state;
        vvState.hardware = hardware;
        vvState.sensors = sensors;
        vvState.tweaks = tweaks;
    }

    @Override
    public void init() {
        // Initialize hardware
        hardware = new VVHardware(hardwareMap);
        sensors = new VVAutoSensors(hardwareMap);

        // Initialize state machine.
        fsm = new RobotFSM();
        fsm.addStateInjector(this);
        fsm.pushState(new VVStartState());
    }

    @Override
    public void loop() {
        fsm.run();
    }
}
