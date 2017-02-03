package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.robot.Robot;

public abstract class State {

    public RobotFSM fsm;

    abstract void run();
}
