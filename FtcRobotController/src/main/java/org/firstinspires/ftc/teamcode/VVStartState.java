package org.firstinspires.ftc.teamcode;

/**
 * Created by luke on 1/31/17.
 */


// Some states can be broken down into actions (motor control) and conditions
// (that trigger the next state)

class DistanceThresholdCondition extends VVState {
    State wrappedState;
    public DistanceThresholdCondition(State wrap)
    {
        wrappedState = wrap;
    }

    public void run()
    {
        // Do we really want to wrap states? Or should we use the existing stack
        // of stacks. For example, by having a function callback for when a \
        // state is popped and pushed. When a state is pushed it will also push
        // its condition when that one is popped (or run, if we choose to only
        // run the top-most state on the stack) that state can automatically
        // pop itself.
        wrappedState.run();
    }
}

class ForwardState extends VVState {
    public void run() {
    }
}

public class VVStartState extends VVState {
    public void run() {
        //fsm.replaceState(new StraightenState());
    }
}
