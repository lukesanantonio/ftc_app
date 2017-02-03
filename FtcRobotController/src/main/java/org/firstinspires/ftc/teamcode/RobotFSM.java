package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

/**
 * Created by luke on 1/31/17.
 */

public class RobotFSM {

    private Stack<State> stateStack;
    private List<StateInjector> stateInjectors;
    
    public RobotFSM()
    {
        stateStack = new Stack<>();
        stateInjectors = new ArrayList<>();
    }

    public void addStateInjector(StateInjector injector)
    {
        stateInjectors.add(injector);
    }

    public State popState() {
        return stateStack.pop();
    }

    public void replaceState(State state) {
        stateStack.pop();
        pushState(state);
    }

    public void pushState(State state)
    {
        // Give this state a reference to this fsm.
        state.fsm = this;
        // Inject any dependencies on the fly
        for(StateInjector injector : stateInjectors)
        {
            injector.onNewState(state);
        }

        // Do the push
        stateStack.push(state);
    }

    public void run() {
        stateStack.peek().run();
    }
}
