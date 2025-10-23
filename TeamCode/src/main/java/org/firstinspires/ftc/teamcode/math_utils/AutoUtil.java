package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class AutoUtil {

    public enum AutoActionState {
        IDLE,
        RUNNING,
        FINISHED
    }


    private static BooleanSupplier isActive;

    private static AutoActionState[] loopStates;

    private static final ElapsedTime timer = new ElapsedTime();

    private static final ElapsedTime loopTimer = new ElapsedTime();

    private static double loopTime = 0.0;

    public static void supplyOpModeActive(BooleanSupplier opModeIsActive) {
        isActive = opModeIsActive;
    }

    /**
     * Runs a list of instantaneous methods in parallel.
     * Methods used must return an AutoActionState immediately after being called.
     * This type represents whether the last call of the method completed a desired action.
     * @param actions The list of methods to run, as an array of Callables.
     * @param utilMethods A secondary list of methods that do not necessarily return AutoActionStates.
     *                    These may be telemetry calls, update calls, etc.
     *                    They are run every loop until the actions are completed.
     * @param timeLimit The maximum time that the actions are allowed to run for.
     * @param timeLimitUnit The unit for the timeLimit.
     */
    public static void runActionsConcurrent(
            List<Callable<AutoActionState>> actions, List<Runnable> utilMethods,
            TimeUnit timeLimitUnit, double timeLimit) {

        if (actions.isEmpty()) return;

        //Reset Timer and Loop States
        timer.reset();
        loopStates = new AutoActionState[actions.size()];
        Arrays.fill(loopStates, AutoActionState.IDLE);
        //While the states are not all FINISHED, run each action in sequence. Then, run each
        //other method in sequence. If the time limit is reached, exit the method.
        while (!Arrays.stream(loopStates).allMatch(state -> state == AutoActionState.FINISHED) ) {
            if(!isActive.getAsBoolean()) return;
            loopTimer.reset();
            for (int i = 0; i < actions.size(); i++) {
                    if(!isActive.getAsBoolean()) return;
                    if (loopStates[i] != AutoActionState.FINISHED) {
                            try { loopStates[i] = actions.get(i).call(); }
                            catch (Exception e) { throw new RuntimeException(e); }
                    }
            }
            for (Runnable method: utilMethods) {
                method.run();
            }
            loopTime = loopTimer.milliseconds();
            if (timer.time(timeLimitUnit) >= timeLimit) { return; }

        }

    }

    /**
     * Continuously runs a set of methods (inputted as Runnables) for a set time interval.
     * @param methods The list of methods to run. Can be Auto Actions or anything else.
     * @param unit The unit that the time parameter is measured in.
     * @param time The time to run the loop for.
     */
    public static void runTimedLoop(List<Runnable> methods, TimeUnit unit, double time) {
        timer.reset();
        while (timer.time(unit) < time) {
            if (!isActive.getAsBoolean()) { return; }
            loopTimer.reset();
            for (Runnable method: methods) {
                method.run();
            }
            loopTime = loopTimer.milliseconds();
        }
    }

    /**
     * Returns an indexed multi-line readout of the AutoActionStates for the running actions.
     * @return The current state readout
     */
    public static String getLoopStatesReadout() {
        return toString(loopStates);
    }

    /**
     * Returns an indexed multi-line readout of a provided array of AutoActionStates
     * @param states The list of states to create the readout with
     * @return The state readout
     */
    public static String toString(AutoActionState[] states) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < states.length; i++) {
            builder.append(i)
                .append(": ")
                .append(toString(states[i]))
                .append(System.lineSeparator());
        }
        builder.append("Time Elapsed: ")
                .append(timer.seconds())
                .append(" sec");
        return builder.toString();
    }

    /**
     * Returns a string representation of an AutoActionState
     * @param state The state to be converted
     * @return The string representation
     */
    public static String toString(AutoActionState state) {
        switch (state) {
            case RUNNING:
                return "RUNNING";
            case FINISHED:
                return "FINISHED";
            default:
                return "IDLE";
        }
    }

    /**
     * Gets the AutoActionStates for the running actions.
     * @return The AutoActionStates
     */
    public static AutoActionState[] getLoopStates() { return loopStates; }

    public static double getLoopTime() { return loopTime; }

}
