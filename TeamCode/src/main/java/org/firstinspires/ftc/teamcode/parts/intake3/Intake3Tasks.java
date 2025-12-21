package org.firstinspires.ftc.teamcode.parts.intake3;

//import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake3Tasks {
    protected final Group movementTask;
    private final TimedTask homeTask;
    public final TimedTask ballLaunchTask;
    public final TimedTask sameTimeBallLaunchTask;
    public final TimedTask moveAndLaunch;
    private final Intake3 intake;
    private final Robot robot;

    public Intake3Tasks(Intake3 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement", intake.getTaskManager());
        homeTask = new TimedTask(TaskNames.Home, movementTask);
        ballLaunchTask= new TimedTask(TaskNames.BallLaunch, movementTask);
        sameTimeBallLaunchTask = new TimedTask(TaskNames.sameTimeBallLaunch, movementTask);
        moveAndLaunch = new TimedTask(TaskNames.MoveAndLaunch, movementTask);
    }

    public void constructAllIntakeTasks() {
        homeTask.autoStart = false;
        /* ***** autoBallLaunchTask ******/
        ballLaunchTask.autoStart = false;
        ballLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.VIOLET.getLedPwm()));
        // launch all three balls one after the other
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.isDone());
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.isDone());
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.isDone());
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);

        // put all launch servos into rest position
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Rest));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Rest));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Rest));
        ballLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.GREEN.getLedPwm()));

        // all three ball task at the same time
        sameTimeBallLaunchTask.autoStart = false;
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.VIOLET.getLedPwm()));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Launch));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Launch));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Launch));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo0.isDone());
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo1.isDone());
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo2.isDone());
        sameTimeBallLaunchTask.addDelay(intake.getSettings().launchServoDelay);

        // put all launch servos into rest position
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Rest));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Rest));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Rest));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.GREEN.getLedPwm()));

        moveAndLaunch.autoStart = false;
        moveAndLaunch.addStep(() -> intake.positionSolver.setNewTarget(intake.launchData.getPosition(), true));
        moveAndLaunch.addStep(() -> intake.setLaunchRPM(intake.launchData.getRPM()));
        moveAndLaunch.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 4000);
        moveAndLaunch.addStep(ballLaunchTask::restart);
        moveAndLaunch.addStep(intake.tasks.ballLaunchTask::isDone);
        moveAndLaunch.addDelay(1000);
        moveAndLaunch.addStep(() -> intake.setLaunchRPM(0));
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String Home = "auto home";
        public final static String BallLaunch = "auto ball launch";
        public final static String sameTimeBallLaunch = "auto same time ball launch task";
        public final static String MoveAndLaunch = "auto move and launch";
    }
}
