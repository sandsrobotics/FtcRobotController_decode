package org.firstinspires.ftc.teamcode.parts.intake3;

//import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake3Tasks {
    protected final Group movementTask;
    private final TimedTask homeTask;
    public final TimedTask ballLaunchTask;
    public final TimedTask sameTimeBallLaunchTask;
    public final TimedTask moveAndLaunch;
    public final TimedTask resetLaunchServos;
    public final TimedTask alignTarget;
    private final Intake3 intake;
    private final Robot robot;

    public Intake3Tasks(Intake3 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        movementTask = new Group("auto movement group", intake.getTaskManager());
        homeTask = new TimedTask(TaskNames.Home, movementTask);
        ballLaunchTask= new TimedTask(TaskNames.BallLaunch, movementTask);
        sameTimeBallLaunchTask = new TimedTask(TaskNames.sameTimeBallLaunch, movementTask);
        moveAndLaunch = new TimedTask(TaskNames.MoveAndLaunch, movementTask);
        resetLaunchServos = new TimedTask(TaskNames.ResetLaunch, movementTask);
        alignTarget = new TimedTask(TaskNames.AlignTarget, movementTask);
    }

    public void constructAllIntakeTasks() {
        homeTask.autoStart = false;

        /* Begin */

        // launch all three balls one after the other
        ballLaunchTask.autoStart = false;
        ballLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.VIOLET.getLedPwm()));

        //unlock all
        ballLaunchTask.addStep(()-> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Unlock));
        ballLaunchTask.addStep(()-> intake.getHardware().lockServo1.setPosition(intake.getSettings().lockServo1Unlock));
        ballLaunchTask.addStep(()-> intake.getHardware().lockServo2.setPosition(intake.getSettings().lockServo2Unlock));
        ballLaunchTask.addStep(()-> intake.getHardware().lockServo2.isDone());

        //launch
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.isDone());
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.isDone());
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.isDone());
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);

        //launch reset
        ballLaunchTask.addStep(()-> resetLaunchServos.restart());

        /* End */

        /* Begin */

        // all three ball task at the same time
        sameTimeBallLaunchTask.autoStart = false;
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.VIOLET.getLedPwm()));

        //unlock all
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Unlock));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().lockServo1.setPosition(intake.getSettings().lockServo1Unlock));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().lockServo2.setPosition(intake.getSettings().lockServo2Unlock));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().lockServo2.isDone());

        //launch
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Launch));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Launch));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Launch));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo0.isDone());
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo1.isDone());
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().launchServo2.isDone());

        //launch reset
        sameTimeBallLaunchTask.addStep(()-> resetLaunchServos.restart());

        /* End */

        /* Begin */

        // move to given position and launch balls sequentially
        moveAndLaunch.autoStart = false;
        moveAndLaunch.addStep(() -> intake.positionSolver.setNewTarget(intake.launchData.getPosition(), true));
        moveAndLaunch.addStep(() -> intake.setLaunchRPM(intake.launchData.getRPM()));
        moveAndLaunch.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 4000);
        moveAndLaunch.addStep(ballLaunchTask::restart);
        moveAndLaunch.addStep(intake.tasks.ballLaunchTask::isDone);
//        moveAndLaunch.addDelay(1000);
        moveAndLaunch.addStep(() -> intake.setLaunchRPM(0));

        /* End */

        /* Begin */

        // put all launch servos into rest position
        resetLaunchServos.autoStart = false;
        resetLaunchServos.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Rest));
        resetLaunchServos.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Rest));
        resetLaunchServos.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Rest));
        resetLaunchServos.addStep(()-> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Lock));
        resetLaunchServos.addStep(()-> intake.getHardware().lockServo1.setPosition(intake.getSettings().lockServo1Lock));
        resetLaunchServos.addStep(()-> intake.getHardware().lockServo2.setPosition(intake.getSettings().lockServo2Lock));
        resetLaunchServos.addStep(()-> intake.getHardware().pixel.setPosition(Intake3.LEDColor.GREEN.getLedPwm()));

        /* End */

        /* Begin */

        alignTarget.autoStart = false;
        alignTarget.addTimedStep(() -> IntakeSettings3.alignTarget = true, () -> intake.isAligned() || !intake.limeLight.tv, 2000);
        alignTarget.addStep(()-> IntakeSettings3.alignTarget = false);

        /* End */
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String Group = "auto group";
        public final static String Home = "auto home";
        public final static String BallLaunch = "auto ball launch";
        public final static String sameTimeBallLaunch = "auto same time ball launch task";
        public final static String MoveAndLaunch = "auto move and launch";
        public final static String ResetLaunch = "auto reset launch servos";
        public final static String AlignTarget = "auto align to april tag";
    }
}
