package org.firstinspires.ftc.teamcode.parts.intake3;

//import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;
import org.firstinspires.ftc.teamcode.parts.artifact.ArtifactDetectionPipeline;

public class Intake3Tasks {
    protected final Group movementTask;
    private final TimedTask homeTask;
    public final TimedTask ballLaunchTask;
    public final TimedTask sameTimeBallLaunchTask;
    public final TimedTask moveAndLaunch;
    public final TimedTask resetLaunchServos;
    public final TimedTask alignTarget;
    public final TimedTask orderedColorLaunchTask;
    public final TimedTask launchOneArtifact;
    private final Intake3 intake;
    private int[] currentLaunchOrder = null;  // Stores computed launch order
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
        orderedColorLaunchTask = new TimedTask(TaskNames.OrderedColorLaunch, movementTask);
        launchOneArtifact = new TimedTask(TaskNames.LaunchOneArtifact, movementTask);
    }

    public void constructAllIntakeTasks() {
        homeTask.autoStart = false;

        /* Begin */

        // launch all three balls one after the other
        ballLaunchTask.autoStart = false;

        //unlock all
        ballLaunchTask.addStep(()-> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Unlock));
        ballLaunchTask.addStep(()-> intake.getHardware().lockServo0.isDone());

        //launch
        ballLaunchTask.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo0.isDone());
        ballLaunchTask.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo1.isDone());
        ballLaunchTask.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Launch));
        ballLaunchTask.addStep(()-> intake.getHardware().launchServo2.isDone());
//        ballLaunchTask.addDelay(intake.getSettings().launchServoDelay);

        //launch reset
        ballLaunchTask.addStep(()-> resetLaunchServos.restart());

        /* End */

        launchOneArtifact.autoStart = false;
//        launchOneArtifact.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
//        launchOneArtifact.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Launch));
//        launchOneArtifact.addStep(()-> intake.getHardware().launchServo0.isDone());
//        launchOneArtifact.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);

        /* Begin */

        // all three ball task at the same time
        sameTimeBallLaunchTask.autoStart = false;

        //unlock all
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Unlock));
        sameTimeBallLaunchTask.addStep(()-> intake.getHardware().lockServo0.isDone());

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

// Ordered color launch
        orderedColorLaunchTask.autoStart = false;

// Unlock
        orderedColorLaunchTask.addStep(() -> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Unlock));
        orderedColorLaunchTask.addStep(() -> intake.getHardware().lockServo0.isDone());

// Compute order using the ONE method
        orderedColorLaunchTask.addStep(() -> {
            ArtifactDetectionPipeline.ArtifactColor[] desiredOrder = intake.limeLight.getClassificationPattern();
            currentLaunchOrder = intake.computeLaunchOrder(desiredOrder);
        });

// Ball 1
        orderedColorLaunchTask.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
        orderedColorLaunchTask.addStep(() -> {
            if (currentLaunchOrder != null && currentLaunchOrder.length > 0) {
                intake.launchServoByIndex(currentLaunchOrder[0]);
            }
        });
        orderedColorLaunchTask.addStep(() -> currentLaunchOrder != null && currentLaunchOrder.length > 0 ? intake.servoIsDoneByIndex(currentLaunchOrder[0]) : true);
        orderedColorLaunchTask.addDelay(intake.getSettings().launchServoDelay);

// Ball 2
        orderedColorLaunchTask.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
        orderedColorLaunchTask.addStep(() -> {
            if (currentLaunchOrder != null && currentLaunchOrder.length > 1) {
                intake.launchServoByIndex(currentLaunchOrder[1]);
            }
        });
        orderedColorLaunchTask.addStep(() -> currentLaunchOrder != null && currentLaunchOrder.length > 1 ? intake.servoIsDoneByIndex(currentLaunchOrder[1]) : true);
        orderedColorLaunchTask.addDelay(intake.getSettings().launchServoDelay);

// Ball 3
        orderedColorLaunchTask.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), intake.getSettings().launchRPMToleranceTime);
        orderedColorLaunchTask.addStep(() -> {
            if (currentLaunchOrder != null && currentLaunchOrder.length > 2) {
                intake.launchServoByIndex(currentLaunchOrder[2]);
            }
        });
        orderedColorLaunchTask.addStep(() -> currentLaunchOrder != null && currentLaunchOrder.length > 2 ? intake.servoIsDoneByIndex(currentLaunchOrder[2]) : true);
        orderedColorLaunchTask.addDelay(intake.getSettings().launchServoDelay);

// Reset
        orderedColorLaunchTask.addStep(() -> resetLaunchServos.restart());

        /* End */



        /* Begin */

        // move to given position and launch balls sequentially
        moveAndLaunch.autoStart = false;
        moveAndLaunch.addStep(() -> intake.positionSolver.setNewTarget(intake.launchData.getPosition(), true));
        moveAndLaunch.addStep(() -> intake.setLaunchRPM(intake.launchData.getRPM()));
        moveAndLaunch.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 4000);
        moveAndLaunch.addStep(orderedColorLaunchTask::restart);  // CHANGED THIS
        moveAndLaunch.addStep(orderedColorLaunchTask::isDone);   // CHANGED THIS
        moveAndLaunch.addStep(() -> intake.setLaunchRPM(0));

        /* End */

        /* Begin */

        // put all launch servos into rest position
        resetLaunchServos.autoStart = false;
        resetLaunchServos.addStep(()-> intake.getHardware().launchServo0.setPosition(intake.getSettings().launchServo0Rest));
        resetLaunchServos.addStep(()-> intake.getHardware().launchServo1.setPosition(intake.getSettings().launchServo1Rest));
        resetLaunchServos.addStep(()-> intake.getHardware().launchServo2.setPosition(intake.getSettings().launchServo2Rest));
        resetLaunchServos.addStep(()-> intake.getHardware().lockServo0.setPosition(intake.getSettings().lockServo0Lock));

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
        public final static String OrderedColorLaunch = "auto ordered color launch";
        public final static String AlignTarget = "auto align to april tag";
        public final static String LaunchOneArtifact = "auto launch just one ball by index";
    }
}
