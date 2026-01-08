package org.firstinspires.ftc.teamcode.parts.intake1;

import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1Settings;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake1Tasks {
    public final Group intakeTasksGroup;
    public final TimedTask intakeTask;
    public final TimedTask artifactIntakeStopTask;
    public final TimedTask outtakeTask;
    public final TimedTask pinkServoLaunch;
    public final TimedTask blueServoLaunch;
    public final TimedTask greenServoLaunch;
    public final TimedTask allServoLaunch;
    public final TimedTask startAutoFarLaunch;
    public final TimedTask startFarLaunch;
    public final TimedTask startGoalLaunch;
    public final TimedTask startThreeLaunch;
    public final TimedTask stopLaunch;
    public final TimedTask startALaunch;
    public final TimedTask startBLaunch;
    public final TimedTask startYLaunch;
    public final TimedTask startXLaunch;
    public final TimedTask viewObelisk;
    public final TimedTask computeAndLaunchInOrder;
    public final TimedTask allServoStore;
    public final TimedTask allServoDock;
    public final TimedTask pinkBlueGreenServoLaunch;

    public final TimedTask teleopFarLaunch;
    public final TimedTask teleopNearLaunch;
    public final TimedTask teleopGoalLaunch;
    public final TimedTask teleopThreeLaunch;

    private final Intake1 intake;
    private final Robot robot;


    public Intake1Tasks(Intake1 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        intakeTasksGroup = new Group("intake", intake.getTaskManager());
        intakeTask = new TimedTask(TaskNames.intakeTask, intakeTasksGroup);
        artifactIntakeStopTask = new TimedTask(TaskNames.artifactIntakeStop, intakeTasksGroup);
        outtakeTask = new TimedTask(TaskNames.outtakeTask, intakeTasksGroup);
        pinkServoLaunch = new TimedTask(TaskNames.pinkServoLaunch, intakeTasksGroup);
        blueServoLaunch = new TimedTask(TaskNames.blueServoLaunch, intakeTasksGroup);
        greenServoLaunch = new TimedTask(TaskNames.greenServoLaunch, intakeTasksGroup);
        allServoLaunch = new TimedTask(TaskNames.allServoLaunch, intakeTasksGroup);
        pinkBlueGreenServoLaunch = new TimedTask(TaskNames.pinkBlueGreenLaunch, intakeTasksGroup);
        startAutoFarLaunch = new TimedTask(TaskNames.startAutoFarLaunch, intakeTasksGroup);
        startFarLaunch = new TimedTask(TaskNames.startFarLaunch, intakeTasksGroup);
        startGoalLaunch = new TimedTask(TaskNames.startGoalLaunch, intakeTasksGroup);
        startThreeLaunch = new TimedTask(TaskNames.startThreeLaunch, intakeTasksGroup);
        stopLaunch = new TimedTask(TaskNames.stopLaunch, intakeTasksGroup);
        startALaunch = new TimedTask(TaskNames.startALaunch, intakeTasksGroup);
        startBLaunch = new TimedTask(TaskNames.startBLaunch, intakeTasksGroup);
        startYLaunch = new TimedTask(TaskNames.startYLaunch, intakeTasksGroup);
        startXLaunch = new TimedTask(TaskNames.startXLaunch, intakeTasksGroup);
        viewObelisk = new TimedTask(TaskNames.viewObelisk, intakeTasksGroup);
        computeAndLaunchInOrder = new TimedTask(TaskNames.computeAndLaunchInOrder, intakeTasksGroup);
        allServoStore = new TimedTask(TaskNames.allServoStore, intakeTasksGroup);
        allServoDock = new TimedTask(TaskNames.allServoDock, intakeTasksGroup);
        teleopFarLaunch = new TimedTask(TaskNames.teleopFarLaunch, intakeTasksGroup);
        teleopNearLaunch = new TimedTask(TaskNames.teleopNearLaunch, intakeTasksGroup);
        teleopGoalLaunch = new TimedTask(TaskNames.teleopGoalLaunch, intakeTasksGroup);
        teleopThreeLaunch = new TimedTask(TaskNames.teleopThreeLaunch, intakeTasksGroup);
    }

    public void constructAllIntakeTasks() {
        /*     Motor Intake Task  */
        intakeTask.autoStart = false;
        intakeTask.addStep(()->{
            intakeTask.addStep(allServoDock::restart);
            intakeTask.addStep(allServoDock::isDone);
            intake.getHardware().intakeMotor.setPower(Intake1Settings.intakeIn);
        });

        /*   Artifact Intake Stop Task   */
        artifactIntakeStopTask.autoStart = false;
        artifactIntakeStopTask.addStep(()-> {
            intake.getHardware().intakeMotor.setPower(Intake1Settings.intakeStop);
        });
        artifactIntakeStopTask.addStep(allServoStore::restart);
        artifactIntakeStopTask.addStep(allServoStore::isDone);

         /*    Artifact outtake Task*/
        outtakeTask.autoStart = false;
        outtakeTask.addStep(() -> intake.getHardware().intakeMotor.setPower(Intake1Settings.intakeOut));

        /*    Servo Transfer Tasks      */
        //         pink
        pinkServoLaunch.autoStart = false;
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkLaunch));
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.isDone());
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkDock));
        //          blue
        blueServoLaunch.autoStart = false;
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueLaunch));
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.isDone());
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueDock));
        //        green
        greenServoLaunch.autoStart = false;
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.setPosition(Intake1Settings.servoGreenLaunch));
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.isDone());
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.setPosition(Intake1Settings.servoGreenDock));

        // all at once
        allServoLaunch.autoStart = false;
        allServoLaunch.addStep(()->{
            pinkServoLaunch.restart();
            blueServoLaunch.restart();
            greenServoLaunch.restart();

        });
        allServoLaunch.addStep(() ->
            intake.getHardware().greenServo.isDone() && intake.getHardware().blueServo.isDone() && intake.getHardware().pinkServo.isDone()
        );

        // pink, blue, green, with 300 delay.
        pinkBlueGreenServoLaunch.autoStart = false;
        pinkBlueGreenServoLaunch.addStep(pinkServoLaunch::restart);
        pinkBlueGreenServoLaunch.addStep(pinkServoLaunch::isDone);
        pinkBlueGreenServoLaunch.addDelay(300);
        pinkBlueGreenServoLaunch.addStep(blueServoLaunch::restart);
        pinkBlueGreenServoLaunch.addStep(blueServoLaunch::isDone);
        pinkBlueGreenServoLaunch.addDelay(300);
        pinkBlueGreenServoLaunch.addStep(greenServoLaunch::restart);

        /*    Launch Tasks         */
        //    start Launch
        startAutoFarLaunch.autoStart = false;
        startAutoFarLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startAutoFarLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startAutoFarLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startAutoFarLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startAutoFarLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().autoFarLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().autoFarLaunchMotorVelocityStart);
        });

        //    start Launch
        startFarLaunch.autoStart = false;
        startFarLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startFarLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startFarLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startFarLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startFarLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().farLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().farLaunchMotorVelocityStart);
        });

        startGoalLaunch.autoStart = false;
        startGoalLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startGoalLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startGoalLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startGoalLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startGoalLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().goalLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().goalLaunchMotorVelocityStart);
        });

        startThreeLaunch.autoStart = false;
        startThreeLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startThreeLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startThreeLaunch.addStep(() -> {
            if (intake.getLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startThreeLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startThreeLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().threeLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().threeLaunchMotorVelocityStart);
        });

        startALaunch.autoStart = false;
        startALaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().aLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().aLaunchMotorVelocityStart);
        });
        startBLaunch.autoStart = false;
        startBLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().bLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().bLaunchMotorVelocityStart);
        });
        startYLaunch.autoStart = false;
        startYLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().yLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().yLaunchMotorVelocityStart);
        });
        startXLaunch.autoStart = false;
        startXLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().xLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().xLaunchMotorVelocityStart);
        });

        //     stop launch
        stopLaunch.autoStart = false;
        stopLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().launchMotorVelocityStop);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().launchMotorVelocityStop);
        });

        //     viewObelisk
        viewObelisk.autoStart = false;
        viewObelisk.addStep(() -> {
            DecodeSettings.setClassificationId(intake.limeLight.getClassificationId());
        });

        //     computeAndLaunchInOrder
        computeAndLaunchInOrder.autoStart = false;
        computeAndLaunchInOrder.addStep( () -> {
            int[] currLaunchOrder = intake.computeLaunchOrder(DecodeSettings.getClassificationId());

            for (int i = 0; i < 3; i++) {
                int currLaunch = currLaunchOrder[i];

                if (currLaunch == 0) {
                    intake.positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionZero(), computeAndLaunchInOrder);
                    computeAndLaunchInOrder.addStep(() -> intake.tasks.greenServoLaunch.restart());
                    computeAndLaunchInOrder.addDelay(300);
                } else if (currLaunch == 1) {
                    intake.positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionOne(), computeAndLaunchInOrder);
                    computeAndLaunchInOrder.addStep(() -> intake.tasks.blueServoLaunch.restart());
                    computeAndLaunchInOrder.addDelay(300);
                } else if (currLaunch == 2) {
                    intake.positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), computeAndLaunchInOrder);
                    computeAndLaunchInOrder.addStep(() -> intake.tasks.pinkServoLaunch.restart());
                    computeAndLaunchInOrder.addDelay(300);
                }
            }
        });

        //.         allServoStore
        allServoStore.autoStart = false;
        allServoStore.addStep(() ->{
            intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkLow);
            intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueLow);
        });
        //.         allServoDock
        allServoDock.autoStart = false;
        allServoDock.addStep(() ->{
            intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkDock);
            intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueDock);
            intake.getHardware().blueServo.setPosition(Intake1Settings.servoGreenDock);
        });

        // teleopFarLaunch
        teleopFarLaunch.autoStart = false;
        teleopFarLaunch.addStep(() -> {
            intake.positionSolver.addMoveToTaskEx(
                    DecodeSettings.isAllianceBlue() ?
                            intake.getSettings().p_teleopFarBlueLaunch :
                            intake.getSettings().p_teleopFarRedLaunch,
                    teleopFarLaunch);
              });
        teleopFarLaunch.addStep(startFarLaunch::restart);
        teleopFarLaunch.addStep(startFarLaunch::isDone);
        teleopFarLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopFarLaunch.addDelay(1500);

        // teleopNearLaunch
        teleopNearLaunch.autoStart = false;
        teleopNearLaunch.addStep(() -> {
            intake.positionSolver.addMoveToTaskEx(
                    DecodeSettings.isAllianceBlue() ?
                            intake.getSettings().p_teleopNearBlueLaunch :
                            intake.getSettings().p_teleopNearRedLaunch,
                    teleopNearLaunch);
        });
        teleopNearLaunch.addStep(startThreeLaunch::restart);
        teleopNearLaunch.addStep(startThreeLaunch::isDone);
        teleopNearLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopNearLaunch.addDelay(1500);

        // teleopGoalLaunch
        teleopGoalLaunch.autoStart = false;
        teleopGoalLaunch.addStep(() -> {
            intake.positionSolver.addMoveToTaskEx(
                    DecodeSettings.isAllianceBlue() ?
                            intake.getSettings().p_teleopGoalBlueLaunch :
                            intake.getSettings().p_teleopGoalRedLaunch,
                    teleopGoalLaunch);
        });
        teleopGoalLaunch.addStep(startGoalLaunch::restart);
        teleopGoalLaunch.addStep(startGoalLaunch::isDone);
        teleopGoalLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopGoalLaunch.addDelay(1500);

        // teleopThreeLaunch
        teleopThreeLaunch.autoStart = false;
        teleopThreeLaunch.addStep(() -> {
            intake.positionSolver.addMoveToTaskEx(
                    DecodeSettings.isAllianceBlue() ?
                            intake.getSettings().p_teleopThreeBlueLaunch :
                            intake.getSettings().p_teleopThreeRedLaunch,
                    teleopThreeLaunch);
        });
        teleopThreeLaunch.addStep(startThreeLaunch::restart);
        teleopThreeLaunch.addStep(startThreeLaunch::isDone);
        teleopThreeLaunch.addStep(allServoLaunch::restart);
        teleopThreeLaunch.addDelay(500);
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String intakeTask = "motor intake";
        public final static String artifactIntakeStop = "artifact intake stop";
        public final static String outtakeTask = "outtake Task";
        public final static String pinkServoLaunch = "pink servo Launch";
        public final static String blueServoLaunch = "blue servo Launch";
        public final static String greenServoLaunch = "green servo Launch";
        public final static String allServoLaunch = "all servos Launch";
        public final static String startAutoFarLaunch = "start AutoFarLaunch";
        public final static String startFarLaunch = "start FarLaunch";
        public final static String startGoalLaunch = "start GoalLaunch";
        public final static String startThreeLaunch = "start ThreeLaunch";
        public final static String stopLaunch = "stop Launch";
        public final static String startALaunch = "start A-Launch";
        public final static String startBLaunch = "start B-Launch";
        public final static String startYLaunch = "start Y-Launch";
        public final static String startXLaunch = "start X-Launch";
        public final static String viewObelisk = "view Obelisk";
        public final static String computeAndLaunchInOrder = "compute and LaunchInOrder";
        public final static String allServoStore = "all servos store";
        public final static String allServoDock = "all servos Dock";
        public final static String pinkBlueGreenLaunch = "pink green blue servo Launch";
        public final static String teleopFarLaunch = "teleop FarLaunch";
        public final static String teleopNearLaunch = "teleop NearLaunch";
        public final static String teleopGoalLaunch = "teleop GoalLaunch";
        public final static String teleopThreeLaunch = "teleop ThreeLaunch";

    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
