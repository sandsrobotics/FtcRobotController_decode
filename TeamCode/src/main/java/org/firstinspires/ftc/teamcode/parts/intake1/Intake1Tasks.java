package org.firstinspires.ftc.teamcode.parts.intake1;

import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1Settings;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake1Tasks {
    public final Group intakeTasksGroup;
    public final Group servoTasksGroup;

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

    public final TimedTask teleopFarRedLaunch;
    public final TimedTask teleopNearRedLaunch;
    public final TimedTask teleopGoalRedLaunch;
    public final TimedTask teleopThreeRedLaunch;

    public final TimedTask teleopFarBlueLaunch;
    public final TimedTask teleopNearBlueLaunch;
    public final TimedTask teleopGoalBlueLaunch;
    public final TimedTask teleopThreeBlueLaunch;
    public final TimedTask teleopMoveToRedLoadingZone;
    public final TimedTask teleopMoveToBlueLoadingZone;

    private final Intake1 intake;
    private final Robot robot;


    public Intake1Tasks(Intake1 intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        intakeTasksGroup = new Group("intake", intake.getTaskManager());
        servoTasksGroup = new Group("servo", intake.getTaskManager());

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
        teleopFarRedLaunch = new TimedTask(TaskNames.teleopFarRedLaunch, intakeTasksGroup);
        teleopNearRedLaunch = new TimedTask(TaskNames.teleopNearRedLaunch, intakeTasksGroup);
        teleopGoalRedLaunch = new TimedTask(TaskNames.teleopGoalRedLaunch, intakeTasksGroup);
        teleopThreeRedLaunch = new TimedTask(TaskNames.teleopThreeRedLaunch, intakeTasksGroup);
        teleopFarBlueLaunch = new TimedTask(TaskNames.teleopFarBlueLaunch, intakeTasksGroup);
        teleopNearBlueLaunch = new TimedTask(TaskNames.teleopNearBlueLaunch, intakeTasksGroup);
        teleopGoalBlueLaunch = new TimedTask(TaskNames.teleopGoalBlueLaunch, intakeTasksGroup);
        teleopThreeBlueLaunch = new TimedTask(TaskNames.teleopThreeBlueLaunch, intakeTasksGroup);
        teleopMoveToRedLoadingZone = new TimedTask(TaskNames.teleopMoveToRedLoadingZone, intakeTasksGroup);
        teleopMoveToBlueLoadingZone = new TimedTask(TaskNames.teleopMoveToBlueLoadingZone, intakeTasksGroup);

    }

    public void constructAllIntakeTasks() {
        /*     Motor Intake Task  */
        intakeTask.autoStart = false;
        intakeTask.addStep(()->{
            intakeTask.addStep(allServoDock::restart);
            intakeTask.addStep(allServoDock::isDone);
            intake.getHardware().intakeMotor.setPower(Intake1Settings.intakeIn);
            intake.getHardware().intakeServo.setPosition(Intake1Settings.intakeServoIn);
            intake.getHardware().augerMotor.setPower(Intake1Settings.augerMotorRun);
        });

        /*   Artifact Intake Stop Task   */
        artifactIntakeStopTask.autoStart = false;
        artifactIntakeStopTask.addStep(()-> {
            intake.getHardware().intakeMotor.setPower(Intake1Settings.intakeStop);
            intake.getHardware().intakeServo.setPosition(Intake1Settings.intakeServoOff);
            intake.getHardware().augerMotor.setPower(Intake1Settings.augerMotorStop);
        });
        artifactIntakeStopTask.addStep(allServoStore::restart);
        artifactIntakeStopTask.addStep(allServoStore::isDone);

         /*    Artifact outtake Task*/
        outtakeTask.autoStart = false;
        outtakeTask.addStep(() -> intake.getHardware().intakeMotor.setPower(Intake1Settings.intakeOut));
        // TODO: Check if this is needed.
//        outtakeTask.addStep(()-> intake.getHardware().intakeServo.setPosition(Intake1Settings.intakeServoOut));

        /*    Servo Transfer Tasks      */
        //         pink
        pinkServoLaunch.autoStart = false;
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkLaunch));
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.isDone());
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkDock));
        pinkServoLaunch.addStep(() -> intake.getHardware().pinkServo.isDone());
        //          blue
        blueServoLaunch.autoStart = false;
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueLaunch));
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.isDone());
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueDock));
        blueServoLaunch.addStep(() -> intake.getHardware().blueServo.isDone());
        //        green
        greenServoLaunch.autoStart = false;
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.setPosition(Intake1Settings.servoGreenLaunch));
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.isDone());
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.setPosition(Intake1Settings.servoGreenDock));
        greenServoLaunch.addStep(() -> intake.getHardware().greenServo.isDone());

        // all at once
        allServoLaunch.autoStart = false;
        allServoLaunch.addStep(()->{
//            pinkServoLaunch.restart();
//            blueServoLaunch.restart();
//            greenServoLaunch.restart();
            greenServoLaunch.restart();
            blueServoLaunch.restart();
            pinkServoLaunch.restart();

        });
//        allServoLaunch.addStep(pinkServoLaunch::isDone);
//        allServoLaunch.addStep(blueServoLaunch::isDone);
//        allServoLaunch.addStep(greenServoLaunch::isDone);
        allServoLaunch.addStep(greenServoLaunch::isDone);
        allServoLaunch.addStep(blueServoLaunch::isDone);
        allServoLaunch.addStep(pinkServoLaunch::isDone);

        // pink, blue, green, with 300 delay.
        pinkBlueGreenServoLaunch.autoStart = false;
        pinkBlueGreenServoLaunch.addStep(pinkServoLaunch::restart);
        pinkBlueGreenServoLaunch.addStep(pinkServoLaunch::isDone);
        pinkBlueGreenServoLaunch.addDelay(300);
        pinkBlueGreenServoLaunch.addStep(blueServoLaunch::restart);
        pinkBlueGreenServoLaunch.addStep(blueServoLaunch::isDone);
        pinkBlueGreenServoLaunch.addDelay(300);
        pinkBlueGreenServoLaunch.addStep(greenServoLaunch::restart);
        pinkBlueGreenServoLaunch.addStep(greenServoLaunch::isDone);
        pinkBlueGreenServoLaunch.addDelay(300);

        /*    Launch Tasks         */
        //    start Launch
        startAutoFarLaunch.autoStart = false;
        startAutoFarLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startAutoFarLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startAutoFarLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startAutoFarLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startAutoFarLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().autoFarLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().autoFarLaunchMotorVelocityStart);
        });
        startAutoFarLaunch.addStep(() -> intake.setLaunchRPM((int) Intake1Settings.autoFarLaunchMotorRPM));
        //%%%
        //startAutoFarLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.autoFarLaunchMotorRPM));
        startAutoFarLaunch.addTimedStep(() -> {}, intake::launchRPMInTolerance, 3000);

        //    start Launch
        startFarLaunch.autoStart = false;
        startFarLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startFarLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startFarLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startFarLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startFarLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().farLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().farLaunchMotorVelocityStart);
        });
        startFarLaunch.addStep(() -> intake.setLaunchRPM((int) Intake1Settings.farLaunchMotorRPM));
        //%%%
        //startFarLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.farLaunchMotorRPM));
        startFarLaunch.addTimedStep(() -> {}, intake::launchRPMInTolerance, 3000);

        startGoalLaunch.autoStart = false;
        startGoalLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startGoalLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startGoalLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startGoalLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startGoalLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().goalLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().goalLaunchMotorVelocityStart);
        });
        startGoalLaunch.addStep(() -> intake.setLaunchRPM((int) Intake1Settings.goalLaunchMotorRPM));
        //%%%
        //startGoalLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.goalLaunchMotorRPM));
        startGoalLaunch.addTimedStep(() -> {}, intake::launchRPMInTolerance, 3000);

        startThreeLaunch.autoStart = false;
        startThreeLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.yLaunchMotorRPM) intake.tasks.startYLaunch.restart();
        });
        startThreeLaunch.addTimedStep(()->{}, startYLaunch::isDone, 1000);
        startThreeLaunch.addStep(() -> {
            if (intake.getCurrentLaunchMotorRPM() < Intake1Settings.xLaunchMotorRPM) intake.tasks.startXLaunch.restart();
        });
        startThreeLaunch.addTimedStep(()->{}, startXLaunch::isDone, 1000);
        startThreeLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().threeLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().threeLaunchMotorVelocityStart);
        });
        startThreeLaunch.addStep(() -> intake.setLaunchRPM((int) Intake1Settings.threeLaunchMotorRPM));
        //%%%
        //startThreeLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.threeLaunchMotorRPM));
        startThreeLaunch.addTimedStep(() -> {}, intake::launchRPMInTolerance, 3000);


        startALaunch.autoStart = false;
        startALaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().aLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().aLaunchMotorVelocityStart);
        });
        //%%%
        //startALaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.aLaunchMotorRPM));
        startBLaunch.autoStart = false;
        startBLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().bLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().bLaunchMotorVelocityStart);
        });
        //%%%
        //startBLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.bLaunchMotorRPM));
        startYLaunch.autoStart = false;
        startYLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().yLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().yLaunchMotorVelocityStart);
        });
        //%%%
        //startYLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.yLaunchMotorRPM));
        startXLaunch.autoStart = false;
        startXLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().xLaunchMotorVelocityStart);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().xLaunchMotorVelocityStart);
        });
        //%%%
        //startXLaunch.addStep(() -> intake.setLaunchMotors((int) Intake1Settings.xLaunchMotorRPM));

        //     stop launch
        stopLaunch.autoStart = false;
        stopLaunch.addStep(() -> {
            intake.getHardware().launchMotorLeft.setVelocity(intake.getSettings().launchMotorVelocityStop);
            intake.getHardware().launchMotorRight.setVelocity(intake.getSettings().launchMotorVelocityStop);
        });
        //%%%
        //stopLaunch.addStep(() -> intake.setLaunchMotors((int) intake.getSettings().launchMotorVelocityStop));

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
        allServoStore.addStep(() -> intake.getHardware().pinkServo.isDone());
        allServoStore.addStep(() -> intake.getHardware().blueServo.isDone());

        //.         allServoDock
        allServoDock.autoStart = false;
        allServoDock.addStep(() ->{
            intake.getHardware().pinkServo.setPosition(Intake1Settings.servoPinkDock);
            intake.getHardware().blueServo.setPosition(Intake1Settings.servoBlueDock);
            intake.getHardware().greenServo.setPosition(Intake1Settings.servoGreenDock);
        });
        allServoDock.addStep(() -> intake.getHardware().pinkServo.isDone());
        allServoDock.addStep(() -> intake.getHardware().blueServo.isDone());
        allServoDock.addStep(() -> intake.getHardware().greenServo.isDone());


//        // teleopFarLaunch. -- Not working.
//        teleopFarLaunch.addStep(() -> {
//            intake.positionSolver.addMoveToTaskEx(
//                    DecodeSettings.isAllianceBlue() ?
//                            Intake1Settings.p_teleopFarBlueLaunch_1 :
//                            Intake1Settings.p_teleopFarRedLaunch_1,
//                    teleopFarLaunch);
//              });
//        teleopFarLaunch.addStep(startFarLaunch::restart);
//        teleopFarLaunch.addStep(startFarLaunch::isDone);
//        teleopFarLaunch.addStep(pinkServoLaunch::restart);
//        teleopFarLaunch.addStep(pinkServoLaunch::isDone);
//        teleopFarLaunch.addDelay(200);
//        teleopFarLaunch.addStep(() -> {
//            intake.positionSolver.addMoveToTaskEx(
//                    DecodeSettings.isAllianceBlue() ?
//                            Intake1Settings.p_teleopFarBlueLaunch_2 :
//                            Intake1Settings.p_teleopFarRedLaunch_2,
//                    teleopFarLaunch);
//        });
//        teleopFarLaunch.addStep(blueServoLaunch::restart);
//        teleopFarLaunch.addStep(blueServoLaunch::isDone);
//        teleopFarLaunch.addDelay(200);
//        teleopFarLaunch.addStep(() -> {
//            intake.positionSolver.addMoveToTaskEx(
//                    DecodeSettings.isAllianceBlue() ?
//                            Intake1Settings.p_teleopFarBlueLaunch_3 :
//                            Intake1Settings.p_teleopFarRedLaunch_3,
//                    teleopFarLaunch);
//        });
//        teleopFarLaunch.addStep(greenServoLaunch::restart);
//        teleopFarLaunch.addStep(greenServoLaunch::isDone);
//        teleopFarLaunch.addDelay(200);

        // teleopFarRedLaunch
        teleopFarRedLaunch.autoStart = false;
        teleopFarRedLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopFarRedLaunch_1, true));
        teleopFarRedLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopFarRedLaunch.addStep(startFarLaunch::restart);
        teleopFarRedLaunch.addStep(startFarLaunch::isDone);
        teleopFarRedLaunch.addStep(pinkServoLaunch::restart);
        teleopFarRedLaunch.addStep(pinkServoLaunch::isDone);
        teleopFarRedLaunch.addDelay(50);
//        teleopFarRedLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopFarRedLaunch_2, true));
//        teleopFarRedLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopFarRedLaunch.addStep(blueServoLaunch::restart);
        teleopFarRedLaunch.addStep(blueServoLaunch::isDone);
        teleopFarRedLaunch.addDelay(50);
//        teleopFarRedLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopFarRedLaunch_3, true));
//        teleopFarRedLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopFarRedLaunch.addStep(greenServoLaunch::restart);
        teleopFarRedLaunch.addStep(greenServoLaunch::isDone);
        teleopFarRedLaunch.addDelay(50);

        // teleopFarBlueLaunch
        teleopFarBlueLaunch.autoStart = false;
        teleopFarBlueLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopFarBlueLaunch_1, true));
        teleopFarBlueLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopFarBlueLaunch.addStep(startFarLaunch::restart);
        teleopFarBlueLaunch.addStep(startFarLaunch::isDone);
        teleopFarBlueLaunch.addStep(pinkServoLaunch::restart);
        teleopFarBlueLaunch.addStep(pinkServoLaunch::isDone);
        teleopFarBlueLaunch.addDelay(200);
        teleopFarBlueLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopFarBlueLaunch_2, true));
        teleopFarBlueLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopFarBlueLaunch.addStep(blueServoLaunch::restart);
        teleopFarBlueLaunch.addStep(blueServoLaunch::isDone);
        teleopFarBlueLaunch.addDelay(200);
        teleopFarBlueLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopFarBlueLaunch_3, true));
        teleopFarBlueLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopFarBlueLaunch.addStep(greenServoLaunch::restart);
        teleopFarBlueLaunch.addStep(greenServoLaunch::isDone);
        teleopFarBlueLaunch.addDelay(200);

//         teleopNearRedLaunch
        teleopNearRedLaunch.autoStart = false;
        teleopNearRedLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopNearRedLaunch, true));
        teleopNearRedLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopNearRedLaunch.addStep(startThreeLaunch::restart);
        teleopNearRedLaunch.addStep(startThreeLaunch::isDone);
        teleopNearRedLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopNearRedLaunch.addDelay(1500);

//         teleopNearBlueLaunch
        teleopNearBlueLaunch.autoStart = false;
        teleopNearBlueLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopNearBlueLaunch, true));
        teleopNearBlueLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopNearBlueLaunch.addStep(startThreeLaunch::restart);
        teleopNearBlueLaunch.addStep(startThreeLaunch::isDone);
        teleopNearBlueLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopNearBlueLaunch.addDelay(1500);

//         teleopGoalRedLaunch
        teleopGoalRedLaunch.autoStart = false;
        teleopGoalRedLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopGoalRedLaunch, true));
        teleopGoalRedLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopGoalRedLaunch.addStep(startGoalLaunch::restart);
        teleopGoalRedLaunch.addStep(startGoalLaunch::isDone);
        teleopGoalRedLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopGoalRedLaunch.addDelay(1500);

//         teleopGoalBlueLaunch
        teleopGoalBlueLaunch.autoStart = false;
        teleopGoalBlueLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopGoalBlueLaunch, true));
        teleopGoalBlueLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopGoalBlueLaunch.addStep(startGoalLaunch::restart);
        teleopGoalBlueLaunch.addStep(startGoalLaunch::isDone);
        teleopGoalBlueLaunch.addStep(pinkBlueGreenServoLaunch::restart);
        teleopGoalBlueLaunch.addDelay(1500);


        // teleopThreeRedLaunch
        teleopThreeRedLaunch.autoStart = false;
        teleopThreeRedLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopThreeRedLaunch, true));
        teleopThreeRedLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopThreeRedLaunch.addStep(startThreeLaunch::restart);
        teleopThreeRedLaunch.addStep(startThreeLaunch::isDone);
        teleopThreeRedLaunch.addStep(allServoLaunch::restart);
        teleopThreeRedLaunch.addDelay(500);

        // teleopThreeBlueLaunch
        teleopThreeBlueLaunch.autoStart = false;
        teleopThreeBlueLaunch.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopThreeBlueLaunch, true));
        teleopThreeBlueLaunch.addStep(() -> intake.positionSolver.isDone());
        teleopThreeBlueLaunch.addStep(startThreeLaunch::restart);
        teleopThreeBlueLaunch.addStep(startThreeLaunch::isDone);
        teleopThreeBlueLaunch.addStep(allServoLaunch::restart);
        teleopThreeBlueLaunch.addDelay(500);

        //  teleopMoveToRedLoadingZone
        teleopMoveToRedLoadingZone.autoStart = false;
        teleopMoveToRedLoadingZone.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopMoveToRedLoadingZone_1, true));
        teleopMoveToRedLoadingZone.addStep(() -> intake.positionSolver.isDone());
        teleopMoveToRedLoadingZone.addStep(intakeTask::restart);
        teleopMoveToRedLoadingZone.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopMoveToRedLoadingZone_2, true));
        teleopMoveToRedLoadingZone.addStep(() -> intake.positionSolver.isDone());
        teleopMoveToRedLoadingZone.addDelay(500);
        teleopMoveToRedLoadingZone.addStep(artifactIntakeStopTask::restart);

        //  teleopMoveToBlueLoadingZone
        teleopMoveToBlueLoadingZone.autoStart = false;
        teleopMoveToBlueLoadingZone.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopMoveToBlueLoadingZone_1, true));
        teleopMoveToBlueLoadingZone.addStep(() -> intake.positionSolver.isDone());
        teleopMoveToBlueLoadingZone.addStep(intakeTask::restart);
        teleopMoveToBlueLoadingZone.addStep(() -> intake.positionSolver.setNewTarget(Intake1Settings.p_teleopMoveToBlueLoadingZone_2, true));
        teleopMoveToBlueLoadingZone.addStep(() -> intake.positionSolver.isDone());
        teleopMoveToBlueLoadingZone.addDelay(500);
        teleopMoveToBlueLoadingZone.addStep(artifactIntakeStopTask::restart);

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
        public final static String teleopFarRedLaunch = "teleop FarRedLaunch";
        public final static String teleopNearRedLaunch = "teleop NearRedLaunch";
        public final static String teleopGoalRedLaunch = "teleop GoalRedLaunch";
        public final static String teleopThreeRedLaunch = "teleop ThreeRedLaunch";
        public final static String teleopFarBlueLaunch = "teleop FarBlueLaunch";
        public final static String teleopNearBlueLaunch = "teleop NearBlueLaunch";
        public final static String teleopGoalBlueLaunch = "teleop GoalBlueLaunch";
        public final static String teleopThreeBlueLaunch = "teleop ThreeBlueLaunch";
        public final static String teleopMoveToRedLoadingZone = "teleop MoveToRedLoadingZone";
        public final static String teleopMoveToBlueLoadingZone = "teleop MoveToBlueLoadingZone";

    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
