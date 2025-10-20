package org.firstinspires.ftc.teamcode.depricated.opmodes;

import static om.self.ezftc.utils.Constants.tileSide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

@Config
@Disabled
@Autonomous (name="14273.1 BUCKET", group="14273")
public class FlipBucketAuto2025 extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    PositionSolver positionSolver;
    Pinpoint odo;
    Intake intake;
    /**************************/
    public int startDelay;
    //  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    PositionTracker pt;
    Vector3 startPosition;
    static public int longDelay = 3000;
    public static int maxDelay = 3000;
    private int parkPosition;
    public long startTime;
    public double samplePosFish = -11;
    public int modelOfSub = 7;


    public void initAuto(){
        transformFunc = (v) -> v;
    }

    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    private Vector3 tileToInchAutoNoZ(Vector3 tiles){ return Constants.tileToInch(transformFunc.apply(tiles)).withZ(tiles.Z); }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    @Override
    public void runOpMode() {
        FlipbotSettings.setAuto();
        long start;
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);
        intake = new Intake(robot);

//        Vector3 fieldStartPos = new Vector3(0,0,-90);
//        Vector3 fieldStartPos = new Vector3(14 + 3.0/8.0, -62, -90);
        Vector3 fieldStartPos = new Vector3(-14.375 - 23.5, -62, 90);
        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        odo = new Pinpoint(pt, true, "pinpoint",
                -56.0, 52.0, 13.26291192f,
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        DecimalFormat df = new DecimalFormat("#0.0");

        robot.init();

        while (!isStarted()) {
            robot.buttonMgr.runLoop();

            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped)) {
                FlipbotSettings.isBlueGood = true;
                FlipbotSettings.isRedGood = false;
            }

            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.b, ButtonMgr.State.wasTapped)) {
                FlipbotSettings.isBlueGood = false;
                FlipbotSettings.isRedGood = true;
            }

            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_left, ButtonMgr.State.wasTapped)) {
                samplePosFish = samplePosFish - 1;
                modelOfSub = modelOfSub-1;
            }

            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_right, ButtonMgr.State.wasTapped)) {
                samplePosFish = samplePosFish + 1;
                modelOfSub++;
            }
            // example configuration capability during init

//            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasTapped)) {
//                FlipbotSettings.autonomousDebugMode = !FlipbotSettings.autonomousDebugMode;   //todo: disable this before competition!
//            }
//            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper, ButtonMgr.State.wasTapped)) {
//                startDelay += 1000;
//            }
//            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.left_bumper, ButtonMgr.State.wasTapped)) {
//                startDelay -= 1000;
//                if(startDelay < 0) startDelay = 0;
//            }
//
//            telemetry.addData("DEBUG?:", FlipbotSettings.autonomousDebugMode ? "***** YES *****" : "No, normal");
//            telemetry.addData("PARK POSITION:", parkPosition == 0 ? "Normal mid wall" : parkPosition == 1 ? "Park MID" : parkPosition == 2 ? "Park CORNER" : "Park BOARD");
//            telemetry.addData("START DELAY:", startDelay / 1000);
            telemetry.addData("SamplePos", samplePosFish);
            telemetry.addData("Red Side", FlipbotSettings.isRedGood);
            telemetry.addData("Blue Side", FlipbotSettings.isBlueGood);

            StringBuilder tempString = new StringBuilder();
            for (int i=0; i< modelOfSub; i++) {
                tempString.append(" ");
            }
            tempString.append("*");

            telemetry.addLine("      " +tempString);
            telemetry.addLine("|---------------------------------------------|");
            telemetry.update();
            sleep(50);
        }

        odo.setPosition(fieldStartPos);
        robot.start();
        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        // Setting up group container, task queue, and setting positionSolver target
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);
        //positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        // Here is where we schedule the tasks for the autonomous run (testAuto function below run loop)
        testNewBucketAuto(autoTasks);
        startTime = System.currentTimeMillis();

        // somewhere after init and before the main run loop should pop the hang servo up!
        //intake.getHardware().hang.setPosition(intake.getSettings().hangServoPreUp);  // or whatever position is correct

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            FlipbotSettings.storeRobotPosition(pt.getCurrentPosition());
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("time", System.currentTimeMillis() - start);
            dashboardTelemetry.update();
            telemetry.update();
        }
        robot.stop();
    }

    private void testBucketAuto(TimedTask autoTasks) {

        // New settings
        Vector3 posBucketSideStart = new Vector3(-14.375, -62, 90);
        Vector3 posBucketScore = new Vector3(-53.5, -53.5, 45);
        Vector3 posSample1 = new Vector3(-36, -39, 135);
        Vector3 posSample2 = new Vector3(-46.5, -39, 135);
        Vector3 posSample3 = new Vector3(-59, -42, 119);
        Vector3 posPark = new Vector3(-23.5, -11, 180);
        Vector3 posPrePark = new Vector3(-39, -11, 180);

        // TODO: Rename the positions to be  more descriptive.
        Vector3 p_1 = new Vector3(-14.375 - 23.5, -62, 90);
        Vector3 p_2 = new Vector3(-14.375 - 23.5, -52, 90);
        Vector3 p_pre_3 = new Vector3(-53.5, -53.5, 90);
        Vector3 p_3 = new Vector3(-53.5, -53.5, 45);
        Vector3 p_4 = new Vector3(-38, -48, 90);
        Vector3 p_pre_5 = new Vector3(-36, -39, 45);
        Vector3 p_5 = new Vector3(-36, -39, 135);
        Vector3 new_p5 = new Vector3(-56.5, -42.5, 66);
        Vector3 p_pre_6 = new Vector3(-46.5, -39, 45);
        Vector3 p_6 = new Vector3(-46.5, -39, 135);
        Vector3 new_p6 = new Vector3(-51,-42.5,115);
        Vector3 p_7 = new Vector3(-58, -42, 119); //X:-59
        Vector3 p_8 = new Vector3(-39, -11, 0);
        Vector3 p_9 = new Vector3(-23.5, -11, 0);

        // End New settings
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addStep(() -> odo.setPosition(p_1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        autoTasks.addStep(() -> intake.getHardware().hang.setPosition(intake.getSettings().hangServoPreUp));

        {
            // Deposit Pre-loaded Sample in High-basket
            autoTasks.addStep(() -> intake.getHardware().hang.setPosition(intake.getSettings().hangServoSafe));
            autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftMax, 1));
            positionSolver.addMoveToTaskEx(p_2, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
            positionSolver.addMoveToTaskEx(p_3, autoTasks);
            autoTasks.addStep(() -> intake.tasks.depositTask.restart());
            autoTasks.addStep(() -> intake.tasks.depositTask.isDone());
        }

        // First Sample.
        grabAndDepositSample(autoTasks, p_pre_5, new_p5, p_3);

        // Second Sample.
        grabAndDepositSample(autoTasks, p_pre_6, new_p6, p_3);

        // Third Sample.
        grabAndDepositSample(autoTasks, p_6, p_7, p_3);

        //Fishing for samples
        intakeSampleForAuto(autoTasks, p_3, p_8, p_9);

        // ParK for AutoAscent.
        parkForAutoAscent(autoTasks, p_8, p_9);

    }

    private void grabAndDepositSample (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));

        // Grab Sample.
        autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor)); //flipperAlmostFloor));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());

        // Deposit Sample in High-Basket.
        positionSolver.addMoveToTaskEx(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.transferTask.isDone());
        autoTasks.addStep(() -> intake.tasks.depositTask.restart());
        autoTasks.addStep(() -> intake.tasks.depositTask.isDone());
    }

    private void grabAndDepositSampleTest (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        // Grab Sample.
        autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor)); //flipperAlmostFloor));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());

        // Deposit Sample in High-Basket.
        positionSolver.addMoveToTaskExNoWait(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.transferTask.isDone());
        autoTasks.addStep(() -> intake.tasks.depositTask.restart());

        autoTasks.addStep(() -> intake.tasks.depositTask.isDone());
    }
    private void grabAndDepositSampleTestForMultitasking (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three, Vector3 pos_four) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        // Grab Sample.
        autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor)); //flipperAlmostFloor));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addDelay(750);
   //     autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());

        // Deposit Sample in High-Basket.
        positionSolver.addMoveToTaskExNoWait(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.transferTask.isDone());
        positionSolver.addMoveToTaskEx(pos_four, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoDepositTask.restart());
   //     autoTasks.addDelay(650);

        autoTasks.addStep(() -> intake.tasks.autoDepositTask.isDone());
    }
    private void grabAndDepositSampleTwoTest (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three, Vector3 pos_four) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        // Grab Sample.
        autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor));//flipperAlmostFloor));
        positionSolver.setSettings(PositionSolverSettings.slowSettings);
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        positionSolver.setSettings(PositionSolverSettings.slowSettings);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addDelay(500);
//        positionSolver.addMoveToTaskEx(pos_four, autoTasks);
//        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());

        // Deposit Sample in High-Basket.
        positionSolver.addMoveToTaskExNoWait(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.transferTask.isDone());
        autoTasks.addStep(() -> intake.tasks.autoDepositTask.restart());
        autoTasks.addStep(() -> intake.tasks.autoDepositTask.isDone());
    }

    private void grabAndDepositSampleOneTest (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three, Vector3 pos_four) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));

        // Grab Sample.
        autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor)); //flipperAlmostFloor));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(pos_four, autoTasks);
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());

        // Deposit Sample in High-Basket.
        autoTasks.addStep(() -> intake.tasks.transferTask.isDone());
        positionSolver.addMoveToTaskExNoWait(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoDepositTask.restart());
        autoTasks.addStep(() -> intake.tasks.autoDepositTask.isDone());
    }

    private void grabAndDepositSampleThreeTest (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two, Vector3 pos_three, Vector3 pos_four) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));

        // Grab Sample.
        autoTasks.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor)); //flipperAlmostFloor));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addDelay(500);
        positionSolver.addMoveToTaskEx(pos_four, autoTasks);
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());

        // Deposit Sample in High-Basket.
//        autoTasks.addStep(() -> intake.tasks.transferTask.isDone());
//        autoTasks.addStep(() -> {if (intake.isSamplePresent()) {
//            positionSolver.addMoveToTaskExNoWait(pos_three, autoTasks);
//            autoTasks.addStep(() -> intake.tasks.autoDepositTask.restart());
//            autoTasks.addStep(() -> intake.tasks.autoDepositTask.isDone());
//        }});
        positionSolver.addMoveToTaskEx(pos_three, autoTasks);
        autoTasks.addStep(() -> intake.tasks.autoDepositSample3.restart());
        autoTasks.addStep(() -> intake.tasks.autoDepositSample3.isDone());
    }

    private void parkForAutoAscent (TimedTask autoTasks, Vector3 pos_one, Vector3 pos_two) {
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(pos_one, autoTasks);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTasks.addStep(() -> intake.getHardware().park.setPosition(intake.getSettings().parkUp));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
    }

    private void intakeSampleForAuto (TimedTask autoTasks, Vector3 pos_three, Vector3 pos_two, Vector3 pos_one) {

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        //Move to submersible

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(pos_two, autoTasks);
        autoTasks.addStep(() -> intake.getHardware().park.setPosition(intake.getSettings().parkUp));  // lk moved up
        positionSolver.addMoveToTaskEx(pos_one, autoTasks);
        //Go Fishing
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
        autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());
        //        if (intake.sampleInIntake()) {
//            //autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.restart());
////            autoTasks.addStep(() -> intake.tasks.autonomousSampleTask.isDone());
//            autoTasks.addStep(() -> intake.tasks.prepareToDepositTask.restart());
//            autoTasks.addStep(() -> intake.tasks.prepareToDepositTask.isDone());
//            autoTasks.addStep(() -> intake.tasks.autoDepositTask.restart());
//            autoTasks.addStep(() -> intake.tasks.depositTask.isDone());
//        }

    }

    private void autoIntakeDeposit (TimedTask autoTasks,Vector3 pos_one) {
//        autoTasks.addStep(() -> intake.tasks.prepareToDepositTask.restart());
//        autoTasks.addStep(() -> intake.tasks.prepareToDepositTask.isDone());
        positionSolver.addMoveToTaskEx(pos_one, autoTasks);
        autoTasks.addStep(() -> intake.tasks.depositTask.restart());
        autoTasks.addStep(() -> intake.tasks.depositTask.isDone());


    }


    private void testNewBucketAuto(TimedTask autoTasks) {
        //positions
        Vector3 posBucketSideStart = new Vector3(-14.375, -62, 90);
        Vector3 posBucketScore = new Vector3(-53.5, -53.5, 45);
        Vector3 posSample1 = new Vector3(-58.75, -50, 67);
        Vector3 posSample1Random = new Vector3(-58.75, -46, 67);
        Vector3 posSample2 = new Vector3(-58.5, -53.5, 90);
        Vector3 posSample3Random = new Vector3(-58.77, -44.5, 112);
        Vector3 posSample3 = new Vector3(-59, -50, 112);
        Vector3 posPark = new Vector3(-23.5 ,samplePosFish , 0);
        Vector3 posPrePark = new Vector3(-39, samplePosFish , 0);

        // TODO: Rename the positions to be  more descriptive.
        Vector3 p_1 = new Vector3(-14.375 - 23.5, -62, 90);
        Vector3 p_2 = new Vector3(-14.375 - 23.5, -52, 90);
        Vector3 p_pre_3 = new Vector3(-53.5, -53.5, 90);
        Vector3 p_3 = new Vector3(-53.5, -53.5, 45);
        Vector3 p_4 = new Vector3(-59.75, -50, 86);
        Vector3 p_pre_5 = new Vector3(-36, -39, 45);
        Vector3 p_5 = new Vector3(-36, -39, 135);
        Vector3 new_p5 = new Vector3(-56.5, -42.5, 66);
        Vector3 p_pre_6 = new Vector3(-46.5, -39, 45);
        Vector3 p_6 = new Vector3(-46.5, -39, 135);
        Vector3 new_p6 = new Vector3(-51,-42.5,115);
        Vector3 p_7 = new Vector3(-58, -42, 119); //X:-59
        Vector3 p_8 = new Vector3(-39, -11, 0);
        Vector3 p_9 = new Vector3(-23.5, -11, 0);

        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        startTime = System.currentTimeMillis();
        autoTasks.addStep(() -> odo.setPosition(p_1));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));

        {
            // Deposit Pre-loaded Sample in High-basket
            autoTasks.addStep(() -> intake.getHardware().hang.setPosition(intake.getSettings().hangServoSafe));
            autoTasks.addDelay(200);
            autoTasks.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftMax, 1));
            positionSolver.addMoveToTaskEx(p_2, autoTasks);
            autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
            positionSolver.addMoveToTaskEx(posSample1, autoTasks);
            autoTasks.addStep(() -> intake.tasks.autoDepositTask.restart());
            grabAndDepositSampleOneTest(autoTasks, p_pre_3, posSample1, posSample1, posSample1Random);
            autoTasks.addStep(() -> intake.tasks.autoDepositTask.isDone());
            autoTasks.addStep(() -> intake.tasks.dockTask.restart());
        }

//        grabAndDepositSampleTestForMultitasking(autoTasks, p_pre_5, posSample1, posSample1);

        grabAndDepositSampleTwoTest(autoTasks, p_pre_5, posSample2, posSample1, posSample3Random);

        grabAndDepositSampleThreeTest(autoTasks, p_pre_3, posSample3, posSample1, posSample3Random);

        autoTasks.addStep(() -> intake.tasks.dockTask.restart());

        intakeSampleForAuto(autoTasks, p_3, posPrePark, posPark);
        autoTasks.addStep(() ->{
            if (((System.currentTimeMillis() - startTime) / 1000) < (30-26)) {
                autoIntakeDeposit(autoTasks, p_3);

            }});

        parkForAutoAscent(autoTasks, posPrePark, posPark);


    }
}