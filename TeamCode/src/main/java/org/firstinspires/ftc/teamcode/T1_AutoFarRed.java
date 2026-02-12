package org.firstinspires.ftc.teamcode;

import static om.self.ezftc.utils.Constants.tileSide;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;
import org.firstinspires.ftc.teamcode.parts.intake1.Intake1;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

@Config
@Autonomous (name="14273.4 AutoFarRed", group="14273")
public class T1_AutoFarRed  extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    PositionSolver positionSolver;
    PositionTracker pt;
    Vector3 startPosition;
    Pinpoint odo;
    Intake1 intake;
    Artifacts artifacts;
    LimeLight limelight;

    Integer launchRPM = 3200;

    static public int runSpikeCount = 2; // Default to 2 spikes
    static public int runLeverOpen = 0; // Default to false.

    // Positions to travel in AutoFarRed
    Vector3 p_targetGoal                 = new Vector3(-70.5, 70.5, 180);   // RedGoal Position.
    Vector3 p_fieldStart                 = new Vector3(64,16,180);
    Vector3 p_obeliskView               = new Vector3(58, 16, 180);  // Was: 56, 16, 180; // FarRed: ObeliskView Position
    Vector3 p_launchPosZero             = new Vector3(58,16,157);    // Was: 56, 16, 153; // FarRed Launching Position.
    Vector3 p_launchPosOne              = new Vector3(58,16,157);    // Was: 56, 16, 153; // FarRed Launching Position.
    Vector3 p_launchPosTwo              = new Vector3(58,16,160);    // Z: 160; FarRed Launching Position for pinkServo. Z:160.

    Vector3 p_pre_intakeArtifactRow1 = new Vector3(-12, 28, -90);  // Was Y: 28; Red: Ready to collect on Row1
    Vector3 p_intakeArtifactRow1     = new Vector3(-12, 53, -90);  // Red: Intake Artifacts in Row1
    Vector3 p_pre_intakeArtifactRow2 = new Vector3(14, 28, -90);   // Was Y:28; Red: Ready to collect on Row2
    Vector3 p_intakeArtifactRow2     = new Vector3(14, 60, -90);   // Red: Intake Artifacts in Row2
    Vector3 p_pre_intakeArtifactRow3 = new Vector3(35.5, 28, -90);   // Was Y:28; Red: Ready to collect in Row3
    Vector3 p_intakeArtifactRow3     = new Vector3(35.5, 60, -90);   // Red: Intake Artifacts in Row3

    Vector3 p_leverOpen                 = new Vector3(0, 55, 180);    // Red: Open Lever Position
    Vector3 p_parkAfterAuto             = new Vector3(46,16,157);    // X:32;

    //  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    public static int maxDelay = 10000;
    /**************************/
    public int startDelay = 0;
    public long startTime;

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
        extraSettings();
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);
        limelight = new LimeLight(robot);
        artifacts = new Artifacts(robot);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), DecodeSettings.getRobotPosition());
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));

        odo = new Pinpoint(pt, false, "odo",
                DecodeSettings.pinpointSettingsXoffset, DecodeSettings.pinpointSettingsYoffset, DecodeSettings.pinpointSettingsResolution,
                GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        positionSolver.setSettings(PositionSolverSettings.defaultSettings);

        intake = new Intake1(robot);
        robot.init();

//        intake.launchRPM = (int) Intake1Settings.autoFarLaunchMotorRPM;

        long timer = System.currentTimeMillis() + 2500;
        while (odo.getValidPosition() == null && System.currentTimeMillis() <= timer) {
            telemetry.addLine("Waiting for Pinpoint...");
            telemetry.update();
            //todo: What to do if it doesn't initialize?
        }
        try {
            odo.setPosition(DecodeSettings.getRobotPosition());
        } catch (Exception e) {
            telemetry.addLine("Exception while odo.setPosition; Ignoring");
            telemetry.update();
        }
        DecodeSettings.odoFirstRun = false;

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
            telemetry.addData("CurrOpMode", DecodeSettings.getCurrentOpMode());
            telemetry.addData("START_P", odo.getPosition());
            // LED color indicator
            if (DecodeSettings.isAllianceRed()) {
                intake.getHardware().ledServo.setPosition(0.279); // Red (0.279),
            } else {
                intake.getHardware().ledServo.setPosition(0.611); //Blue (0.611),
            }
            telemetry.addData("Alliance Color",(DecodeSettings.isAllianceRed()?"Red":"Blue"));

            telemetry.addData("D-PAD UP/DOWN to change spike count","");
            // D-pad controls for spike count
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasTapped))
            {
                runSpikeCount = Math.min(3, runSpikeCount + 1);
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasTapped))
            {
                runSpikeCount = Math.max(1, runSpikeCount - 1);
            }
            telemetry.addData("             Spikes to run", runSpikeCount);

            telemetry.addData("D-PAD LEFT / RIGHT to disable/enable Lever Open","");
            // D-pad controls for LeverOpen
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_left, ButtonMgr.State.wasTapped))
            {
                runLeverOpen = 0;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_right, ButtonMgr.State.wasTapped))
            {
                runLeverOpen = 1;
            }
            telemetry.addData("             Lever Open:", (runLeverOpen == 1) ? "Enabled" : "Disabled");

            telemetry.addData("RB (+) / LB (-) to change Start Delay","");

            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper, ButtonMgr.State.wasTapped)) {
                startDelay += 1000;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.left_bumper, ButtonMgr.State.wasTapped)) {
                startDelay -= 1000;
                if(startDelay < 0) startDelay = 0;
            }
            if(startDelay > maxDelay) startDelay = maxDelay;

            telemetry.addData("             START DELAY (Seconds):", startDelay / 1000);
            telemetry.update();
            sleep(50);
        }

        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        // Setting up group container, task queue, and setting positionSolver target
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);
        //positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        if (startDelay > 0) {
            autoTasks.addDelay(startDelay);
        }

        // Here is where we schedule the tasks for the autonomous run (testNewAuto function below run loop)
        // testNewAuto - Successfully Tested!
        testNewAuto(autoTasks);

        startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            robot.run();
            DecodeSettings.setRobotPosition(pt.getCurrentPosition());
            telemetry.addData("Position", odo.getPosition());
            telemetry.addData("Current Launch Motor RPM", intake.getCurrentLaunchMotorRPM());
            telemetry.addData("ClassificationId", DecodeSettings.getClassificationId());
            telemetry.addData("time", System.currentTimeMillis() - startTime);
            if (startDelay > 0)
                telemetry.addData("Delayed Start", startDelay - (System.currentTimeMillis() - startTime));
            telemetry.update();
        }
        robot.stop();
    }

    // testNewAuto
    //     Look at Obelisk, Determine Game Classification Pattern and Store it.
    //     Launch Pre-Loaded Artifacts.
    //     Intake and Launch Artifacts in Row-3.
    //     Intake and Launch Artifacts in Row-2.
    //     Intake and Launch Artifacts in Row-1.
    //     Park!
    protected void testNewAuto(TimedTask autoTasks) {

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addDelay(100);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        autoTasks.addStep(() -> intake.tasks.allServoStore.restart());
//        autoTasks.addTimedStep(() -> {}, () -> intake.tasks.allServoStore.isDone(), 100);

        //Move to ObeliskView position.
        positionSolver.addMoveToTaskEx(DecodeSettings.getObeliskViewPos(), autoTasks, 1000);
        // Look at Obelisk, determine classificationId and Store it.
        autoTasks.addStep(intake.tasks.viewObelisk::restart);

        // Prep "Launch Motor".
        autoTasks.addStep(() -> intake.setLaunchRPM((int) DecodeSettings.getLaunchRPM()));
        autoTasks.addStep(() -> intake.tasks.startAutoFarLaunch.restart());   // TODO: Update startAutoFarLaunch to use intake.launchRPM.
        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);

        // Launch Pre-loaded Artifacts.
        // Determine LaunchOrder and Launch
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.restart());
        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.isDone());
//        autoTasks.addDelay(100);
//        //      Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
//        autoTasks.addStep(() -> intake.tasks.pinkBlueGreenServoLaunch.restart());
//        autoTasks.addStep(() -> intake.tasks.pinkBlueGreenServoLaunch.isDone());
//        autoTasks.addDelay(300);

//        //      Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
//        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
//        autoTasks.addStep(() -> intake.tasks.pinkServoLaunch.restart());
//        autoTasks.addDelay(300);
//        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionOne(), autoTasks);
//        autoTasks.addStep(() -> intake.tasks.blueServoLaunch.restart());
//        autoTasks.addDelay(300);
//        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionZero(), autoTasks);
//        autoTasks.addStep(() -> intake.tasks.greenServoLaunch.restart());
//        autoTasks.addDelay(300);

        // Intake from Row3 and Launch.
        if (runSpikeCount >= 1) {
            artifactIntakeAndLaunch(autoTasks, DecodeSettings.getPreIntakeArtifactRow3(), DecodeSettings.getIntakeArtifactRow3());
        }

        // Intake from Row2 and Launch.
        if (runSpikeCount >= 2) {
            artifactIntakeAndLaunch(autoTasks, DecodeSettings.getPreIntakeArtifactRow2(), DecodeSettings.getIntakeArtifactRow2());
        }

        // Intake from Row1 and Launch.
        if (runSpikeCount >= 3) {
            artifactIntakeAndLaunch(autoTasks, DecodeSettings.getPreIntakeArtifactRow1(), DecodeSettings.getIntakeArtifactRow1());
        }
        // Move to ParkAfterAuto Position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getParkAfterAutoPos(), autoTasks);
    }

    // Artifact Intake and Launch.
    protected void artifactIntakeAndLaunch (TimedTask autoTasks,
                                            Vector3 pos_pre_intake,
                                            Vector3 pos_intake) {
        // Move to pre_intake position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(pos_pre_intake, autoTasks);

        // Start "intake".
        autoTasks.addStep(() -> intake.tasks.intakeTask.restart());
        autoTasks.addStep(() -> intake.tasks.intakeTask.isDone());

        // Set positionSolver to "ExtraSlow" to allow intake slowly.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceExtraSlowSettings));

        // Move to intake.
        positionSolver.addMoveToTaskEx(pos_intake, autoTasks, 2000);
        autoTasks.addDelay(1000); // 1500; 2500; Test with 1000.

        // Move to launch.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.restart());
        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.isDone());

        //  Determine LaunchOrder and Launch
        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.restart());
        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.isDone());
//        autoTasks.addDelay(100);

//        //      Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
//        autoTasks.addStep(() -> intake.tasks.pinkBlueGreenServoLaunch.restart());
//        autoTasks.addStep(() -> intake.tasks.pinkBlueGreenServoLaunch.isDone());
//        autoTasks.addDelay(300);

//        // Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
//        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionTwo(), autoTasks);
//        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.restart());
//        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.isDone());
//        autoTasks.addStep(() -> intake.tasks.pinkServoLaunch.restart());
//        autoTasks.addDelay(300);
//        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionOne(), autoTasks);
//        autoTasks.addStep(() -> intake.tasks.blueServoLaunch.restart());
//        autoTasks.addDelay(300);
//        positionSolver.addMoveToTaskEx(DecodeSettings.getLaunchPositionZero(), autoTasks);
//        autoTasks.addStep(() -> intake.tasks.greenServoLaunch.restart());
//        autoTasks.addDelay(300);
    }

    public void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setAuto();
        DecodeSettings.setAllianceRed();
        DecodeSettings.setCurrentOpMode("T1_AutoFarRed");
        DecodeSettings.setTargetGoalPos(p_targetGoal);
        DecodeSettings.setRobotPosition(p_fieldStart);
        DecodeSettings.setObeliskViewPos(p_obeliskView);
        DecodeSettings.setLaunchPositionZero(p_launchPosZero);
        DecodeSettings.setLaunchPositionOne(p_launchPosOne);
        DecodeSettings.setLaunchPositionTwo(p_launchPosTwo);
        DecodeSettings.setPreIntakeArtifactRow1(p_pre_intakeArtifactRow1);
        DecodeSettings.setIntakeArtifactRow1(p_intakeArtifactRow1);
        DecodeSettings.setPreIntakeArtifactRow2(p_pre_intakeArtifactRow2);
        DecodeSettings.setIntakeArtifactRow2(p_intakeArtifactRow2);
        DecodeSettings.setPreIntakeArtifactRow3(p_pre_intakeArtifactRow3);
        DecodeSettings.setIntakeArtifactRow3(p_intakeArtifactRow3);
        DecodeSettings.setLeverOpenPos(p_leverOpen);
        DecodeSettings.setParkAfterAutoPos(p_parkAfterAuto);

        DecodeSettings.setLaunchRPM(launchRPM);
        DecodeSettings.lkTestMode1 = false;
    }
}