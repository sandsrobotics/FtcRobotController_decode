package org.firstinspires.ftc.teamcode;

import static om.self.ezftc.utils.Constants.tileSide;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;
import org.firstinspires.ftc.teamcode.parts.intake1.Intake1;
import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1Settings;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.text.DecimalFormat;
import java.util.Objects;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

@Config
@Autonomous (name="14273 AutoFarRed", group="14273")
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

    // Positions to travel in AutoFarRed
    Vector3 p_fieldStartPos             = new Vector3(64,16,180);
    Vector3 p_parkAfterAuto             = new Vector3(32,16,157);

    Vector3 p_obeliskView               = new Vector3(58, 16, 180);  // Was: 56, 16, 180; // FarRed: ObeliskView Position
    Vector3 p_LaunchPos                 = new Vector3(58,16,157);    // Was: 56, 16, 153; // FarRed Launching Position.
    Vector3 p_LaunchPosZero             = new Vector3(58,16,157);    // Was: 56, 16, 153; // FarRed Launching Position.
    Vector3 p_LaunchPosOne              = new Vector3(58,16,157);    // Was: 56, 16, 153; // FarRed Launching Position.
    Vector3 p_LaunchPosTwo              = new Vector3(58,16,160);    // FarRed Launching Position for pinkServo. Z:160.

    Vector3 p_pre_IntakeRedArtifactRow1 = new Vector3(-12, 28, -90);  // Red: Ready to collect on Row1
    Vector3 p_IntakeRedArtifactRow1     = new Vector3(-12, 53, -90);  // Red: Intake Artifacts in Row1
    Vector3 p_pre_IntakeRedArtifactRow2 = new Vector3(12, 28, -90);   // Red: Ready to collect on Row2
    Vector3 p_IntakeRedArtifactRow2     = new Vector3(12, 60, -90);   // Red: Intake Artifacts in Row2
    Vector3 p_pre_IntakeRedArtifactRow3 = new Vector3(35.5, 28, -90);   // Red: Ready to collect in Row3
    Vector3 p_IntakeRedArtifactRow3     = new Vector3(35.5, 60, -90);   // Red: Intake Artifacts in Row3
    Vector3 p_LeverOpen                 = new Vector3(0, 55, 180);    // Red: Open Lever Position

    //  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    public static int maxDelay = 10000;
    /**************************/
    public int startDelay;
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

        intake.launchRPM = (int) Intake1Settings.autoFarLaunchMotorRPM;

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

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
            telemetry.addData("AUTO RED: ", "Not Started");
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper, ButtonMgr.State.wasTapped)) {
                startDelay += 1000;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.left_bumper, ButtonMgr.State.wasTapped)) {
                startDelay -= 1000;
                if(startDelay < 0) startDelay = 0;
            }
            if(startDelay > maxDelay) startDelay = maxDelay;

            telemetry.addData("START POSITION:", odo.getPosition());
            telemetry.addData("START DELAY:", startDelay / 1000);
            telemetry.update();
            sleep(50);
        }

        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        // Setting up group container, task queue, and setting positionSolver target
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);
        //positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        // Here is where we schedule the tasks for the autonomous run (testNewAuto function below run loop)
        // testNewAuto - Successfully Tested!
        testNewAuto(autoTasks);

        startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            robot.run();
            DecodeSettings.storeRobotPosition(pt.getCurrentPosition());
            telemetry.addData("Position", odo.getPosition());
            telemetry.addData("Launch Motor RPM", intake.getLaunchMotorRPM());
            telemetry.addData("ClassificationId", DecodeSettings.getClassificationId());
            telemetry.addData("time", System.currentTimeMillis() - startTime);
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
    private void testNewAuto(TimedTask autoTasks) {

        // Reset and Get Ready.
        autoTasks.addStep(() -> intake.stopAllIntakeTasks());
        autoTasks.addDelay(250);
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        autoTasks.addStep(() -> intake.tasks.allServoStore.restart());
        autoTasks.addTimedStep(() -> {}, () -> intake.tasks.allServoStore.isDone(), 250);

        //Move to ObeliskView position.
        positionSolver.addMoveToTaskEx(p_obeliskView, autoTasks);
        // Look at Obelisk, determine classificationId and Store it.
        DecodeSettings.setClassificationId(limelight.getClassificationId());

        // Prep "Launch Motor".
        autoTasks.addStep(() -> intake.setLaunchRPM((int) Intake1Settings.autoFarLaunchMotorRPM));
        autoTasks.addStep(() -> intake.tasks.startAutoFarLaunch.restart());
        autoTasks.addTimedStep(() -> {}, () -> intake.launchRPMInTolerance(), 3000);

//        // Move to Launch Position.
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
//        positionSolver.addMoveToTaskEx(p_LaunchPos, autoTasks);
//
        // Launch Pre-loaded Artifacts.
        //      Determine LaunchOrder and Launch.
//        autoTasks.addStep(() -> intake.computeLaunchOrderAndLaunch(DecodeSettings.getClassificationId()));
//        autoTasks.addDelay(2500);
        //      Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
        positionSolver.addMoveToTaskEx(p_LaunchPosTwo, autoTasks);
        autoTasks.addStep(() -> intake.tasks.pinkServoLaunch.restart());
        autoTasks.addDelay(300);
        positionSolver.addMoveToTaskEx(p_LaunchPosOne, autoTasks);
        autoTasks.addStep(() -> intake.tasks.blueServoLaunch.restart());
        autoTasks.addDelay(300);
        positionSolver.addMoveToTaskEx(p_LaunchPosZero, autoTasks);
        autoTasks.addStep(() -> intake.tasks.greenServoLaunch.restart());
        autoTasks.addDelay(300);

        // Intake from Row3 and Launch.
        artifactIntakeAndLaunch(autoTasks, p_pre_IntakeRedArtifactRow3, p_IntakeRedArtifactRow3,
                    p_LaunchPos, launchRPM);

        // Intake from Row2 and Launch.
        artifactIntakeAndLaunch(autoTasks, p_pre_IntakeRedArtifactRow2, p_IntakeRedArtifactRow2,
                                    p_LaunchPos, launchRPM);

        // Intake from Row1 and Launch.
//        artifactIntakeAndLaunch(autoTasks, p_pre_IntakeRedArtifactRow1, p_IntakeRedArtifactRow1,
//                p_LaunchPos, launchRPM);

        // Move to ParkAfterAuto Position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(p_parkAfterAuto, autoTasks);
    }

    // Artifact Intake and Launch.
    protected void artifactIntakeAndLaunch (TimedTask autoTasks,
                                            Vector3 p_pre_intake, Vector3 p_intake,
                                            Vector3 p_Launch, Integer launchRPM) {

        // Move to pre_intake position.
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSettings));
        positionSolver.addMoveToTaskEx(p_pre_intake, autoTasks);

        // Start "intake".
        autoTasks.addStep(() -> intake.tasks.intakeTask.restart());
        autoTasks.addStep(() -> intake.tasks.intakeTask.isDone());

        // Set positionSolver to "Slow" to allow intake slowly.
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceExtraSlowSettings));

        // Move to intake.
        positionSolver.addMoveToTaskEx(p_intake, autoTasks);
        autoTasks.addDelay(2500); // Test with 1000.

        // StopIntake.
        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.restart());
        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.isDone());
//        autoTasks.addStep(() -> intake.tasks.allServoStore.restart());

//        // Move to launch.
//        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
//        positionSolver.addMoveToTaskEx(p_Launch, autoTasks);
//
//        // Compute LaunchOrder and Launch Artifacts.
//        // autoTasks.addStep(() -> intake.computeLaunchOrderAndLaunch(DecodeSettings.getClassificationId()));
//        autoTasks.addStep(() -> intake.tasks.computeAndLaunchInOrder.restart());
////        autoTasks.addStep(() -> intake.tasks.pinkBlueGreenServoLaunch.restart());
//        autoTasks.addDelay(2000);

        // Move to LaunchPositions and launchServos in defaultOrder. (pink, blue, green).
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultTwiceSlowSettings));
        positionSolver.addMoveToTaskEx(p_LaunchPosTwo, autoTasks);
        autoTasks.addStep(() -> intake.tasks.pinkServoLaunch.restart());
        autoTasks.addDelay(300);
        positionSolver.addMoveToTaskEx(p_LaunchPosOne, autoTasks);
        autoTasks.addStep(() -> intake.tasks.blueServoLaunch.restart());
        autoTasks.addDelay(300);
        positionSolver.addMoveToTaskEx(p_LaunchPosZero, autoTasks);
        autoTasks.addStep(() -> intake.tasks.greenServoLaunch.restart());
        autoTasks.addDelay(300);
//        autoTasks.addStep(() -> intake.tasks.artifactIntakeStopTask.restart());
    }

    public void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setAuto();
        DecodeSettings.setAllianceRed();

        DecodeSettings.storeRobotPosition(p_fieldStartPos);
        DecodeSettings.storeLaunchPositionZero(p_LaunchPosZero);
        DecodeSettings.storeLaunchPositionOne(p_LaunchPosOne);
        DecodeSettings.storeLaunchPositionTwo(p_LaunchPosTwo);
    }
}