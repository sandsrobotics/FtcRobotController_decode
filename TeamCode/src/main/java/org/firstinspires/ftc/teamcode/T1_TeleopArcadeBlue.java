package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;
import org.firstinspires.ftc.teamcode.parts.intake1.Intake1;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;
import org.firstinspires.ftc.teamcode.parts.intake1.Intake1Teleop;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.text.DecimalFormat;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;

@TeleOp(name="14273 TeleArcadeBlue", group="B14273")
public class T1_TeleopArcadeBlue extends LinearOpMode {
    double tileSide = 23.5;
    Drive drive;
    Robot robot;
    Intake1 intake;
    Artifacts artifacts;
    LimeLight limelight;
    PositionSolver positionSolver;
    PositionTracker pt;
    Pinpoint odo;
    protected Vector3 fieldStartPos = new Vector3(64,-16,180);
    boolean testModeReverse = false;

    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeArcade1(robot));
    }

    @Override
    public void runOpMode() {
        extraSettings();

        DecimalFormat df = new DecimalFormat("#0.0");
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        robot = new Robot(this);
        new BulkRead(robot);
        drive = new Drive(robot);

        limelight = new LimeLight(robot);
        artifacts = new Artifacts(robot);

        initTeleop();

        Vector3 tempPosition = DecodeSettings.getRobotPosition();
        if (tempPosition.X == 0.0 && tempPosition.Y == 0.0 && tempPosition.Z == 0.0)  {
            DecodeSettings.storeRobotPosition(fieldStartPos);
        }

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), DecodeSettings.getRobotPosition());
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        // XRelativeSolver solver = new XRelativeSolver(drive);

//       EncoderTracker et = new EncoderTracker(pt);
//       pt.positionSourceId = EncoderTracker.class;

        odo = new Pinpoint(pt, false, "odo",
                DecodeSettings.pinpointSettingsXoffset, DecodeSettings.pinpointSettingsYoffset, DecodeSettings.pinpointSettingsResolution,
                GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        positionSolver.setSettings(PositionSolverSettings.specimenAssistSettings);

        intake = new Intake1(robot);
        new Intake1Teleop(intake);

        robot.init();
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
            telemetry.addData("Not Started", "Not Started");

//            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped) ||
//                    robot.buttonMgr.getState(2, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped)) {
//                intake.initializeServos();
//            }

            telemetry.addData("Position", odo.getPosition());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        robot.start();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("Launch Motor RPM", intake.getLaunchMotorRPM());
            telemetry.addData("Position", odo.getPosition());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }

    protected void extraSettings() {
        DecodeSettings.isDemoMode = false;
        DecodeSettings.setTeleOp();
        DecodeSettings.setAllianceBlue();
    }
}
