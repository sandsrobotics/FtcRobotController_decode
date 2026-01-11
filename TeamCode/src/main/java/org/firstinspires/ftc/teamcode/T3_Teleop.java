package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.depricated.intake.FlipbotSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.artifact.ArtifactDetectionPipeline;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.intake3.Intake3;
import org.firstinspires.ftc.teamcode.parts.intake3.IntakeTeleop3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamProp;

import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
import static om.self.ezftc.utils.Constants.tileSide;

@TeleOp(name="Arcade", group="32859")
public class T3_Teleop extends LinearOpMode {
    Drive drive;
    Robot robot;
    PositionSolver positionSolver;
    PositionTracker pt;
    Vector3 fieldStartPos = new Vector3(0,0,0);
    Pinpoint odo;
    Artifacts artifacts;
    LimeLight limelight;
    public void initTeleop(){
        new DriveTeleop(this.drive);
    }

    @Override
    public void runOpMode() {
        DecimalFormat df = new DecimalFormat("#0.0");
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        robot = new Robot(this);
        new BulkRead(robot);
        drive = new Drive(robot);
        artifacts = new Artifacts(robot);
        limelight = new LimeLight(robot);
        Led led = new Led(robot);
        initTeleop();

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
//        XRelativeSolver solver = new XRelativeSolver(drive);
        odo = new Pinpoint(pt,false);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive);
        positionSolver.setSettings(PositionSolverSettings.specimenAssistSettings);

        Intake3 intake = new Intake3(robot, "Teleop");
        new IntakeTeleop3(intake);
        robot.init();

        /* *********** Take this out for competition ************/
//        odo.setPosition(fieldStartPos);
        /*  ******************************************************/

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
//            led.setMiddleGroup2(3); //set to purple
            telemetry.addData("position", odo.getPosition());
            telemetry.addData ("Press Red or Blue button to select alliance","");
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped)) {
                intake.getHardware().pixel.setPosition(Intake3.LEDColor.BLUE.getLedPwm());
                IntakeSettings3.isRedSide = false;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.b, ButtonMgr.State.wasTapped)) {
                intake.getHardware().pixel.setPosition(Intake3.LEDColor.RED.getLedPwm());
                IntakeSettings3.isRedSide = true;
            }
            telemetry.addData("Alliance Color:",(IntakeSettings3.isRedSide?"Red":"Blue"));
            telemetry.update();
        }
        robot.start();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
//            telemetry.addData("position tracker", pt.getCurrentPosition());
            telemetry.addData("time", System.currentTimeMillis() - start);
//            telemetry.addData("Intake speed",intake.getHardware().intakeMotor.getVelocity());
            telemetry.addData("launch speed",intake.getCurrentLaunchRPM());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }

    /************************* Utilities **************************/
    public void moveRobot(Vector3 target){
        positionSolver.setNewTarget(target, false);
    }
    public Vector3 tiletoField(Vector3 p){
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }
    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }
}
