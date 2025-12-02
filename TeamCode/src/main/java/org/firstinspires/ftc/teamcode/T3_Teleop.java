package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.intake3.Intake3;
import org.firstinspires.ftc.teamcode.parts.intake3.IntakeTeleop3;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
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
        initTeleop();

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        XRelativeSolver solver = new XRelativeSolver(drive);
        odo = new Pinpoint(pt,false);
        pt.positionSourceId = Pinpoint.class;

        Intake3 intake = new Intake3(robot, "Teleop");
        new IntakeTeleop3(intake);
        robot.init();

        while (!isStarted()) {
            telemetry.addData("position", odo.getPosition());
            telemetry.update();
        }
        robot.start();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position tracker", pt.getCurrentPosition());
            telemetry.addData("time", System.currentTimeMillis() - start);
            telemetry.addData("LaunchServo", intake.getHardware().launchServo.getPosition());
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
