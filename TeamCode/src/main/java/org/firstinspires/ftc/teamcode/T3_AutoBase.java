package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake3.Intake3;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
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
import om.self.ezftc.utils.Vector;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;
import static om.self.ezftc.utils.Constants.tileSide;

//@Config
@Disabled
@Autonomous(name="Base (Test only)", group="32859")
public class T3_AutoBase extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public boolean shutdownps;
    PositionSolver positionSolver;
    PositionTracker pt;
    Pinpoint odo;
    Intake3 intake;
    Artifacts artifacts;
    LimeLight limelight;

    //  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    public static int maxDelay = 3000;
    /**************************/
    Vector3 fieldStartPos;

    public void initAuto(){
        transformFunc = (v) -> v;
        fieldStartPos = new Vector3(-51, 51, 143);
    }

    @Override
    public void runOpMode() {
        long start;
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);
        intake = new Intake3(robot, "Autonomous");
        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2, 2, 2), fieldStartPos);
        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
        odo = new Pinpoint(pt);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        DecimalFormat df = new DecimalFormat("#0.0");
        artifacts = new Artifacts(robot);
        limelight = new LimeLight(robot);
        robot.init();

        while (!isStarted()) {
            telemetry.addData("position", pt.getCurrentPosition());
            limelight.onRun();
            telemetry.update();
            sleep(50);
        }
        
        odo.setPosition(fieldStartPos);
        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        // Setting up group container, task queue, and setting positionSolver target
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);

        // call the method to create auto tasks
        BaseAuto(autoTasks);

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run(); // Tasks are run as part of this run.
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            //telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("time", System.currentTimeMillis() - start);
            telemetry.addData("target launch speed", intake.getTargetLaunchRPM());
            telemetry.addData("current launch speed", df.format(intake.getCurrentLaunchRPM()));
            telemetry.addData("launch servo", intake.getHardware().launchServo0.getPosition());
            dashboardTelemetry.update();
            telemetry.update();
        }
        robot.stop();
    }

    public void BaseAuto(TimedTask autoTasks) {
        TestAuto(autoTasks);
    }

    /*********************** Autonomous Methods ******************/
    public void TestAuto(TimedTask autoTasks) { // from 14273
        // Positions to travel in Auto
        Vector3 shootRed = new Vector3(12, 12, 45);
        Vector3 shootBlue = new Vector3(-12, -12, -45);
        Vector3 fieldzero = new Vector3(0, 0, 0);

//        Vector3 StartOpt1 = new Vector3(-50,52,141);
        Vector3 AprilTag = new Vector3(-23.199,20.818,67.953);
        Vector3 ShootingPosition = new Vector3(-23.199,20.818,135.608);
        // set accuracy of position
        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        // movement tasks
//        positionSolver.addMoveToTaskEx(StartOpt1, autoTasks);
        positionSolver.addMoveToTaskEx(AprilTag, autoTasks);
        autoTasks.addDelay(5000);
        positionSolver.addMoveToTaskEx(ShootingPosition, autoTasks);
    }

    /************************* Utilities ************************/
    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    private Vector3 tileToInchAutoNoZ(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles)).withZ(tiles.Z);
    }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }
}
