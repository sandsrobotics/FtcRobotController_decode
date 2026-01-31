package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake3.Intake3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
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
    public boolean isRedSide;
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
    static public int runSpikeCount = 3; // - Default to all 3 spikes
    /**************************/
    Vector3 fieldStartPos;

    public void initAuto(){
        IntakeSettings3.launchArmed = false;
        isRedSide = false;
        transformFunc = (v) -> v;
        fieldStartPos = new Vector3(-51, 51, 143);
    }

    double launchServo0Rest = IntakeSettings3.launchServo0Rest;
    double launchServo1Rest = IntakeSettings3.launchServo1Rest;
    double launchServo2Rest = IntakeSettings3.launchServo2Rest;

    public void resetIntakeSettings()
    {
        IntakeSettings3.launchServo0Rest = launchServo0Rest;
        IntakeSettings3.launchServo1Rest = launchServo1Rest;
        IntakeSettings3.launchServo2Rest = launchServo2Rest;
    }

    public void setIntakeSettings()
    {
        IntakeSettings3.launchServo0Rest = 0.637; //0.654 - ground, Qualifier - 0.62
        IntakeSettings3.launchServo1Rest = 0.35; //.324 - ground, Qualifier - 0.35 intermediate - 0.337
        IntakeSettings3.launchServo2Rest = 0.265; //.237 - ground, Qualifier - 0.265 intermediate - 2.51
    }

    @Override
    public void runOpMode() {
        long start;

        setIntakeSettings();
        initAuto();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);
        intake = new Intake3(robot, "Autonomous");
        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2, 2, 2), fieldStartPos);
        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
        odo = new Pinpoint(pt);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive);
        DecimalFormat df = new DecimalFormat("#0.0");
        artifacts = new Artifacts(robot);
        limelight = new LimeLight(robot);
        robot.init();
        IntakeSettings3.isRedSide = this.isRedSide;

        while (!isStarted()) {
            robot.buttonMgr.runLoop();

            telemetry.addData("position", odo.getPosition());
            telemetry.addData("Press D-PAD UP/DOWN to change spike count from closest to farthest","");

            // D-pad controls for spike count
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasHeld))
            {
                runSpikeCount = Math.min(3, runSpikeCount + 1);
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasHeld))
            {
                runSpikeCount = Math.max(1, runSpikeCount - 1);
            }

            telemetry.addData("Spikes to run:", runSpikeCount);

            // LED color indicator
            if (IntakeSettings3.isRedSide) {
                intake.getHardware().pixel.setPosition(Intake3.LEDColor.RED.getLedPwm());
            } else {
                intake.getHardware().pixel.setPosition(Intake3.LEDColor.BLUE.getLedPwm());
            }

            telemetry.addData("Alliance Color:",(IntakeSettings3.isRedSide?"Red":"Blue"));
            limelight.onRun();
            telemetry.update();
            sleep(50);
        }

        odo.setPosition(fieldStartPos);
        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);

        Group container = new Group("container", robot.taskManager);
        TimedTask autoTasks = new TimedTask("auto task", container);

        BaseAuto(autoTasks);

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("time", System.currentTimeMillis() - start);
            telemetry.addData("target launch speed", intake.getTargetLaunchRPM());
            telemetry.addData("current launch speed", df.format(intake.getCurrentLaunchRPM()));
            telemetry.addData("launch servo", intake.getHardware().launchServo0.getPosition());
            telemetry.update();
        }

        resetIntakeSettings();
        robot.stop();
    }

    public void BaseAuto(TimedTask autoTasks) {
        TestAuto(autoTasks);
    }

    public void TestAuto(TimedTask autoTasks) {
        Vector3 shootRed = new Vector3(12, 12, 45);
        Vector3 shootBlue = new Vector3(-12, -12, -45);
        Vector3 fieldzero = new Vector3(0, 0, 0);

        Vector3 AprilTag = new Vector3(-23.199,20.818,67.953);
        Vector3 ShootingPosition = new Vector3(-23.199,20.818,135.608);

        autoTasks.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        positionSolver.addMoveToTaskEx(AprilTag, autoTasks);
        autoTasks.addDelay(5000);
        positionSolver.addMoveToTaskEx(ShootingPosition, autoTasks);
    }

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