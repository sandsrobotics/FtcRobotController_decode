package org.firstinspires.ftc.teamcode.parts.intake3;

import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake3.hardware.IntakeHardware3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

//@Config
public class Intake3 extends ControllablePart<Robot, IntakeSettings3, IntakeHardware3, IntakeControl3> {
    private final EdgeConsumer homingBucketZero = new EdgeConsumer();
    protected Drive drive;
    public Intake3Tasks tasks;
    protected PositionTracker pt;
    protected PositionSolver positionSolver;
    public boolean isTeleop;
    public int launchRPM;
    public int intakeRPM;
    public IntakeSettings3.LaunchData launchData;

    //***** Constructors *****
    public Intake3(Robot parent, String modeName) {
        super(parent, "Slider", () -> new IntakeControl3(false));
        this.isTeleop = modeName.equalsIgnoreCase("Teleop");
        setConfig(
                IntakeSettings3.makeDefault(),
                IntakeHardware3.makeDefault(parent.opMode.hardwareMap)
        );
    }

//    public void strafeRobot(DriveControl control) {
//        if (abs(strafePower) > .01) {
//            control.power = control.power.addX(strafePower / 3);
//        }
//    }

    public void setLaunchRPM(int RPM) {
        this.launchRPM = RPM;
        double targetTPS = (RPM / 60.0) * IntakeSettings3.ticksPerRev;
        getHardware().launchMotor.setVelocity(targetTPS);
    }

    public double getTargetLaunchRPM() {
        return this.launchRPM;
    }

    public double getCurrentLaunchRPM() {
        return ((getHardware().launchMotor.getVelocity()/IntakeSettings3.ticksPerRev) * 60);
    }

    public boolean launchRPMInTolerance() {
        return ((getHardware().launchMotor.getVelocity() * 60) / IntakeSettings3.ticksPerRev) >= (this.launchRPM - IntakeSettings3.launchRPMTolerance);
    }

    public void setIntakeRPM(int RPM) {
        this.intakeRPM = RPM;
        double targetTPS = (RPM / 60.0) * IntakeSettings3.ticksPerRev1150;
        getHardware().intakeMotor.setVelocity(targetTPS);
    }

    public double getTargetIntakeRPM() {
        return this.intakeRPM;
    }

    public void eStop() {
        stopAllIntakeTasks();
        getHardware().pixel.stop();
        getHardware().intakeMotor.setVelocity(0);
        getHardware().launchMotor.setVelocity(0);
        getHardware().launchServo0.stop();
        getHardware().launchServo1.stop();
        getHardware().launchServo2.stop();
    }

    public void initializeServos() {
        getHardware().launchServo0.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo1.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo2.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
        getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
        getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
    }

    @Override
    public void onInit() {
        getHardware().pixel.setPosition(LEDColor.YELLOW.getLedPwm());
        initializeServos();
        parent.opMode.sleep(1200);
        initializeServos();
        tasks = new Intake3Tasks(this, parent);
        tasks.constructAllIntakeTasks();
        getHardware().pixel.setPosition(LEDColor.GREEN.getLedPwm());
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
    }

    public void stopAllIntakeTasks() {
        tasks.movementTask.runCommand(Group.Command.PAUSE);
        tasks.movementTask.getActiveRunnables().clear();    // this is the magic sauce... must be used after the PAUSE or it will stop working
    }
    @Override
    public void onBeanLoad() {
//        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        initializeServos();
    }

    @Override
    public void onRun(IntakeControl3 control) {
        if (control.robotEStop) {
            eStop();
        }
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onStop() {
    }

    public enum LEDColor {
        OFF("off",0.0),
        RED("Red",0.279),
        ORANGE("Orange",0.333),
        YELLOW("Yellow",0.388),
        SAGE("Sage",0.444),
        GREEN("Green", 0.500),
        AZURE("Azure", 0.555),
        BLUE("Blue", 0.611),
        INDIGO("Indigo",0.666),
        VIOLET("Violet", 0.722),
        WHITE("White", 1.0);

        private final String name;
        private final Double ledPwm;

        Double getLedPwm(){
            return ledPwm;
        }

        String getName(){
            return name;
        }

        LEDColor(String name, Double ledPwm){
            this.ledPwm = ledPwm;
            this.name = name;
        }
    }
}