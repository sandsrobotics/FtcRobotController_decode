package org.firstinspires.ftc.teamcode.parts.intake3;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake3.hardware.IntakeHardware3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

import static java.lang.Math.abs;

//@Config
public class Intake3 extends ControllablePart<Robot, IntakeSettings3, IntakeHardware3, IntakeControl3> {
    private final EdgeConsumer homingBucketZero = new EdgeConsumer();
    protected Drive drive;
    public Intake3Tasks tasks;
    protected PositionTracker pt;
    public boolean isTeleop;

    //***** Constructors *****
    public Intake3(Robot parent, String modeName) {
        super(parent, "Slider", () -> new IntakeControl3(0.5, false, 0, false));
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

    public void eStop() {
        stopAllIntakeTasks();
        getHardware().pixel.stop();
        getHardware().intakeMotor.setVelocity(0);
        getHardware().launchMotor.setVelocity(0);
        getHardware().launchServo.stop();
    }


    public void initializeServos() {
        getHardware().launchServo.setSweepTime(getSettings().launchSweepTime);
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
    }

    public void stopAllIntakeTasks() {
        tasks.movementTask.runCommand(Group.Command.PAUSE);
        tasks.movementTask.getActiveRunnables().clear();    // this is the magic sauce... must be used after the PAUSE or it will stop working
    }
    @Override
    public void onBeanLoad() {
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

        private Double getLedPwm(){
            return ledPwm;
        }

        private String getName(){
            return name;
        }

        LEDColor(String name, Double ledPwm){
            this.ledPwm = ledPwm;
            this.name = name;
        }
    }
}