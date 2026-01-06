package org.firstinspires.ftc.teamcode.parts.intake3.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.lib.ServoSSR;
import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class IntakeHardware3 {
    public final DcMotorEx intakeMotor;
    public final DcMotorEx launchMotor;
    public final ServoSSR launchServo0;
    public final ServoSSR launchServo1;
    public final ServoSSR launchServo2;
    public final ServoSSR lockServo0;
    public final ServoSSR lockServo1;
    public final ServoSSR lockServo2;
    public final ServoSSR pixel;

    public IntakeHardware3(DcMotorEx intakeMotor, DcMotorEx launchMotor, ServoSSR launchServo0,ServoSSR launchServo1,ServoSSR launchServo2, ServoSSR pixel, ServoSSR lockServo0,ServoSSR lockServo1,ServoSSR lockServo2) {
        this.intakeMotor = intakeMotor;
        this.launchMotor = launchMotor;
        this.launchServo0 = launchServo0;
        this.launchServo1 = launchServo1;
        this.launchServo2 = launchServo2;
        this.lockServo0 = lockServo0;
        this.lockServo1 = lockServo1;
        this.lockServo2 = lockServo2;

        this.pixel = pixel;

        DcMotorEx[] motors = {this.intakeMotor, this.launchMotor};
        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
    //beans
    public static IntakeHardware3 makeDefault(HardwareMap hardwareMap)  {
        MotorSettings intakeMotorSettings =new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings launchMotorSettings = new MotorSettings(MotorSettings.Number.THREE_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);

        return new IntakeHardware3(
            intakeMotorSettings.makeExMotor(hardwareMap),
            launchMotorSettings.makeExMotor(hardwareMap),
            new ServoSSR(hardwareMap.get(Servo.class,"servo0B")), // launch servo on the left
            new ServoSSR(hardwareMap.get(Servo.class,"servo1B")), //launch servo in the middle
            new ServoSSR(hardwareMap.get(Servo.class,"servo2B")), // launch servo on the right
            new ServoSSR(hardwareMap.get(Servo.class,"servo5")),
            new ServoSSR(hardwareMap.get(Servo.class,"servo0")), // lock servo on the left
            new ServoSSR(hardwareMap.get(Servo.class,"servo1")), // lock servo in the middle
            new ServoSSR(hardwareMap.get(Servo.class,"servo2")) // lock servo on the right
        );
    }
}