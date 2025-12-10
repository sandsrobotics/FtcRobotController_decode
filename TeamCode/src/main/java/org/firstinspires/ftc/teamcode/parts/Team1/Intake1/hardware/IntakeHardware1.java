package org.firstinspires.ftc.teamcode.parts.Team1.Intake1.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class IntakeHardware1 {
    public final DcMotorEx intakeMotor;

    public final DcMotorEx launcherMotor1;

    public final DcMotorEx launcherMotor2;

    public final ServoSSR flickServo1;

    public final ServoSSR flickServo2;

    public final ServoSSR flickServo3;

    public final ServoSSR launcherServo;

    public final ServoSSR colorpixel;

    public IntakeHardware1(DcMotorEx intakeMotor, DcMotorEx launcherMotor1, DcMotorEx launcherMotor2, ServoSSR flickServo1, ServoSSR flickServo2, ServoSSR flickServo3, ServoSSR launcherServo, ServoSSR colorpixel) {
        this.intakeMotor = intakeMotor;
        this.launcherMotor1 = launcherMotor1;
        this.launcherMotor2 = launcherMotor2;
        this.flickServo1 = flickServo1;
        this.flickServo2 = flickServo2;
        this.flickServo3 = flickServo3;
        this.launcherServo = launcherServo;
        this.colorpixel = colorpixel;

        DcMotorEx[] motors = {this.intakeMotor, this.launcherMotor1, this.launcherMotor2};
        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }

    public static IntakeHardware1 makeDefault(HardwareMap hardwareMap) {
        MotorSettings intakeMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings launchMotor1Settings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorEx.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings launchMotor2Settings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorEx.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 1);

        return new IntakeHardware1(
            intakeMotorSettings.makeExMotor(hardwareMap),
            launchMotor1Settings.makeExMotor(hardwareMap),
            launchMotor2Settings.makeExMotor(hardwareMap),
            new ServoSSR(hardwareMap.get(Servo.class,"servo0")),
            new ServoSSR(hardwareMap.get(Servo.class,"servo1")),
            new ServoSSR(hardwareMap.get(Servo.class,"servo2")),
            new ServoSSR(hardwareMap.get(Servo.class,"servo3")),
            new ServoSSR(hardwareMap.get(Servo.class,"servo4"))
        );
    }
}
