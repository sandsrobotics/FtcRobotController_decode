package org.firstinspires.ftc.teamcode.parts.intake1.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class Intake1Hardware {
    public final DcMotorEx            intakeMotor;
    public final DcMotorEx            augerMotor;
    public final ServoSSR             intakeServo;
    public final ServoSSR             pinkServo;
    public final ServoSSR             blueServo;
    public final ServoSSR             greenServo;
    public final DcMotorEx            launchMotorLeft;
    public final DcMotorEx            launchMotorRight;

    public Intake1Hardware(DcMotorEx intakeMotor, DcMotorEx augerMotor, ServoSSR intakeServo, ServoSSR pinkServo, ServoSSR blueServo,
                           ServoSSR greenServo, DcMotorEx launchMotorLeft, DcMotorEx launchMotorRight) {
        this.intakeMotor = intakeMotor;
        this.augerMotor = augerMotor;
        this.intakeServo = intakeServo;
        this.pinkServo   = pinkServo;
        this.blueServo   = blueServo;
        this.greenServo  = greenServo;
        this.launchMotorLeft = launchMotorLeft;
        this.launchMotorRight = launchMotorRight;


//        DcMotorEx[] motors = {this.intakeMotor, this.launchMotorLeft, this.launchMotorRight};
        DcMotorEx[] motors = {this.intakeMotor};

        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
//beans
    public static Intake1Hardware makeDefault(HardwareMap hardwareMap)  {
        MotorSettings intakeMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings augerMotorSettings = new MotorSettings(MotorSettings.Number.THREE_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings launchMotorLeftSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.FLOAT, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings launchMotorRightSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.FLOAT, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);

        return new Intake1Hardware(
                intakeMotorSettings.makeExMotor(hardwareMap),
                augerMotorSettings.makeExMotor(hardwareMap),
                new ServoSSR(hardwareMap.get(Servo.class,"servo0")), // intakeServo
                new ServoSSR(hardwareMap.get(Servo.class,"servo1")), // pink servo
                new ServoSSR(hardwareMap.get(Servo.class,"servo2")),
                new ServoSSR(hardwareMap.get(Servo.class,"servo3")),
                launchMotorLeftSettings.makeExMotor(hardwareMap),
                launchMotorRightSettings.makeExMotor(hardwareMap)
                );
    }
}