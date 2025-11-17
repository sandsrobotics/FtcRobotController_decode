package org.firstinspires.ftc.teamcode.parts.intake3.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import om.self.ezftc.core.Robot;

@Config
public class IntakeTeleopSettings3 {
    public static PIDFCoefficients spinnerPID = new PIDFCoefficients(100,0,0,12.4);
    public static int launchRPM = 2500;
    public static int intakeRPM = 1500;
    public static double servoPosition = 0.5;
    public static double ticksPerRev = 38;


    public IntakeTeleopSettings3() {}

    public static IntakeTeleopSettings3 makeDefault(Robot robot) {
        return new IntakeTeleopSettings3();
    }
}
