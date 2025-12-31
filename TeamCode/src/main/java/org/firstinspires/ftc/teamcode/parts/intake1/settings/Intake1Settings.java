package org.firstinspires.ftc.teamcode.parts.intake1.settings;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Intake1Settings {

    public static double intakeIn =                                    -1;
    public static double intakeStop =                                   0;
    public static double intakeOut =                                    1;
    public double sorterStop =                                 0.5;
    public double sorterStart =                                  0;
    public double servoGreenDock =                           0.463;
    public double servoGreenLaunch =                         0.235;
    public double servoPinkDock =                            0.518;
    public double servoPinkLaunch =                          0.719;
    public double servoBlueDock =                            0.494;
    public double servoBlueLaunch =                          0.671;
    public double ticksPerRevolution =                          28;
    public double secondsPerMinute =                            60;
    public static double farLaunchMotorRPM =                         3200;
    public static double goalLaunchMotorRPM =                        2500;
    public double farLaunchMotorVelocityStart =                  farLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double goalLaunchMotorVelocityStart =                 goalLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double launchMotorVelocityStop =                      0;


    public Intake1Settings() {
    }

    public static Intake1Settings makeDefault(){
        return new Intake1Settings();
    }
}

