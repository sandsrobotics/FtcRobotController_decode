package org.firstinspires.ftc.teamcode.parts.decode.settings;

public class  IntakeSettings {

    public double intakeIn =                                    -1;
    public double intakeStop =                                   0;
    public double sorterStop =                                 0.5;
    public double sorterStart =                                  0;
    public double servoGreenDock =                           0.425;
    public double servoGreenLaunch =                         0.275;
    public double servoPinkDock =                            0.546;
    public double servoPinkLaunch =                           0.34;
    public double servoBlueDock =                            0.494;
    public double servoBlueLaunch =                           0.65;
    public double launchMultiplierRate =                        28;
    public double ticksPerMinute =                              60;
    public double launchMotorRPM =                            6000;
    public double launchMotorVelocityStart =                  launchMotorRPM*ticksPerMinute/launchMultiplierRate;

    public double launchMotorVelocityStop =                      0;



    public IntakeSettings() {
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings();
    }
}

