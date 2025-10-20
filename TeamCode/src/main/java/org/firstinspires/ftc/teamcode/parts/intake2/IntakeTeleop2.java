package org.firstinspires.ftc.teamcode.parts.intake2;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake2.settings.IntakeTeleopSettings2;
import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleop2 extends LoopedPartImpl<Intake2, IntakeTeleopSettings2, ObjectUtils.Null> {
    private IntakeTeleopSettings2 settings;
    int myDelay = 1000;
    public IntakeTeleop2(Intake2 parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings2.makeDefault(parent.parent));
    }

    public IntakeTeleop2(Intake2 parent, IntakeTeleopSettings2 settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
    }

    public IntakeTeleopSettings2 getSettings() {
        return settings;
    }

    public void setSettings(IntakeTeleopSettings2 settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl2(
                (double) settings.sweepSpeedSupplier.get(),
                (int) settings.sweepLiftSupplier.get(),
                (int) settings.sweepSlideSupplier.get(),
                (int) settings.bucketLiftSupplier.get(),
                (int) settings.robotLiftSupplier.get(),
                (int) settings.rotationServoSupplier.get(),
                (float) settings.strafeSpeedSupplier.get(),
                (int) settings.specimenServoSupplier.get(),
                (boolean) settings.robotEStopSupplier.get(),
                (boolean) settings.rangeingSupplier.get(),
                (int) settings.autoSampleSupplier.get()
        ), true);
    }

    @Override
    public void onRun() {
        if(settings.autoHomeSupplier.get()) // back button
            parent.tasks.startAutoHome();
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }
}
