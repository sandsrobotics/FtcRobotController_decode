package org.firstinspires.ftc.teamcode.parts.drive.headeraimer;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.part.Part;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.PID;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.modifiers.EdgeExModifier;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class HeaderAimer extends Part<Drive, HeaderAimerSettings, ObjectUtils.Null> {
    private PositionTracker pt;
    private HeaderAimerSettings settings;

    private Vector3 storedTarget = new Vector3();
    private boolean autoAim = false;
    private double lastHeading;
    private final PID pid = new PID();

    private final TimedTask headerAimer = new TimedTask("aim header");
    private final EdgeExModifier edgeModifier = new EdgeExModifier();

    public HeaderAimer(Drive parent, HeaderAimerSettings settings) {
        super(parent, "header aimer");
        setSettings(settings);
    }

    public HeaderAimer(Drive parent) {
        super(parent, "header aimer");
        setSettings(HeaderAimerSettings.makeDefault());
    }

    public HeaderAimerSettings getSettings() {
        return settings;
    }

    public void setSettings(HeaderAimerSettings settings) {
        this.settings = settings;

        pid.PIDs = settings.pidCoefficients;
//        pid.minClamp = -settings.maxI;
//        pid.maxClamp = settings.maxI;
        buildHeaderAimer();
    }

    @Override
    public void onBeanLoad() {
        pt = getBeanManager().getBestMatch(PositionTracker.class, false, false);
    }

    private void buildHeaderAimer(){
        headerAimer.clear();
//        headerAimer.addDelay(getSettings().headingSettleDelay);
        headerAimer.addStep(() -> {
//            lastHeading = pt.getCurrentPosition().Z;
            pid.resetErrors();
        });
//        headerAimer.addStep(() -> pid.updatePID(AngleMath.findAngleError(pt.getCurrentPosition().Z, lastHeading)), () -> false);
//        headerAimer.addStep(() -> pid.updatePID(AngleMath.findAngleError(pt.getCurrentPosition().Z, calculateTargetHeading())), () -> false); //!autoAim);
        headerAimer.addStep(() -> pid.updatePID(AngleMath.findAngleError(pt.getOverridePosition().Z, calculateTargetHeading())), () -> false); //!autoAim);
    }

    @Override
    public void onInit() {
        //create header keeper task
        headerAimer.attachParent(getTaskManager());
        buildHeaderAimer();

        //automatically trigger and pause
        edgeModifier.setOnRise(headerAimer::restart);
        edgeModifier.setOnFall(() -> headerAimer.runCommand(Group.Command.PAUSE));
    }

    @Override
    public void onStart() {
        parent.addController("header aimer", (control) -> {
            if(edgeModifier.apply(Math.abs(control.power.Z) < settings.minRegisterVal && autoAim)){
                control.power = control.power.withZ(pid.returnValue());
            }
        });
    }

    @Override
    public void onStop() {
        parent.removeController("header aimer");
    }

    public void setAutoAim(boolean autoAim) {
        this.autoAim = autoAim;
    }

    public void setTarget(Vector3 newTarget) {
        if (newTarget != null) storedTarget = newTarget.copy();
    }

    double calculateTargetHeading () {
        // Target is where to aim. Need to calculate appropriate heading
//        Vector3 currentPos = pt.getCurrentPosition();
        Vector3 currentPos = pt.getOverridePosition();

        // Exit if either position is null
        if (currentPos==null) return 0;  // what else can we do?
        if (storedTarget==null) return currentPos.Z;  // if no target use current heading?

        // Calculate the vector from the current position to the target
        double x = storedTarget.X - currentPos.X;
        double y = storedTarget.Y - currentPos.Y;
        double distance = Math.sqrt(x*x + y*y);
        double angle = AngleMath.scaleAngle(Math.toDegrees(Math.atan2(y, x)));

        // Return the new heading
        return angle;
    }
}
