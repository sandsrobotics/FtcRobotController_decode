package org.firstinspires.ftc.teamcode.parts.positionsolver;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.HeadingSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.Part;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.Vector3;

public class HeadingSolver extends Part<Drive, HeadingSolverSettings, ObjectUtils.Null> {

/* * Hacked up addition by RhindleLAK * */

//    public static final class Events{
//        public static final String done = "DONE";
//    } //TODO add done event

    private boolean t = false;
    private long startTime;
    private boolean isStarted = false;
    Vector3 storedTarget = new Vector3();

    protected PositionTracker positionTracker;

    public final HChannelSolver hChannel = new HChannelSolver(this, "h channel") {
        @Override
        public double getError(double target) {
            // LK note:  This was originally designed with a solver target that is static, but
            // now we want it to be dynamic so things are going to get a little hack-ey.
            // Every update to the target resets the variables that track whether the movement is
            // complete, meaning either .isDone becomes useless or we need to avoid resetting
            // the target if not necessary.  This is the explanation for the following
            // weirdness.

            Vector3 currentPos = positionTracker.getCurrentPosition();
            if (currentPos==null) return 0;  // no position = no error

            // find the updated target angle
            double angle = calculateTargetHeading(currentPos, storedTarget);

            // if the target changes more than a certain amount, update;
            // otherwise leave it alone
            if (Math.abs(angle - target) > 0.5) {
                setNewTarget(angle, false);
            }

            // return the error
            return AngleMath.findAngleError(currentPos.Z, angle);
        }

        @Override
        public void move(DriveControl base) {
            if (base.power.Z==0) {   // this allows user rotate to override
                base.power = base.power.withZ(pid.returnValue());
            }
        }
    };

    public HeadingSolver(Drive parent) {
        super(parent, "heading solver");
        setSettings(HeadingSolverSettings.defaultSettings);
    }

    public HeadingSolver(Drive parent, HeadingSolverSettings settings){
        super(parent, "heading solver");
        setSettings(settings);
    }

    public void setNewTarget(Vector3 target, boolean resetPID){
        // Target is where to aim.
        Vector3 currentPos = positionTracker.getCurrentPosition();

        // Exit if either position is null
        if (target==null || currentPos==null) return;

        // Store the target for dynamic targeting later
        storedTarget = target.copy();

        // Calculate the vector from the current position to the target
        double angle = calculateTargetHeading(currentPos, storedTarget);

        // Set the solver for the target angle
        hChannel.setNewTarget(angle, resetPID);

        // Activate the solver (???)
        triggerEvent(Robot.Events.START);// TODO make this better
        isStarted = true;
    }

    double calculateTargetHeading(Vector3 currentPos, Vector3 target) {
        // Exit if either position is null
        if (currentPos==null) return 0;  // what else can we do?
        if (target==null) return currentPos.Z;  // if no target use current heading?

        // Calculate the vector from the current position to the target
        double x = target.X - currentPos.X;
        double y = target.Y - currentPos.Y;
        double distance = Math.sqrt(x*x + y*y);
        double angle = AngleMath.scaleAngle(Math.toDegrees(Math.atan2(y, x)));

        // Return the new heading
        return angle;
    }

    public void startSolver() {
        if (!isStarted) {
            triggerEvent(Robot.Events.START);
            isStarted = true;
        }
    }

    public void stopSolver() {
        if (isStarted) {
            triggerEvent(Robot.Events.STOP);
            isStarted = false;
        }
    }

    public void setMaxPower(double maxR){
        hChannel.setMaxPower(maxR);
    }

    public boolean isDone(){
        return hChannel.isDone();
    }

//    public void addMoveToTaskEx(Vector3 target, TaskEx task){
//        task.addStep(() -> setNewTarget(target, true));
//        task.addStep(this::isDone);
//    }
//
//    public void addMoveToTaskExNoWait(Vector3 target, TaskEx task){
//        task.addStep(() -> setNewTarget(target, true));
//    }
//
//    public void addMoveToTaskEx(Vector3 target, TaskEx task, int time){
//        //sanitize input
//        if(time <= 0) return;
//        task.addStep(() -> {startTime = System.currentTimeMillis();});
//        task.addStep(() -> setNewTarget(target, true));
//        task.addStep(() -> (System.currentTimeMillis() - startTime >= time) || isDone());
//    }

    @Override
    public void onBeanLoad() {
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, false);
    }

    @Override
    public void onSettingsUpdate(HeadingSolverSettings settings) {
        hChannel.setSettings(settings.rChannelSettings);
    }

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        // LK: if it's not started, weirdness happens and it doesn't stop. Investigate?
        if(!t) {
            t = true;
            setNewTarget(positionTracker.getCurrentPosition(), true);
        }
    }

    @Override
    public void onStop() {

    }
}
