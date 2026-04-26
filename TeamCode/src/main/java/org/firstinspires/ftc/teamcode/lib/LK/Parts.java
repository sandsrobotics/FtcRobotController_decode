package org.firstinspires.ftc.teamcode.lib.LK;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.LK.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.lib.LK.DataTypes.Position;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

// This class is a special glue class to allow the LK grafted classes (such as AutoDrive)
// to interface with the Om-type robot code without major rewrites.


//public class Parts { //implements PartsInterface {

public class Parts extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {

   public AutoDrive autoDrive;
   public PositionTracker positionTracker;
   public Drive drive;
   public boolean useDrivetrainEncoders = true;   //glue
   public boolean userDrive_isDriving = false;    //glue
   public double userDrive_storedHeading;

   public Parts(Robot parent) {
      super(parent, "parts");

      // maybe this moves to onInit?
      autoDrive = new AutoDrive(this);
      autoDrive.initialize();
      TelemetryMgr.setup(parent.opMode);
      TelemetryMgr.setDebugLevel(10);
      TelemetryMgr.enableAllCategories();
   }

   @Override
   public void onRun() {
      autoDrive.runLoop();
//      TelemetryMgr.Update();
   }

   @Override
   public void onBeanLoad() {
      positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, false);
      drive = getBeanManager().getBestMatch(Drive.class, false, false);
   }

   @Override
   public void onInit() {
   }

   @Override
   public void onStart() {
   }

   @Override
   public void onStop() {
   }

   // Glue functions
   public Position positionMgr_robotPosition() {
      return new Position(positionTracker.getOverridePosition());
   }

   public Position positionMgr_headingOnly() {
      return new Position(positionTracker.getOverridePosition());
   }

   public boolean positionMgr_hasPosition() {
      return true;
   }

   public boolean positionMgr_noPosition() {
      return false;
   }

   public void drivetrain_stopDriveMotors() {
      drive.stopRobot();
   }

   public void drivetrain_stopDriveMotors(boolean boo) {
      drive.stopRobot();
   }

   public void drivetrain_setDrivePowers(DrivePowers drivePowers) {
      drive.moveRobot(drivePowers.toArray());
   }

}