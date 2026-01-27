package org.firstinspires.ftc.teamcode.parts.intake1;

import om.self.ezftc.utils.Vector3;

public class DecodeSettings {

   /* The purpose of this class is to store values accessible to all classes in FlipBot,
      and which can remain from run to run
    */

   static boolean modeTeleOp = true;

   static boolean allianceBlue = true;

   static Vector3 robotPosition = new Vector3();
   static Vector3 currentLaunchPosZero = new Vector3(); // Each Mode is supposed to set this value before calling computeAndLaunchInOrder.
   static Vector3 currentLaunchPosOne = new Vector3(); // Each Mode is supposed to set this value before calling computeAndLaunchInOrder.
   static Vector3 currentLaunchPosTwo = new Vector3(); // Each Mode is supposed to set this value before calling computeAndLaunchInOrder.

   // LK new test stuff
   static Vector3 fusedRobotPosition = new Vector3();
   public static Vector3 targetRed  = new Vector3(-70.5, 70.5, 0.0);
   public static Vector3 targetBlue = new Vector3(-70.5, -70.5, 0.0);
   public static boolean lkPinpoint = false;
   public static boolean lkTestMode1 = false;
   public static boolean lkTestMode2 = false;
   // --

   public static boolean autonomousDebugMode = false;
   public static boolean firstRun = true;
   static Vector3 controlGovernor = new Vector3(1,1,1);
   public static boolean isDemoMode = false;
   public static double demoDriverDefaultMultiplier = 0.5;
   public static double demoDriverMultiplier = 0;

   public static int pinpointSettingsXoffset = +105;
   public static int pinpointSettingsYoffset = +150;
   public static float pinpointSettingsResolution = 13.26291192f;

   static int classificationId = 21; // Defaults to 21.

   public static void setClassificationId (int Id) { classificationId = Id; };
   public static int getClassificationId () { return classificationId; };

   public static void setTeleOp () { modeTeleOp = true; }
   public static void setAuto () { modeTeleOp = false; }
   public static boolean isTeleOp() { return modeTeleOp; }
   public static boolean isAuto() { return !modeTeleOp; }

   public static void setAllianceBlue () { allianceBlue = true; }
   public static void setAllianceRed() { allianceBlue = false; }
   public static boolean isAllianceBlue() { return allianceBlue; }
   public static boolean isAllianceRed() { return !allianceBlue; }

   public static void setControlGovernor(Vector3 multiplier) {
      controlGovernor = new Vector3(clamp(multiplier.X), clamp(multiplier.Y), clamp(multiplier.Z));
   }
   public static void setControlGovernor(double multiplier) {
      controlGovernor = new Vector3(clamp(multiplier));
      //controlGovernor = new Vector3(clamp(multiplier, 0.25, 1));
   }
   public static Vector3 getControlGovernor() {
      return controlGovernor;
   }

   public static void storeRobotPosition(Vector3 currentPosition) {
      if (currentPosition.X == 0 && currentPosition.Y == 0 && currentPosition.Z == 0) return;
      robotPosition = currentPosition;
   }

   public static Vector3 getRobotPosition() {
      return robotPosition;
   }

   public static void storeFusedPosition(Vector3 fusedPosition) {
      fusedRobotPosition = fusedPosition;
   }

   public static Vector3 getFusedRobotPosition() {
      if (fusedRobotPosition != null && (fusedRobotPosition.X != 0 || fusedRobotPosition.Y != 0 || fusedRobotPosition.Z != 0 )) {
         return fusedRobotPosition;
      }
      else return robotPosition;
   }

   public static void storeLaunchPositionZero(Vector3 launchPosition) {
      if (launchPosition.X == 0 && launchPosition.Y == 0 && launchPosition.Z == 0) return;
      currentLaunchPosZero = launchPosition;
   }
   public static Vector3 getLaunchPositionZero() {
      return currentLaunchPosZero;
   }

   public static void storeLaunchPositionOne(Vector3 launchPosition) {
      if (launchPosition.X == 0 && launchPosition.Y == 0 && launchPosition.Z == 0) return;
      currentLaunchPosOne = launchPosition;
   }
   public static Vector3 getLaunchPositionOne() {
      return currentLaunchPosOne;
   }

   public static void storeLaunchPositionTwo(Vector3 launchPosition) {
      if (launchPosition.X == 0 && launchPosition.Y == 0 && launchPosition.Z == 0) return;
      currentLaunchPosTwo = launchPosition;
   }
   public static Vector3 getLaunchPositionTwo() {
      return currentLaunchPosTwo;
   }

   private static double clamp(double pos) {
      return Math.max(0, Math.min(pos, 1));
   }
   private static double clamp(double val, double min, double max) {
      return Math.max(min, Math.min(val, max));
   }

}