package frc.robot.UA6391;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.ControlVectorList;

public class Trajectory6391 {   
   public static Trajectory fromWaypoints(Path path, TrajectoryConfig config) throws IOException {
      try (BufferedReader reader = Files.newBufferedReader(path)) {
         var interiorWaypoints = new ArrayList<Translation2d>();
         Pose2d start = new Pose2d();
         Pose2d end = new Pose2d();
         int loop = 0;
         String line;
         String lastline = "";

         while ((line = reader.readLine()) != null) {
            if (loop == 0 || loop == 2) {
               ; // skip the header and the second line because we are logging last
            }
            else if (loop == 1) {
               start = createPoseWaypoint(line);
            }
            else {
               interiorWaypoints.add(createTranslationWaypoint(lastline));
            }
            lastline = line;
            loop++;
         }

         end = createPoseWaypoint(lastline);

         return TrajectoryGenerator.generateTrajectory(
            start,
            interiorWaypoints,
            end,
            config);
      }
   }

   public static Trajectory importPathToQuinticTrajectory(String filepath, TrajectoryConfig config) throws IOException {
      try (FileReader fr = new FileReader(new File(filepath));
      BufferedReader reader = new BufferedReader(fr)) {

      ControlVectorList controlVectors = new ControlVectorList();
      int loop = 0;
      String line;

      while ((line = reader.readLine()) != null) {
         if (loop == 0) {
            if (!line.equals("X,Y,Tangent X,Tangent Y,Fixed Theta,Reversed,Name")) {
               throw new RuntimeException("this isn’t a PathWeaver csv file");
            }
         }
         else {
            controlVectors.add(createControlVector(line));
         }
         loop++;
      }
      return TrajectoryGenerator.generateTrajectory(controlVectors,            
            config);
      }
   }

   private static Pose2d createPoseWaypoint(String input) {
      String[] arrOfStr = input.split(",", 0);
      // 8.21m is the Height of the field PathWeaver and traj use different starting points
      return new Pose2d(new Translation2d(Double.parseDouble(arrOfStr[0]), 4.572 + Double.parseDouble(arrOfStr[1])), //8.21 field height & 4.572 challenge height
         new Rotation2d(Double.parseDouble(arrOfStr[2]), Double.parseDouble(arrOfStr[3])));
   }

   private static Translation2d createTranslationWaypoint(String input) {
      String[] arrOfStr = input.split(",", 0);
      return new Translation2d(Double.parseDouble(arrOfStr[0]), 4.572 + Double.parseDouble(arrOfStr[1]));
   }

   private static ControlVector createControlVector(String input) {
      String[] arrOfStr = input.split(",", 0);
      double[] x = new double[] {Double.parseDouble(arrOfStr[0]), Double.parseDouble(arrOfStr[2])};
      double[] y = new double[] {Double.parseDouble(arrOfStr[1]), Double.parseDouble(arrOfStr[3])};
      return new ControlVector(x, y);
   }

   public static List<Double> getEventsFromWaypoints(Trajectory trajectory, Path waypointpath, List<Integer> eventWaypoints) throws IOException {
      try (BufferedReader reader = Files.newBufferedReader(waypointpath)) {
         int loop = 0;
         int eventIndex = 0;
         String line;
         List<Double> eventTimes = new ArrayList<>();

         while ((line = reader.readLine()) != null) {
            if (loop != 0) {
               // skip the header
               if (eventWaypoints.get(eventIndex) == loop - 1) {
                  // we need the time at this waypoint
                  eventTimes.add(searchTranslation(trajectory, createTranslationWaypoint(line)));
                  eventIndex++;
                  if (eventIndex == eventWaypoints.size()) {
                     break;
                  }
               }
            }
            loop++;
         }
         return eventTimes;
      }
   }

   public static Double searchTranslation(Trajectory trajectory, Translation2d searchTranslation) {
      for (var state : trajectory.getStates()) {
         if (state.poseMeters.getTranslation().getDistance(searchTranslation) < .01) {
            return state.timeSeconds;
         }
      }
      return -1.0; // It didn't find the translation in that trajectory
   }
}