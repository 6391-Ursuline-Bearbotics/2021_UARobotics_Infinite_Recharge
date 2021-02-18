package frc.robot.UA6391;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

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
}

/* public final class WaypointUtil {
   private static final Logger LOGGER = Logger.getLogger(WaypointUtil.class.getName());
 
   private WaypointUtil() {
     throw new UnsupportedOperationException("This is a utility class!");
   }
 
   
   public static boolean export(String fileLocation, String filename, Double yoffset) {
     CsvMapper csvMapper = new CsvMapper();
     CsvSchema schema = CsvSchema.emptySchema().withHeader();
     WaypointCSV waypoint = new WaypointCSV();
 
     try (
         BufferedWriter writer = Files.newBufferedWriter(java.nio.file.Path.of(fileLocation + filename));
 
         FileInputStream fs = new FileInputStream (new File(filename));
         
         MappingIterator<Map<Double, Double, Double, Double, Boolean, Boolean, String>> it = csvMapper.readerFor(Map).with(schema).readValues(fs);
     ) {
       while (it.hasNext()) {
         double xPos = wp.getX();
         double yPos = wp.getY() + yoffset;
         double tangentX = wp.getTangentX();
         double tangentY = wp.getTangentY();
         String name = wp.getName();
         csvPrinter.printRecord(xPos, yPos, tangentX, tangentY, wp.isLockTangent(), wp.isReversed(), name);
       }
       csvPrinter.flush();
     } catch (IOException except) {
       LOGGER.log(Level.WARNING, "Could not save Path file", except);
       return false;
     }
     return true;
 */
