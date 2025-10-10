package frc.robot.utils;

public class MapleTimeUtils {
    public static void delay(double seconds){
        try{
            // Convert seconds to total mnilliseconds
            long totalMills = (long) ( seconds * 1000);
            // Calculate the remaining nanoseconds
            int nanoPart = (int) ((seconds * 1000 - totalMills) * 1000000);

            // Pause the thread for the specified duration in millseconds and nanoseconds
            Thread.sleep(totalMills, nanoPart);
        } catch (InterruptedException e){
            // Restore the interrupted status
            Thread.currentThread().interrupt();
            // Optionally, handle the interruption, e.g. logging or throwing a runtime exception
            System.err.println("The sleep was interrupted");
        }
    }

}
