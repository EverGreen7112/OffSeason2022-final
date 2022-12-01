package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Collections;

import frc.robot.statics.Constants;

public class InitInterpolation {
    
    /**
     * 
     * @param fileName the name of the file you want to read data from. two double values seperated by ',' per line, each line is a point.
     * @return new linear interpolation with data from the text file.
     */
    public static LinearInterpolation getLinearInterpulation(String fileName){
        ArrayList<Point> points = new ArrayList<Point>();
        try(BufferedReader br = new BufferedReader(new FileReader(fileName))) {
            String line = br.readLine();
            while (line != null) {
                String[] pointSTR = line.split(",");
                points.add(new Point(Double.parseDouble(pointSTR[0]), Double.parseDouble(pointSTR[1])));
                line = br.readLine();
            }
        } catch(Exception e){
            e.printStackTrace();
        }
        return new LinearInterpolation((Point[])points.toArray());
    }
}
