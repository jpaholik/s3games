package s3games.robot;

/** Represents a particular robot location - i.e. two sets of
 * all servo angles. First set for the "above the location" position [0-4],
 * second set for the "at the location" position [5-9]. */
public class RobotLocation
{
    /** the arm has 5 degrees of freedom + the gripper */
    public static final int NUMBER_OF_ROBOT_ANGLES = 10;
    public static final int NUMBER_OF_COORDINATES = 3;
    
    //** position of the arm base in centimeters */
    public static final double ARM_BASE_POSITION_X = 0;
    public static final double ARM_BASE_POSITION_Y = 0;
    public static final double ARM_BASE_POSITION_Z = 5;
    
    /** lengths of arm parts */
    public static final double RAMENO = 14.605;
    public static final double PREDLAKTIE = 18.7325;
    public static final double ZAPASTIE = 8.5725;
    
    public static final double UPPER_FIELD_POSITION = ZAPASTIE;

    /** 2x5 angles */
    public double angles[];
    public double coordinates[];

    /** construct an empty location */
    public RobotLocation()
    {
        angles = new double[NUMBER_OF_ROBOT_ANGLES];
        coordinates = new double[NUMBER_OF_COORDINATES];
    }
    
    /** parse the location specification string as loaded from the game specification file */
    public RobotLocation(String locations) throws Exception
    {
        angles = new double[NUMBER_OF_ROBOT_ANGLES];
        coordinates = new double[NUMBER_OF_COORDINATES];
        String[] angs = locations.split(",");
        
        if (angs.length == angles.length) {
            for (int i = 0; i < angles.length; i++)
                angles[i] = Double.parseDouble(angs[i].trim());
        }
        else if (angs.length == coordinates.length) {
            for (int i = 0; i < coordinates.length; i++)
                coordinates[i] = Double.parseDouble(angs[i].trim());
            
            double[] anglesBottomPosition = getAnglesFromCoordinates(
                coordinates[0],
                coordinates[1],
                coordinates[2]
            );
            
            double[] anglesUpperPosition = getAnglesFromCoordinates(
                coordinates[0],
                coordinates[1],
                coordinates[2] + UPPER_FIELD_POSITION
            );
            
            for(int i = 0; i< angles.length; i++) {
                if(i < 5) {
                    angles[i] = anglesUpperPosition[i];
                }
                else {
                    angles[i] = anglesBottomPosition[i-5];
                }
            }
        }
        else {
            throw new Exception("incorrect specification of robot angles: '" + locations + "'");
        }
    }

    /** make a copy of this location */
    public RobotLocation getCopy()
    {
        RobotLocation copied = new RobotLocation();
        for (int i = 0; i < angles.length; i++)
            copied.angles[i] = angles[i];
        return copied;
    }
    
    /** convert the first part of the location to string for visualization in robot controll window */
    @Override
    public String toString()
    {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < NUMBER_OF_ROBOT_ANGLES / 2 ; i++)
        {
            b.append(angles[i]);
            b.append(" ");
        }
        return b.toString();
    }
    
    /** implementation of inverse kinematics - converts coordinates to arm angles */
    public double[] getAnglesFromCoordinates(double x, double y, double z) throws Exception {
        double pointQX = 0;
        double pointQY = y;
        
        double p = getTwoPointsDistance(pointQX, pointQY, x, y);
        double q = getTwoPointsDistance(ARM_BASE_POSITION_X, ARM_BASE_POSITION_Y, x, y);
        double sinAnglePi = p / q;
        
        /** rotation angle of arm base */
        double anglePi = sinToDegree(sinAnglePi);
        
        double pointCX = x;
        double pointCY = y;
        double pointCZ = z + ZAPASTIE - ARM_BASE_POSITION_Z;
        
        double b = Math.sqrt(Math.pow(q, 2) + Math.pow(pointCZ, 2));
        
        double cosAngleAlpha = ((Math.pow(b, 2) + Math.pow(RAMENO, 2) - Math.pow(PREDLAKTIE, 2)) 
                / (2 * b * RAMENO));
        double cosAngleBeta = ((Math.pow(RAMENO, 2) + Math.pow(PREDLAKTIE, 2) - Math.pow(b, 2)) 
                / (2 * RAMENO * PREDLAKTIE));
        double sinAngleAlphaApostrophe = pointCZ / b;
        double sinAngleGammaApostrophe = q / b;
        
        double angleAlpha = cosToDegree(cosAngleAlpha);
        double angleBeta = cosToDegree(cosAngleBeta);
        double angleGamma = 180 - angleAlpha - angleBeta;
        double angleAlphaApostrophe = sinToDegree(sinAngleAlphaApostrophe);
        double angleGammaApostrophe = sinToDegree(sinAngleGammaApostrophe);
        
        if(x < 0) {
            anglePi = -anglePi;
        }
        double[] anglesToReturn = {
            anglePi, 
            - (90 - (angleAlpha + angleAlphaApostrophe)),
            - (90 - angleBeta),
            - (180 - (angleGamma + angleGammaApostrophe)),
            85 // this can be changed
        };
        
        for(int i = 0; i < anglesToReturn.length; i++) {
            if(Double.isNaN(anglesToReturn[i])) {
                throw new Exception("Unreachable position");
            }
        }
        
        return anglesToReturn;
    }
    
    /** returns distance between two points - [x1, y1] and [x2, y2] */
    private double getTwoPointsDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2)); 
    }
    
    private double sinToDegree(double sinAngle) {
        return Math.asin(sinAngle) * 180.0d / Math.PI;
    }
    
    private double cosToDegree(double cosAngle) {
        return Math.acos(cosAngle) * 180.0d / Math.PI;
    }
}
