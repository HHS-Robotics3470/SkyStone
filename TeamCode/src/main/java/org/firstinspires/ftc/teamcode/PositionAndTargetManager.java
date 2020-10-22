package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PositionAndTargetManager {
    /**info found: https://firstinspiresst01.blob.core.windows.net/first-game-changers/ftc/field-setup-guide.pdf starting page 8
     *        and: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-remote-events.pdf starting page 26
     * Coords format: x,y,z
     * Origin (0,0) : center of the 6x6(square) grid
     *      field x bounds [-1.79705m,+1.79705m]
     *      field y bounds [-1.79705m,+1.79705m]
     * z = height from the foam grid
     * x+ = toward red alliance station (right from audience area perspective)
     * x- = toward blue alliance station
     * y+ = toward the tower goal and power shot targets
     * x- = toward audience
     *
     * position of red targets (center of volume) (x,y,z)
     *  - power shot 1:     +0.09525m,      +1.79705m,    +0.784225m
     *  - power shot 2:     +0.28575m,      +1.79705m,    +0.784225m
     *  - power shot 3:     +0.47625m,      +1.79705m,    +0.784225m
     *  - high goal:        +0.8905875m,    +1.79705m,    +0.911225m
     *  - medium goal:      -0.8905875m,    +1.79705m,    +0.6873875m
     *  - low goal:         +0.8905875m,    +1.79705m,    +0.4318m
     *
     * position of blue targets (center of volume) (x,y,z)
     *  - power shot 1:     -0.09525m,      +1.79705m,    +0.784225m
     *  - power shot 2:     -0.28575m,      +1.79705m,    +0.784225m
     *  - power shot 3:     -0.47625m,      +1.79705m,    +0.784225m
     *  - high goal:        -0.8905875m,    +1.79705m,    +0.911225m
     *  - medium goal:      +0.8905875m,    +1.79705m,    +0.6873875m
     *  - low goal:         -0.8905875m,    +1.79705m,    +0.4318m
     *
     * Red starting pos:    +___, -___, 0m
     * blue starting pos:   -___, -___, 0m
     */
    /*
        [r]
        r = 0, power shot 1
        r = 1, power shot 2
        r = 2, power shot 3
        r = 3, high goal
        r = 4, medium goal
        r = 5, low goal
        [c]
        c = 0, x
        c = 1, y
        c = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[][] targets = {
            /*x,y,z*/
            {0.09525,1.79705,0.784225},     /*power shot 1*/
            {0.28575,1.79705,0.784225},     /*power shot 2*/
            {0.47625,1.79705,0.784225},     /*power shot 3*/
            {0.8905875,1.79705,0.911225},   /*high goal*/
            {-0.8905875,1.79705,0.6873875}, /*medium goal*/
            {0.8905875,1.79705,0.4318}      /*low goal*/
    };

    /*
        [r]
        r = 0, x
        r = 1, y
        r = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[] robotPosition = new double[3]; //TODO: 10/18/2020 replace "new double[3];" with the array of the starting position if on red team
    double[] targetPosition = new double[3];
    double launchZone = 2.06375-1.79705/*+- 0.0254m*/;       //any position with a y coordinate less (maybe more than?) than launchZone is in the launch zone

    double heading = 90.0;

    double metersPerRevolution;
    double robotWidth;

    byte powerShotsHit = 0;
    ////////////////////////////// constructors //////////////////////////////
    /**
     * constructor for the PositionAndTargetManager
     * @param isTeamRed pass as true if the robot is on the red alliance
     */
    public PositionAndTargetManager(HardwareUltimateGoal robot, boolean isTeamRed) {
        //take some variables from the robot
        metersPerRevolution = robot.NADO_METERS_PER_REV;
        robotWidth = robot.robotWidth;
        //flip some things for if the robot is on blue team
        if (!isTeamRed) {
            for (int r = 0; r < targets.length; r ++) {
                targets[r][0] *= -1.0; //flip the x-axis if not on the red team
            }

            robotPosition[0] *= -1.0; //flip the x-axis if not on the red team
        }
    }
    ////////////////////////////// update and calculate method //////////////////////////////

    /**
     * the update method takes the number of times that the two drive motors have rotated since the last update, using this it calculates the change in
     * position and heading
     * @param leftRotations     number of times the left motor has rotated since last update
     * @param rightRotations    number of times the right motor has rotated since last update
     */
    public void update(double leftRotations, double rightRotations) {
        double positionChange;
        double headingChange = 0.0;

        //new logic
        double s1 = leftRotations * metersPerRevolution; //distance the left wheel traveled (m)
        double s2 = rightRotations * metersPerRevolution; //distance the right wheel traveled (m)
        double r = 0.0;// = robotWidth; //for case 4
        /*
        4 cases:
        1) equal in distance,       equal in direction      (both sides equal)
        2) not equal in distance,   equal in direction      (both are same direction, one is greater than the other)
        3) equal in distance,       not equal in direction  (one is forward, one is backward)
        4) not equal in distance,   not equal in direction  (one is zero)

        order: 2,4,1,3
         */
        //TODO: 10/20/2020 check if this works when the robot is going backwards too
        if (Math.abs(s1) != Math.abs(s2)) { //are the sides traveling different distances (case 2,4)
            try { //try case 2, if it doesn't work (divide by zero), do case 4      // filters out cases where one side is positive and the other is 0
                if (Math.abs(s1) > Math.abs(s2)) { //the abs should both: allow this to work when the robot is going backwards, and filter out cases where one side is negative and the other is 0
                    r = (s1 - s2) / (s2 * robotWidth); //calculating r for heading calculation
                    headingChange = s2/r;//https://www.desmos.com/calculator/r16kcermq2
                }
                else if (Math.abs(s1) < Math.abs(s2)) {
                    r = (s2 - s1) / (s1 * robotWidth); //calculating r for heading calculation
                    headingChange = s1/r; //calculating headingChange
                }
                r = r + robotWidth/2; // calculating r for position calculation

            } catch (Exception e) { //case 4
                r = robotWidth; //calculating r for heading calculation
                headingChange = s1 / r;
                if (s1 == 0.0) {headingChange = s2 / r;}
                r /= 2.0;// calculating r for position calculation
            }
            positionChange = Math.sqrt( Math.pow((r)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((r)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        }
        else { // going in the same direction (case 1,3)
            positionChange = s1; //case 1
            if (s1 != s2) { //case 3
                if (Math.abs(s1) > Math.abs(s2)) { //turning clockwise
                    r = (robotWidth * s2) / (s1 + s2);
                    headingChange = s1/r; //calculating headingChange
                } else if (Math.abs(s1) < Math.abs(s2)) { //turning counter clockwise
                    r = (robotWidth * s1) / (s2 + s1);
                    headingChange = s2/r; //calculating headingChange
                }
                positionChange = Math.sqrt( Math.pow((r)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((r)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
            }
        }
        //store changes to position and heading
        heading += Math.toDegrees(headingChange);
        robotPosition[0] += Math.cos(heading) * positionChange;
        robotPosition[1] += Math.sin(heading) * positionChange;

        /*old logic VVVVV
        if ((float)leftRotations == (float)rightRotations) { // both sides going either forward or backward same speed;
            positionChange = leftRotations * 2 * Math.PI * wheelRadius;
        }
        else if ( (leftRotations < 0 && rightRotations < 0) || (leftRotations > 0 && rightRotations > 0) ) { //both sides either going forward, or backward (different speed) (coded with forward in mind)
            // variables
            double s1 = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            double s2 = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            double r = 0; //radius of the inner circle

            //calculate r, headingChange, and position change
            if (s1 > s2) {
                r = (s1 - s2) / (s2 * robotWidth); // calculating r
                headingChange = s2/r;//https://www.desmos.com/calculator/r16kcermq2
            }
            else if (s1 < s2) {
                r = (s2 - s1) / (s1 * robotWidth);// calculating r
                headingChange = s1/r; //calculating headingChange
            }
            positionChange = Math.sqrt( Math.pow((r + robotWidth/2)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((r + robotWidth/2)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        }
        else if ((float)leftRotations < 0 || (float)rightRotations < 0) { //one side going forward, other going backward
            //variables
            double s1 = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            double s2 = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            double r; //radius of the inner circle

            //calculate r, headingChange, and position change
            if (Math.abs(s1) > Math.abs(s2)) { //turning clockwise
                r = (robotWidth * s2) / (s1 + s2);
                headingChange = s1/r; //calculating headingChange
            } else { //turning counter clockwise
                r = (robotWidth * s1) / (s2 + s1);
                headingChange = s2/r; //calculating headingChange
            }
            positionChange = Math.sqrt( Math.pow((robotWidth/2 -r)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((robotWidth/2 -r)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        }
        else { //one side isn't moving
            //variables
            double s = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            if (s == 0) {
                s = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            }
            double r = robotWidth;
            headingChange = s/r;
            positionChange = Math.sqrt( Math.pow((robotWidth/2)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((robotWidth/2)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        }
        //store changes to position and heading
        heading += Math.toDegrees(headingChange);
        robotPosition[0] += Math.cos(heading) * positionChange;
        robotPosition[1] += Math.sin(heading) * positionChange;
        //call a method to choose the best target at the moment
        bestTargetPosition(timeElapsed); */
    }

    /**
     * acts as an update method, that also returns the value it is setting the target position to
     * @param timeSeconds  time elapsed during match, measured in seconds, used to differentiate between mid game and end game targets
     * @return the position of the target selected, it also sets targetPosition to these coordinates
     */
    public double[] bestTargetPosition(double timeSeconds) {
        double[] bestTarget;

        int i = (int) (Math.random() * (targets.length - 1)); //random row of targets -1
        if (i < 3 || i >= 5) { //prevents the robot from shooting the powershots before endgame, and increased the chance of aiming to the high goal
            i = 3;
        } // at this point, i should be either 3 or 4
        //TODO: 10/19/2020 change the > to < depending on what the launch zone actually is
        if (robotPosition[1] > launchZone) { // if the robot is outside of the launchZone
            i = 5; //target low goal
        }
        bestTarget = targets[i];

        if (timeSeconds >= 200 && (int) (Math.random()*3 + 1) == 2 && powerShotsHit <= 2) { //if it's the endgame,  and rng (1/3)
            bestTarget = targets[powerShotsHit]; // cycle through the powershots
            powerShotsHit ++;
        }

        targetPosition = bestTarget;
        return targetPosition;
    }
    ////////////////////////////// get methods //////////////////////////////
}