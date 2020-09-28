package org.firstinspires.ftc.teamcode;

/**
 * this will be the main class for the aim assist.
 * it should be able to take an input consisting of the robots position, heading, velocity(?), acceleration(?), and the position
 * of the target and be able to output the heading, and pitch required for the robot to launch the ring into the goal
 *
 * parameters with a ? before the description may not be needed.
 *
 * @param   rPosition         a double array with two indexs recording the position of the robot (x,y) relative to the starting position.
 * @param   rHeading              the direction that the robot is facing relative to the starting direction.
 * @param   rVelocity         ?the magnitude of the velocity of the robot
 * @param   rAcceleration     ?the acceleration the robot is undergoing, needed to correct the aim of the turret to anticipate
 *                              the impact the robots acceleration will have on the arc of the projectile
 *                              (may not be needed if the robot stops before firing)
 *
 * @param   tuHeading             ?the direction that the turret is facing relative to the robot
 *
 * @param   tPosition         a double array with three indexs recording the position of the target (x,y,z(up))
 *
 *
 *
 * @return headingToTarget          the heading that the turret needs to point to (relative to the robot) to face the target
 * @return pitchToTarget        the pitch that the turret needs to be at to hit the target.
 */

public class AimAssist {

    //variables defining the robots characteristics
    double[] robotPosition; // x, y, coords of robot, measured in meters
    double robotHeading;        // direction the robot is facing relative to it's starting direction (0), measured in degrees
    double robotVelocity;   // magnitude of the velocity of the robot, measure in meters per second
    double robotAcceleration; // magnitude of the acceleration of the robot, measured in meters per second per second

    //variables defining the turrets characteristics
    double turretHeading; // the direction that the turret is facing relative to the front of the robot (0), measured in degrees

    //variables defining the targets characteristics
    double[] targetPosition; // x, y, z, coords of the robot, measured in meters

    //variables that are calculated
    double headingToTarget; // the direction that the turret needs to face relative to the front of the robot (0), measured in degrees
    double pitchToTarget; // the pitch that the turret needs to be at to hit the target, measured in degrees

    /**
     * the constructors for the turret
     */
    /**default constructor**/
    public AimAssist(){

    }
    /**constructor**/
    public AimAssist(double[] rPosition, double rHeading, double rVelocity, double rAcceleration, double tuHeading, double[] tPosition)
    {
        robotPosition = rPosition;
        robotHeading = rHeading;
        robotVelocity = rVelocity;
        robotAcceleration = rAcceleration;

        turretHeading = tuHeading;

        targetPosition = tPosition;
    }

    /**
     * method to update the the variables to the most recent values
     */
    public void Update(double[] rPosition, double rHeading, double rVelocity, double rAcceleration, double tuHeading, double[] tPosition)
    {
        robotPosition = rPosition;
        robotHeading = rHeading;
        robotVelocity = rVelocity;
        robotAcceleration = rAcceleration;

        turretHeading = tuHeading;

        targetPosition = tPosition;

        headingToTarget = HeadingCalculation();
        pitchToTarget = pitchCalculation();
    }

    /**
     * the method to calculate the heading that the turret needs to point at in order to point to the target
     * @return headingToTarget the heading to the target
     */
    private double HeadingCalculation ()
    {
        //x and y values of the robot
        double rX = robotPosition[0];
        double rY = robotPosition[1];
        //x and y values of the target
        double tX = targetPosition[0];
        double tY = targetPosition[1];
        //x and y values of imaginary triangle with the hypotenuse being the line between the turret and the target
        double x = tX - rX;
        double y = tY -rY;

        // heading relative to the field (-|+)
        double heading;
        heading = Math.atan(y / x);
        heading = heading - turretHeading; // heading relative to the robot

        return heading;
    }

    /**
     * the method to calculate the pitch that the turret needs to be at in order to point to the target
     * @return pitchToTarget the heading to the target
     */
    // TODO: 9/24/2020 adjust method to take air resistance and gravity into effect, need to know: the initial velocity the turret launches at, weight of the rings, and more 
    private double pitchCalculation()
    {
        final double turretHeight = 0.30; //height from the floor of the field to the turret (measured in meters)

        //calculate distance to target
        double x = targetPosition[0] - robotPosition[0];
        double y = targetPosition[1] - robotPosition[1];
        double distance = Math.sqrt( x*x + y*y);

        //calculate height to target
        double height = targetPosition[2] - turretHeight;

        //calculate pitch to target
        return Math.atan(height / distance);
    }
    /**
     * calculations for more accurate pitch calculations
     *
     * knowns:
     *  y displacement needed = height
     *  mass of ring = 0.065lbs = 0.029483504kg
     *  weight of ring (downward acceleration newtons?) = mass * -9.80665m/s/s = -0.2891344045016N
     *  a of y = -9.80665m/s/s
     *
     *  x displacement needed = distance
     *
     *  knowns to calculate initial velocity?
     *      maximum RPM of the motor (tetrix torqueNADO) (no load) = 100 RPM = 5/3 RPS
     *      stall torque of the motor = 700 oz/in. = 4.94308628333331 newton meter
     *
     *      diameter of the flywheels (andymark 4in compliant wheels) = 4in = 0.1016m
     *      circumference of flywheels = 0.1016 * pi = 0.319185813605m
     *      mass of the flywheels:  [0.228, 0.269] Lbs (between those numbers) = [0.1034190604, 0.12201635] kg ~= 0.1127177052 kg
     *      velocity of flywheel at point of contact? = 5/3 rps * (0.1016m * pi)/r = (0.508)*pi/3 m/s = 0.169333...*pi m/s = 0.531976 m/s
     *
     *      torque applied to the wheel = torque of the motor / 2 because 1 motor is driving two outputs = 2.47154314167 newton meter
     *      force at point of contact = torque to wheel / radius = 2.47154314167Nm / 0.1016/2m = 48.6524N
     *
     *
     *  need to: find the pitch
     *  assuming that the fly wheels are able to apply all of their speed to the ring, without the ring slipping, basically, assuming high friction
     *  initial velocity of the ring = velocity of the flywheel at point of contact = 0.531976 m/s
     *
     *  know:
     *  velocity = 0.531976 m/s,
     *  x displacement = distance,
     *  y displacement = height,
     *  x acceleration = 0m/s/s,
     *  y acceleration = -9.8m/s/s
     *
     *  v of x = cos( arcsin( v of y / velocity
     *
     *  time:
     *  d = v * t
     *  t = d/v
     *
     *
     *
     *  if we disregard air resistance, the initial velocity of x = average velocity of x
     *  d = v * t
     *  t = d/v
     *
     *  time = distance / (v of x)
     *
     *  finding initial velocity of y:
     *  height = (initial velocity of y) * t + 0.5 * a * t^2
     *  (initial velocity of y) * t = 0.5 * a * t^2 - height
     *  (initial velocity of y) = (0.5 * a * t^2 - height) / t
     *  (initial velocity of y) = ( t(0.5 * -9.80665m/s/s * t - (height/t)) )/t
     *  (initial velocity of y) = 0.5 * -9.80665m/s/s * t - (height/t)
     *
     *  finding t:
     *  t = sqrt( (2*distance) / (a of x) )
     *  a of x = force at point of contact / m = 48.6524N / 0.029483504kg
     *
     *  initial velocity of y = sin( arccos( (v of x) / v ) )
     */
}
