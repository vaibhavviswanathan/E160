using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;


namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 50;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double maxVelocity = 0.10;
        const double Kpho = 2;
        const double Kalpha = 4;
        const double Kbeta = -1.0;//-1;
        const double alphaTrackingAccuracy = 0.05;
        const double betaTrackingAccuracy = 0.05;
        const double phoTrackingAccuracy = 0.10;
        double desiredPos = 5;
        double time = 0;
        DateTime startTime;

        // global variables forwardTest
        bool going_forward = true;
        bool increment_distance = false;
        double x_initial = 0;
        double t_initial = 0;
        bool need_to_wait = false;
        double time1 = 0;
        double dist_to_travel = Math.PI / 18;


        #endregion


        #region Navigation Setup

        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            this.Initialize();

            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            // global variables forwardTest
            going_forward = true;
            increment_distance = false;
            x_initial = 0;
            t_initial = 0;
            need_to_wait = false;
            time1 = 0;
            dist_to_travel = Math.PI / 18;


        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).

                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                //LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    ActuateMotors();
                }

                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }

        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;
                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotors()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                String newData = time.ToString() + " " + x + " " + y + " " + t + " " + wheelDistanceL + " " + wheelDistanceR;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control
            /*
            double desiredPos = 1.0;
            double calibratedPos = centralLaserRange * 0.0009844 - 0.2005;
            double error = (calibratedPos - desiredPos);
            double k = 100;
            double signal = error * k;
            
            signal = Math.Min(200, Math.Max(-200, signal));

            motorSignalR = (short)signal; //desiredWheelSpeedL
            motorSignalL = (short)signal; //desiredWheelSpeedL'
            */


            if (increment_distance)
            {
                dist_to_travel = (dist_to_travel < 8 * Math.PI) ? dist_to_travel + Math.PI / 18 : 50;
                increment_distance = false;
            }

            if (need_to_wait)
            {
                TimeSpan ts2 = DateTime.Now - startTime;
                double time2 = ts2.TotalSeconds;
                if (time2 - time1 > 10) need_to_wait = false;
            }
            else if (dist_to_travel < 50) rotationTest(dist_to_travel);

        }

        // This function is called to test the forward odometry
        private void forwardTest(double distance)
        {
            // compute target
            double xtarget = going_forward ? x_initial + distance : x_initial - distance;
            // move to target
            if (Math.Abs(xtarget - x) < 0.1)
            {
                x_initial = x;
                if (!going_forward) increment_distance = true;
                going_forward = !going_forward;

                // stop motors
                motorSignalL = 0;
                motorSignalR = 0;

                // tell controller to wait 10 seconds
                TimeSpan ts1 = DateTime.Now - startTime;
                time1 = ts1.TotalSeconds;
                need_to_wait = true;

            }
            else {
                // proportional control loop
                double error = x - xtarget;
                double k = -100;
                double signal = error * k;

                signal = Math.Min(200, Math.Max(-200, signal));

                motorSignalR = (short)signal; //desiredWheelSpeedR
                motorSignalL = (short)signal; //desiredWheelSpeedL
            }


        }

        // This function is called to test the forward odometry
        private void rotationTest(double distance)
        {
            // compute target
            double t_target = -distance;
            while (t_target < 0)
            {
                t_target = t_target + 2 * Math.PI;
            }
            t_target = (t_target > Math.PI * 2) ? t_target % (2 * Math.PI) : t_target;
            t_est = (t < 0) ? t + 2 * Math.PI : t;

            // move to target
            if (Math.Abs(t_target - t_est) < (3.0 / 180.0) * Math.PI)
            {
                // stop motors
                motorSignalL = 0;
                motorSignalR = 0;

                // tell controller to wait 10 seconds
                TimeSpan ts1 = DateTime.Now - startTime;
                time1 = ts1.TotalSeconds;
                need_to_wait = true;
                increment_distance = true;

            }
            else {
                // proportional control loop
                double error = t_est - t_target;
                double k = 500;
                double signal = Math.Abs(error) * k;

                signal = Math.Min(200, Math.Max(-200, signal));

                motorSignalR = (short)20; //desiredWheelSpeedR
                motorSignalL = (short)(300); //desiredWheelSpeedL
            }


        }



        // ****************** Additional Student Code: End   ************                



        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Calculate delta X and delta Y
            double delta_x = desiredX - x;
            double delta_y = desiredY - y;

            // Calculate state
            double pho = Math.Sqrt(Math.Pow(delta_x, 2)+ Math.Pow(delta_y,2));
            double alpha = -t + Math.Atan2(delta_y, delta_x); //check to be within -pi and pi
            double beta = -t - alpha - desiredT; //check to be within -pi and pi

            // Threshold errors
            double epsilon_pho = 0.1; // m
            double epsilon_alpha = 2 * (Math.PI / 180);
            double epsilon_beta = epsilon_alpha;

            pho = (Math.Abs(pho) < epsilon_pho) ? 0 : pho;
            alpha = (Math.Abs(alpha) < epsilon_alpha) ? 0 : alpha;
            beta = (Math.Abs(beta) < epsilon_beta) ? 0 : beta;

            // Make sure within -pi and pi
            alpha = (alpha < -Math.PI) ? alpha + 2 * Math.PI : ((alpha > Math.PI) ? alpha - 2 * Math.PI : alpha);
            beta = (beta < -Math.PI) ? beta + 2 * Math.PI : ((beta > Math.PI) ? beta - 2 * Math.PI : beta);

            // Check to see if point is behind robot
            bool behindRobot = (Math.Abs(alpha) > Math.PI/2) ? true : false;
            Console.WriteLine(behindRobot);

            // adjust alpha if point is behind robot
            alpha = (behindRobot) ? -t + Math.Atan2(-delta_y, -delta_x) : alpha;
            alpha = (alpha < -Math.PI) ? alpha + 2 * Math.PI : ((alpha > Math.PI) ? alpha - 2 * Math.PI : alpha);

            // calculate desired velocity 
            double desiredV = (behindRobot) ? -Kpho * pho : Kpho * pho;
            double desiredW = Kalpha * alpha + Kbeta * beta;

            // saturate desired velocity
            double saturatedV = Math.Sign(desiredV) * Math.Max(Math.Abs(desiredV), 0.25);
            // scale down W if V was saturated 
            double saturatedW = saturatedV > 0 ? desiredW * (saturatedV / desiredV) : desiredW;

            // desired wheel velocities
            double L = 2*robotRadius;
            double desiredRotRateL = (L * saturatedW - saturatedV) / wheelRadius;
            double desiredRotRateR = (L * saturatedW + saturatedV) / wheelRadius;

            // rad/s to encoder pulses per second and send to motors
            motorSignalL = (short)(-desiredRotRateL * pulsesPerRotation / (2*Math.PI));
            motorSignalR = (short)(desiredRotRateR * pulsesPerRotation / (2 * Math.PI));



            // ****************** Additional Student Code: End   ************
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            // compute diffEncoderPulses
            double diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            double diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;

            // set previous encoder values
            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;

            // correct for rollover
            diffEncoderPulseL = diffEncoderPulseL < -encoderMax / 2 ?
                diffEncoderPulseL + encoderMax : diffEncoderPulseL;
            diffEncoderPulseR = diffEncoderPulseR < -encoderMax / 2 ?
                diffEncoderPulseR + encoderMax : diffEncoderPulseR;
            diffEncoderPulseL = diffEncoderPulseL > encoderMax / 2 ?
                diffEncoderPulseL - encoderMax : diffEncoderPulseL;
            diffEncoderPulseR = diffEncoderPulseR > encoderMax / 2 ?
                diffEncoderPulseR - encoderMax : diffEncoderPulseR;

            // reverse direction for right side
            diffEncoderPulseR = -diffEncoderPulseR;

            // compute distance travelled in timestep
            wheelDistanceR = (diffEncoderPulseR * 2 * Math.PI * wheelRadius) / pulsesPerRotation; //check wheel r
            wheelDistanceL = (diffEncoderPulseL * 2 * Math.PI * wheelRadius) / pulsesPerRotation; //check wheel r

            // compute angle and distance travelled
            distanceTravelled = (wheelDistanceR + wheelDistanceL) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius); //check robot radius






            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x = x + distanceTravelled * Math.Cos(t + angleTravelled);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled);
            t = t + angleTravelled;
            if (t > Math.PI)
                t = t - 2 * Math.PI;
            else if (t < -Math.PI)
                t = t + 2 * Math.PI;

            Console.WriteLine(x);
            Console.WriteLine(y);
            Console.WriteLine(t);


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}