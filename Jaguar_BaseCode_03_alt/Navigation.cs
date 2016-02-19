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
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;

        // global variables forwardTest
        bool going_forward = true;
        bool increment_distance = false;
        double x_initial = 0;
        double t_initial = 0;
        bool need_to_wait = false;
        double time1 = 0;
        double dist_to_travel = Math.PI / 18;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 8;//4
        private double Kbeta = -0.5;//-1.0;
        const double alphaTrackingAccuracy = 0.20;
        const double betaTrackingAccuracy = 0.10;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;
        DateTime previousTime;
        double measured_timestep;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        // PWM error globals
        private double int_error_L = 0;
        private double int_error_R = 0;

        // TrackTrajectory variables
        private int traj_i = 0;

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

            previousTime = DateTime.Now;

            traj_i = 0;
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
                // functions to call here. For lab 3, we just call the function
                // FlyToSetPoint().
                
                // Update Sensor Readings
                UpdateSensorMeasurements();
                DateTime currentTime = DateTime.Now;
                measured_timestep = (currentTime - previousTime).TotalMilliseconds / 1000;
                previousTime = currentTime;

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

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
                    //WallPositioning();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Determine the desired PWM signals for desired wheel speeds
                    CalcMotorSignals();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    ActuateMotorsWithPWMControl();
                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
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

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

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

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();

                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }


        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds.
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // ****************** Additional Student Code: Start ************

            // Students must set motorSignalL and motorSignalR. Make sure
            // they are set between 0 and maxPosOutput. A PID control is
            // suggested.

            double Kp_PWM, Ki_PWM;
            Kp_PWM = 1;
            Ki_PWM = 1;

            double rotRateLest = (wheelDistanceL / wheelRadius)/measured_timestep;
            double rotRateRest = (wheelDistanceR / wheelRadius) / measured_timestep;

            double errorL = desiredRotRateL - rotRateLest;
            double errorR = desiredRotRateR - rotRateRest;


            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

           // motorSignalL = (short) -desiredRotRateL;
            //motorSignalR = desiredRotRateR;

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            // ****************** Additional Student Code: End   ************

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
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
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString();

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

            pho = (Math.Abs(pho) < phoTrackingAccuracy) ? 0 : pho;
            alpha = (Math.Abs(alpha) < alphaTrackingAccuracy) ? 0 : alpha;
            beta = (Math.Abs(beta) < betaTrackingAccuracy) ? 0 : beta;

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

            // use different controller if within threshold
            if (Math.Abs(delta_x) < phoTrackingAccuracy && Math.Abs(delta_y) < phoTrackingAccuracy)
            {
                beta = -t - desiredT;
                beta = (beta < -Math.PI) ? beta + 2 * Math.PI : ((beta > Math.PI) ? beta - 2 * Math.PI : beta);
                double KbetaNew = -Kbeta * 2;
                desiredW = KbetaNew * beta;
                desiredV = 0;

            }
                

            // saturate desired velocity
            // double saturatedV1 = Math.Sign(desiredV) * Math.Min(Math.Abs(desiredV), 0.25);
            // double saturatedW1 = saturatedV1 > 0 ? desiredW * (saturatedV1 / desiredV) : desiredW;

            double L = 2*robotRadius;
            // double saturatedW = Math.Sign(saturatedW1) * Math.Min(Math.Abs(saturatedW1) * L, 0.25);
            // double saturatedV = saturatedW > 0 ? saturatedV1 * (saturatedW / saturatedW1) : saturatedV1;
            double saturatedV = desiredV;
            double saturatedW = desiredW;


            // desired wheel velocities
            double desiredWheelVelL = -(L * saturatedW - saturatedV);
            double desiredWheelVelR = (L * saturatedW + saturatedV);
            double satWheelVelL = Math.Sign(desiredWheelVelL) * Math.Min(Math.Abs(desiredWheelVelL), 0.25);
            double satWheelVelR = Math.Sign(desiredWheelVelR) * Math.Min(Math.Abs(desiredWheelVelR), 0.25);
            if (Math.Abs(satWheelVelL) / Math.Abs(desiredWheelVelL) < Math.Abs(satWheelVelR) / Math.Abs(desiredWheelVelR))
                satWheelVelR = desiredWheelVelR * Math.Abs(satWheelVelL) / Math.Abs(desiredWheelVelL);
            else
                satWheelVelL = desiredWheelVelL * Math.Abs(satWheelVelR) / Math.Abs(desiredWheelVelR);
            desiredRotRateL = (short)(satWheelVelL / wheelRadius * pulsesPerRotation / (2 * Math.PI));
            desiredRotRateR = (short)(satWheelVelR / wheelRadius * pulsesPerRotation / (2 * Math.PI));



            // ****************** Additional Student Code: End   ************
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            // The purpose of this function is to track a trajectory determined by the trajectory_x, trajectory_y array
            // It will do so by altering the value of desired_x, desired_y, desired_t
            double[] trajectory_x = new double[] { 1, 3, 5, 3, 1 };
            double[] trajectory_y = new double[] { 1, 2, 3, 4, 5 };
            int max_i = trajectory_x.Length;
            
            // increment i if within range of 
            double x1 = trajectory_x[traj_i];
            double y1 = trajectory_y[traj_i];
            if(Math.Sqrt( Math.Pow(x-x1,2) + Math.Pow(y-y1,2) ) < phoTrackingAccuracy)
            {
                if(traj_i < max_i - 1) traj_i++;
                x1 = trajectory_x[traj_i];
                y1 = trajectory_y[traj_i];
            }
            // if at least two more points to hit
            if (traj_i < max_i - 1)
            {
                double x2 = trajectory_x[traj_i + 1];
                double y2 = trajectory_y[traj_i + 1];

                // TODO Vai's code to set desiredX, desiredY, desiredT
                //This function creates a smooth circular trajectory between 3 points
                // Compute the slopes of line A (p1 - p2) and line B (p2 - p3)
                double ma = (y1-y)/(x1-x);
                double mb = (y2-y1)/(x2-x1);

                //compute center of circle
                double xc = (ma*mb*(y-y2)+mb*(x+x1)-ma*(x1+x2))/(2*(mb-ma));
                double yc = -1/ma * (xc-(x+x1)/2)+(y+y1)/2; 

                //compute radius of circle
                double r = Math.Sqrt(Math.Pow(xc-x,2)+Math.Pow(yc-y,2));

                // Compute dT
                double currT = Math.Atan2(y - yc, x - xc);
                double currT1 = Math.Atan2(y1 - yc, x1 - xc);
                double dT = Math.Sign(currT1-currT)*0.5/r;

                //determine new desired state
                desiredX = r * Math.Cos(currT + dT) + xc;
                desiredY = r * Math.Sin(currT + dT) + yc;
                desiredT = Math.Atan2(desiredY-yc, desiredX-xc);
            }
            else // just try to go to the last point
            {
                desiredX = x1;
                desiredY = y1;
                desiredT = 0;
            }





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
        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // above.

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
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************


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
