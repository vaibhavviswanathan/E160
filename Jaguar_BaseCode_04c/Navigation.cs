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
        double desiredX_prev, desiredY_prev, desiredT_prev;


        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

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
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;
        
        // TODO MAKE THESE ACTUAL VALUES
        double std_l = 1.2;
        double std_r = 1.2;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;
        public int numParticles_temp = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 1;
        private double w_tot = 1;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        // ---- previous lab stuff

        DateTime previousTimeL;
        double measured_timestepL;
        DateTime previousTimeR;
        double measured_timestepR;

        // PWM error globals
        double errorLInt = 0;
        double errorRInt = 0;
        double wprevL = 0;
        double wprevR = 0;
        double signalL, signalR;

        double rotRateLest = 0;
        double rotRateRest = 0;

        bool within_tracking = false;

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
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

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
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // stuff from previous labs
            previousTimeL = DateTime.Now;
            previousTimeR = DateTime.Now;
            measured_timestepL = 0;
            measured_timestepR = 0;

            traj_i = 1;

            bool within_tracking = false;


        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
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

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


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

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

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


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


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

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 500)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
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
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // ****************** Additional Student Code: Start ************

            // Students must set motorSignalL and motorSignalR. Make sure
            // they are set between 0 and maxPosOutput. A PID control is
            // suggested.
            

            double Kp_PWM, Ki_PWM, Kd_PWM;
            Kp_PWM = 12; // 8; //  2.25 * 16;
            Ki_PWM = 2; // 10 * 2;
            Kd_PWM = 0.1;
            double N = 30; // filter coefficient

            DateTime currentTime = DateTime.Now;
            double MTSL = (currentTime - previousTimeL).TotalMilliseconds / 1000;
            if (wheelDistanceL != 0 || MTSL > 0.5)
            {
                measured_timestepL = (currentTime - previousTimeL).TotalMilliseconds / 1000;
                previousTimeL = currentTime;
                rotRateLest = (wheelDistanceL / (2 * Math.PI * wheelRadius)) * pulsesPerRotation / measured_timestepL;
            }
            double MTSR = (currentTime - previousTimeR).TotalMilliseconds / 1000;
            if (wheelDistanceR != 0 || MTSR > 0.5)
            {
                measured_timestepR = (currentTime - previousTimeR).TotalMilliseconds / 1000;
                previousTimeR = currentTime;
                rotRateRest = (wheelDistanceR / (2 * Math.PI * wheelRadius)) * pulsesPerRotation / measured_timestepR;
            }
            double errorL = desiredRotRateL - rotRateLest;
            errorLInt += ( wheelDistanceL != 0 || MTSL > 0.5 ) ? errorL * measured_timestepL : 0;
            double DerrorL = (wheelDistanceL != 0 || MTSL > 0.5) ? N * (errorL - wprevL) : 0;
            wprevL += (wheelDistanceL != 0 || MTSL > 0.5) ? DerrorL * measured_timestepL : 0;
            signalL = Kp_PWM * errorL + Ki_PWM * errorLInt + Kd_PWM * DerrorL;

            double errorR = desiredRotRateR - rotRateRest;
            errorRInt += ( wheelDistanceR != 0 || MTSR > 0.5 ) ? errorR * measured_timestepR : 0;
            double DerrorR = (wheelDistanceR != 0 || MTSR > 0.5) ? N * (errorR - wprevR) : 0;
            wprevR += (wheelDistanceR != 0 || MTSR > 0.5) ? DerrorR * measured_timestepR : 0;
            signalR = Kp_PWM * errorR + Ki_PWM * errorRInt + Kd_PWM * DerrorR;
        

            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            motorSignalL = (short)((zeroOutput + desiredRotRateL * 100 + signalL) / (1.8519 / 1.8519));// (zeroOutput + u_L);
            motorSignalR = (short)((zeroOutput - desiredRotRateR * 100 - signalR) / (1.6317 / 1.8519));//(zeroOutput - u_R);

           // motorSignalL = (short) -desiredRotRateL;
            //motorSignalR = desiredRotRateR;

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            double satsignalL = motorSignalL - zeroOutput;
            double satsignalR = motorSignalR - zeroOutput;

            // errorLInt += measured_timestep / Ki_PWM * (satsignalL - motorSignalR);

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
                double centralLaserRange = LaserData[113];
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString()
                    + " " + x_est.ToString() + " " + y_est.ToString() + " " + t_est.ToString();

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

 

            // ****************** Additional Student Code: End   ************                
        }


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
            double beta = -t - alpha + desiredT; //check to be within -pi and pi

            // Threshold errors

            pho = (Math.Abs(pho) < phoTrackingAccuracy) ? 0 : pho;
            alpha = (Math.Abs(alpha) < alphaTrackingAccuracy) ? 0 : alpha;
            beta = (Math.Abs(beta) < betaTrackingAccuracy) ? 0 : beta;

            // Make sure within -pi and pi
            alpha = (alpha < -Math.PI) ? alpha + 2 * Math.PI : ((alpha > Math.PI) ? alpha - 2 * Math.PI : alpha);
            beta = (beta < -Math.PI) ? beta + 2 * Math.PI : ((beta > Math.PI) ? beta - 2 * Math.PI : beta);

            // Check to see if point is behind robot
            bool behindRobot = (Math.Abs(alpha) > Math.PI/2) ? true : false;

            // adjust alpha if point is behind robot
            alpha = (behindRobot) ? -t + Math.Atan2(-delta_y, -delta_x) : alpha;
            alpha = (alpha < -Math.PI) ? alpha + 2 * Math.PI : ((alpha > Math.PI) ? alpha - 2 * Math.PI : alpha);

            // calculate desired velocity 
            double desiredV = (behindRobot) ? -Kpho * pho : Kpho * pho;
            double desiredW = Kalpha * alpha + Kbeta * beta;

            // use different controller if within threshold
            if (Math.Sqrt(Math.Pow(delta_x,2) + Math.Pow(delta_y,2)) < phoTrackingAccuracy)
            {
                within_tracking = true;

            }
            if (within_tracking && desiredX == desiredX_prev && desiredY == desiredY_prev && desiredT == desiredT_prev)
            {
                beta = -t + desiredT;
                beta = (beta < -Math.PI) ? beta + 2 * Math.PI : ((beta > Math.PI) ? beta - 2 * Math.PI : beta);
                double KbetaNew = -6 * Kbeta;
                desiredW = KbetaNew * beta;
                desiredV = 0;
            }
            else
            {
                within_tracking = false;
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
            //lower saturation
            double satWheelVelL = 0;
            double satWheelVelR = 0;

            // min rot rate if trying to move
            if (Math.Abs(desiredWheelVelL) > 0)
                satWheelVelL = (Math.Sign(desiredWheelVelL) * Math.Max((double)Math.Abs(desiredWheelVelL), 0.05));
            if (Math.Abs(desiredWheelVelR) > 0)
                satWheelVelR = (Math.Sign(desiredWheelVelR) * Math.Max((double)Math.Abs(desiredWheelVelR), 0.05));

            if (satWheelVelL != 0 && satWheelVelR != 0)
            {
                if (satWheelVelL / desiredRotRateL > satWheelVelR / desiredRotRateR)
                    satWheelVelR = (desiredWheelVelR * satWheelVelL / desiredWheelVelL);
                else
                    satWheelVelL = (desiredWheelVelL * satWheelVelR / desiredWheelVelR);
            }

            desiredWheelVelL = satWheelVelL;
            desiredWheelVelR = satWheelVelR;

            // upper saturation
            satWheelVelL = Math.Sign(desiredWheelVelL) * Math.Min(Math.Abs(desiredWheelVelL), 0.25);
            satWheelVelR = Math.Sign(desiredWheelVelR) * Math.Min(Math.Abs(desiredWheelVelR), 0.25);
            if (Math.Abs(satWheelVelL) / Math.Abs(desiredWheelVelL) < Math.Abs(satWheelVelR) / Math.Abs(desiredWheelVelR))
                satWheelVelR = desiredWheelVelR * Math.Abs(satWheelVelL) / Math.Abs(desiredWheelVelL);
            else
                satWheelVelL = desiredWheelVelL * Math.Abs(satWheelVelR) / Math.Abs(desiredWheelVelR);
            desiredRotRateL = (short)(satWheelVelL / wheelRadius * pulsesPerRotation / (2 * Math.PI));
            desiredRotRateR = (short)(satWheelVelR / wheelRadius * pulsesPerRotation / (2 * Math.PI));
            
            // desiredRotRateL = (short) (-pulsesPerRotation*2);
            // desiredRotRateR = (short) (-pulsesPerRotation*2);

            desiredX_prev = desiredX;
            desiredY_prev = desiredY;
            desiredT_prev = desiredT;



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
                double dT = Math.Sign(currT1-currT)*0.3/r;

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


                // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        /*
        private void TrackTrajectory()
        {
            // The purpose of this function is to track a trajectory determined by the trajectory_x, trajectory_y array
            // It will do so by altering the value of desired_x, desired_y, desired_t
            double[] trajectory_x = new double[] { 1, 3, 5, 3, 1 };
            double[] trajectory_y = new double[] { 1, 2, 3, 4, 5 };
            int max_i = trajectory_x.Length;
            
            // increment i if within range of 
            double x2 = trajectory_x[traj_i];
            double y2 = trajectory_y[traj_i];
            if(Math.Sqrt( Math.Pow(x-x2,2) + Math.Pow(y-y2,2) ) < phoTrackingAccuracy)
            {
                if(traj_i < max_i - 1) traj_i++;
                x2 = trajectory_x[traj_i];
                y2 = trajectory_y[traj_i];
            }
            double x1 = trajectory_x[traj_i - 1];
            double y1 = trajectory_y[traj_i - 1];
           
            //This function creates a linear trajectory between 2 points
            // Compute the slopes and intercept
            double m = (y2-y1)/(x2-x1);
            double b = y2-m*x2;
            // Calculate closest point to robot
            double m_r = -1/m;
            double b_r = y-m*x;
            double x_close = (b_r-b)/(m_r-m);
            double y_close = m_r*x_close+b_r;
            
            //  TODO Compute dT
            double dx = Math.Sqrt(1/(1+Math.Pow(m, 2)));
            double dy = m*dx;
            //TODO determine new desired state
            desiredX = x_close+dx;
            desiredY = y_close+dy;
            desiredT = Math.Atan2(dy, dx);
        }
         * */
        

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
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x = x + distanceTravelled * Math.Cos(t + angleTravelled/2);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled/2);
            t = t + angleTravelled;
            if (t > Math.PI)
                t = t - 2 * Math.PI;
            else if (t < -Math.PI)
                t = t + 2 * Math.PI;


            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF
            bool weShouldReSample = ((wheelDistanceL != 0) || (wheelDistanceR != 0)) && newLaserData;
            newLaserData = false; // reset newLaserData

            // propogate particles using odomotery
            for (int i = 0; i < numParticles; i++)
            {
                double rand_l = wheelDistanceL * (1 + std_l * RandomGaussian() / w_tot);
                double rand_r = wheelDistanceR * (1 + std_r * RandomGaussian() / w_tot);

                // compute angle and distance travelled
                double randDistance = (rand_r + rand_l) / 2;
                double randAngle = (rand_r - rand_l) / (2 * robotRadius);

                // add this to a particle 
                if (propagatedParticles[i].Equals(null))
                {
                    Console.Write(' ');
                }
                if (particles[i].Equals(null))
                {
                    Console.Write(' ');
                }
                propagatedParticles[i].x = particles[i].x + randDistance * Math.Cos(particles[i].t + randAngle / 2);
                propagatedParticles[i].y = particles[i].y + randDistance * Math.Sin(particles[i].t + randAngle / 2);
                propagatedParticles[i].t = particles[i].t + randAngle;

                // ensures that angle stays with -pi and pi
                if (propagatedParticles[i].t > Math.PI)
                    propagatedParticles[i].t = propagatedParticles[i].t - 2 * Math.PI;
                else if (propagatedParticles[i].t < -Math.PI)
                    propagatedParticles[i].t = propagatedParticles[i].t + 2 * Math.PI;

                

            }


            if (weShouldReSample)
            {
                


                
                for (int i = 0; i < numParticles; i++)
                {
                    CalculateWeight(i);
                }
                

                // resample particles

                int maxSamples = 10;
                int[] sampled_inds = new int[numParticles * maxSamples];

                // double w_tot = 0; // find w_tot
                for (int i = 0; i < numParticles; i++)
                {
                    if (propagatedParticles[i].w > w_tot)
                        w_tot = propagatedParticles[i].w;
                }

                int bufferLimit = 0;
                for (int i = 0; i < numParticles; i++) // sample particles
                {
                    double particlesToAdd_temp = (propagatedParticles[i].w / w_tot);
                    particlesToAdd_temp = (propagatedParticles[i].w / w_tot) * maxSamples;
                    int particlesToAdd = particlesToAdd_temp>0 ? Math.Min((int)(particlesToAdd_temp) + 1, 10) : 0;
                    for (int j = bufferLimit; j < bufferLimit + particlesToAdd; j++)
                        sampled_inds[j] = i;
                    bufferLimit += particlesToAdd;
                }

                for (int i = 0; i < numParticles; i++) // randomly sample from resampled
                {
                    int j = random.Next(0, bufferLimit);
                    int p = sampled_inds[j];
                    particles[i].x = propagatedParticles[p].x;
                    particles[i].y = propagatedParticles[p].y;
                    particles[i].t = propagatedParticles[p].t;
                }
            }
            else
            {
                for (int i = 0; i < numParticles; i++) // randomly sample from resampled
                {
                    particles[i].x = propagatedParticles[i].x;
                    particles[i].y = propagatedParticles[i].y;
                    particles[i].t = propagatedParticles[i].t;
                }
            }

            // average all particle states
            x_est = 0; y_est = 0; t_est = 0;
            for (int i = 0; i < numParticles; i++)
            {
                x_est += particles[i].x / numParticles;
                y_est += particles[i].y / numParticles;
                t_est += particles[i].t / numParticles;
            }
            


            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {
	        double weight = 0;

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated weight. Feel free to use the
	        // function map.GetClosestWallDistance from Map.cs.

            if (inReachableSpace(p))
            {

                double sigma_laser_percent = 0.01; // ( 1%, from datasheet);
                double sigma_wall = 0.03; // cm

                // take 8 arcs of the laser pi/4 apart
                int numArcs = 5;
                for (int i = 0; i < numArcs; i++)
                {
                    int laseriter = (int)Math.Round(((double)i / numArcs) * (LaserData.Length));
                    double laserangle = laserAngles[laseriter];
                    double sensor_measurement = (double)(LaserData[laseriter]) / 1000;
                    double minDist = map.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t - Math.PI / 2 + laserangle);

                    double sigma_laser = Math.Max(0.01, sigma_laser_percent * sensor_measurement);
                    double sigma = 5*Math.Sqrt(Math.Pow(sigma_laser,2) + Math.Pow(sigma_wall,2));

                    double prob = 1 / (Math.Sqrt(2 * Math.PI)) * Math.Exp(-Math.Pow(sensor_measurement - minDist, 2) / (2 * Math.Pow(sigma, 2)));
                    if (sensor_measurement == 6.0) prob /= 1000; // range at infinity is 6.0. getting 6.0 and 6.0 shouldnt give you a perfect match.
                    weight += prob;

                    
                }

            }
            propagatedParticles[p].w = weight;

        }

        bool inReachableSpace(int p)
        {
            double px = propagatedParticles[p].x;
            double py = propagatedParticles[p].y;
            bool IRS = (py < 2.794) && (py > -2.74); 
            IRS &= (px < -3.55/2) ? (py > 0)||(py<-2.74) : true;
            IRS &= (px > 3.55/2) ? (py > 0) || (py < -2.74) : true;
            return true;
        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles()
        {
            numParticles = numParticles_temp;
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX)

            particles[p].x = map.minX + (map.maxX - map.minX) * random.NextDouble();
            particles[p].y = map.minY + (map.maxY - map.minY) * random.NextDouble();
            particles[p].t = -Math.PI + 2 * Math.PI * random.NextDouble();



            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
