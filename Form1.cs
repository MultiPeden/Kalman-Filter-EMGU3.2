using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;

using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.UI;
using Emgu.CV.Util;


namespace Kalman_Filter
{
    public partial class Form1 : Form
    {
        #region Variables
        float px, py, cx, cy, ix, iy;
        float ax, ay;
        #endregion

        #region Kalman Filter and Poins Lists
        PointF[] oup = new PointF[2];
        private KalmanFilter kal;
        private SyntheticData syntheticData;
        private List<PointF> mousePoints;
        private List<PointF> kalmanPoints;
        #endregion

        #region Timers
        Timer MousePositionTaker = new Timer();
        Timer KalmanOutputDisplay = new Timer();
        #endregion

        public Form1()
        {
            InitializeComponent();
            KalmanFilter(); //initialize kalman filter
        }

        //Record mouse position when over the specific tracking area
        private void MouseTrackingArea_MouseMove(object sender, MouseEventArgs e)
        {
            ax = e.X;// store mouse locations over picturebox in avriables
            ay = e.Y;
            MouseLive_LBL.Text = "Mose Position Live- X:" + ax.ToString() + " Y:" + ay.ToString();
        }

        private void KalmanFilterRunner(object sender, EventArgs e)
        {
            //Graphics G = Graphics.FromImage(pictureBox1.Image);
            //Image<Bgr, Byte> gridx = grid; //Not needed
            PointF inp = new PointF(ix, iy);

            oup = new PointF[2];

            oup = filterPoints(inp);

            PointF[] pts = oup;

            MouseCorrected_LBL.Text = "Mouse Position Corrected- X:" + cx.ToString() + " Y:" + cy.ToString();
            MousePredicted_LBL.Text = "Mouse Position Predicted- X:" + px.ToString() + " Y:" + py.ToString();


            Graphics G = MouseTrackingArea.CreateGraphics();

            G.FillEllipse(Brushes.Magenta, cx, cy, 5, 5);

            G.FillEllipse(Brushes.Cyan, px, py, 5, 5);

            G.FillEllipse(Brushes.Blue, ix, iy, 5, 5);

            // Action<PointF, Bgr> drawCross =
            // delegate(PointF point, Bgr color)
            // {
            //     gridx.Draw(new Cross2DF(point, 15, 15), color, 1);
            //};

            //drawCross(inp, new Bgr(Color.Black)); //draw current state in White
            // drawCross(oup[1], new Bgr(Color.Red)); //draw the measurement in Red
            // drawCross(oup[0], new Bgr(Color.Blue)); //draw the prediction (the next state) in green
            //gridx.Draw(new LineSegment2DF(inp, oup[0]), new Bgr(Color.Magenta), 1); //Draw a line between the current position and prediction of next position

            //pictureBox1.Image = gridx.ToBitmap();


        }

        private void MousePositionRecord(object sender, EventArgs e)
        {
            Random rand = new Random();
            ix = (int)ax;
            iy = (int)ay;
            MouseTimed_LBL.Text = "Mouse Position Timed- X:" + ix.ToString() + " Y:" + iy.ToString();
        }

        public PointF[] filterPoints(PointF pt)
        {
            syntheticData.state[0, 0] = pt.X;
            syntheticData.state[1, 0] = pt.Y;

            Mat prediction = kal.Predict();


            float[] data = GetMatData(prediction);


            PointF predictPoint = new PointF(data[0], data[1]);
            PointF measurePoint = new PointF(syntheticData.GetMeasurement()[0, 0],
            syntheticData.GetMeasurement()[1, 0]);

            Mat estimated = kal.Correct(syntheticData.GetMeasurement().Mat);
            data = GetMatData(estimated);
            PointF estimatedPoint = new PointF(data[0], data[1]);
            syntheticData.GoToNextState();
            PointF[] results = new PointF[2];
            results[0] = predictPoint;
            results[1] = estimatedPoint;
            px = predictPoint.X;
            py = predictPoint.Y;
            cx = estimatedPoint.X;
            cy = estimatedPoint.Y;
            return results;
        }

        public void KalmanFilter()
        {
            mousePoints = new List<PointF>();
            kalmanPoints = new List<PointF>();
            kal = new KalmanFilter(4, 2, 0);
            syntheticData = new SyntheticData();
            Matrix<float> state = new Matrix<float>(new float[]
            {
                    0.0f, 0.0f, 0.0f, 0.0f
            });
            /*
            kal.CorrectedState = state;
            kal.TransitionMatrix = syntheticData.transitionMatrix;
            kal.MeasurementNoiseCovariance = syntheticData.measurementNoise;
            kal.ProcessNoiseCovariance = syntheticData.processNoise;
            kal.ErrorCovariancePost = syntheticData.errorCovariancePost;
            kal.MeasurementMatrix = syntheticData.measurementMatrix;
            */
            state.Mat.CopyTo(kal.StatePre);
            syntheticData.transitionMatrix.Mat.CopyTo(kal.TransitionMatrix);    // kal.TransitionMatrix = syntheticData.transitionMatrix;
            syntheticData.measurementNoise.Mat.CopyTo(kal.MeasurementNoiseCov); // kal.MeasurementNoiseCovariance = syntheticData.measurementNoise;
            syntheticData.processNoise.Mat.CopyTo(kal.ProcessNoiseCov);         // kal.ProcessNoiseCovariance = syntheticData.processNoise;
            syntheticData.errorCovariancePost.Mat.CopyTo(kal.ErrorCovPost);     // kal.ErrorCovariancePost = syntheticData.errorCovariancePost;
            syntheticData.measurementMatrix.Mat.CopyTo(kal.MeasurementMatrix);  // kal.MeasurementMatrix = syntheticData.measurementMatrix;





        }

        //Initialse Kalman Filter and Timers
        private void Start_BTN_Click(object sender, EventArgs e)
        {
            if (Start_BTN.Text == "Start")
            {
                MouseTrackingArea.Refresh();
                InitialiseTimers(100);
                Start_BTN.Text = "Stop";
            }
            else
            {
                StopTimers();
                Start_BTN.Text = "Start";
            }
        }
        private void InitialiseTimers(int Timer_Interval = 1000)
        {
            MousePositionTaker.Interval = Timer_Interval;
            MousePositionTaker.Tick += new EventHandler(MousePositionRecord);
            MousePositionTaker.Start();
            KalmanOutputDisplay.Interval = Timer_Interval;
            KalmanOutputDisplay.Tick += new EventHandler(KalmanFilterRunner);
            KalmanOutputDisplay.Start();
        }
        private void StopTimers()
        {
            MousePositionTaker.Tick -= new EventHandler(MousePositionRecord);
            MousePositionTaker.Stop();
            KalmanOutputDisplay.Tick -= new EventHandler(KalmanFilterRunner);
            KalmanOutputDisplay.Stop();
        }


        private float[] GetMatData(Mat mat)
        {
            float[] data = new float[mat.Width * mat.Height];
            Marshal.Copy(mat.DataPointer, data, 0, mat.Width * mat.Height);
            mat.Dispose();
            return data;
        }


    }
}
