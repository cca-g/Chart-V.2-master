using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Threading;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Reactive.Joins;
using Rychusoft.NumericalLibraries.Approximation;
using Accord.Audio.ComplexFilters;
using System.Numerics;
using Accord.Audio;
using MathNet;
using System.Diagnostics;
using MathNet.Numerics.Interpolation;
using Rychusoft.NumericalLibraries.Interpolation;
using CenterSpace.NMath.Core;

namespace plot
{
    /// <summary>
    /// Логика взаимодействия для MainWindow.xaml
    /// </summary>

    public partial class MainWindow : Window
    {

        private Task flow;
        private DispatcherTimer dispatcherTimer;
        private List<double> mx = new List<double>();
        private List<double> time = new List<double>();
        private List<double> timeMin = new List<double>();
        private List<double> min = new List<double>();
        
        private int i = 1;
        private int maxIndex = 0;

        private List<double> x = new List<double>();
       
        private List<double> y = new List<double>();
        private List<int> z = new List<int>();
        private List<int> zmin = new List<int>();
        private bool tumbler = false;
        
        public MainWindow()
        {

            InitializeComponent();
            flow = new Task(chart);
            dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 1 );
        }


        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            LineA.Plot(y.ToArray(), x.ToArray());
            if (x.Count > 5)
            {
                cha();
                LineMAX.Plot(time, mx);
                LineMIN.Plot(timeMin, min);
            }


        }


        private void chart()
        {
            Random g = new Random();
            double w0, w, a, t = 3.2;
            w0 = 3.9;
            double tic = 0.1;
            float c = 0;
            while (tumbler)
            {
                flow.Wait(100);
                w = (2 * Math.PI) / tic;
                a = g.Next(1, 6);
                // x.Add(a * Math.Cos(w * t + w0));
                x.Add(Math.Sin(c));
                y.Add(tic);
                c += 2.5f;
                tic += 0.1;
                if (tic > 25)
                    tumbler = false;
            }
            MessageBox.Show("dead flow");

        }       

        private void cha()
        {
            double max, mi;
            mi = x[i - 1];
            max = x[i - 1];
            bool tum = true;
            for (int j = i; j < x.Count; j++)
            {

                if (mi > x[j] && !tum)
                    mi = x[j];
                if (max < x[j] && tum)
                    max = x[j];

                if (max > x[j] && tum)
                {
                    z.Add(j - 1);
                    mi = x[j];
                    tum = false;
                }
                if (mi < x[j] && !tum)
                {
                    zmin.Add(j - 1);
                    max = x[j];
                    tum = true;
                    maxIndex = j;
                }
            }
            smooth();
            i = maxIndex;

        }
        private void But_Copy_Click(object sender, RoutedEventArgs e)
        {
            tumbler = !tumbler;
            if (tumbler)
            {
                flow.Start();
                dispatcherTimer.Start();
            }
            else
            {
                dispatcherTimer.Stop();
            }
        }

       
        int p = 0;
        int pp = 0;
        void smooth()
        {

             CSpline spline;
            List<CPoint> points = new List<CPoint>();
            for (int j=p;j<z.Count;j++)
            {
                points.Add(new CPoint());
                
                points[j-p].Y = x[z[j]];
                points[j-p].X = y[z[j]] ;
                
            }

            p = z.Count;
            spline = new CSpline((CPoint[]) points.ToArray());

            spline.GenerateSplines();
            spline.Draw(ref mx, ref time);           
            CSpline splinelMIN;
            List<CPoint> pointsMIN = new List<CPoint>();
            for (int j = pp; j < zmin.Count; j++)
            {
                pointsMIN.Add(new CPoint());

                pointsMIN[j - pp].Y = x[zmin[j]];
                pointsMIN[j - pp].X = y[zmin[j]];
            }
            pp = zmin.Count;
            splinelMIN = new CSpline((CPoint[])pointsMIN.ToArray());

            splinelMIN.GenerateSplines();
            splinelMIN.Draw(ref min, ref timeMin);
          
        }           
    }

    class CPoint
    {
        public double X { get; set; }
        public double Y { get; set; }

        public double Df { get; set; }
        public double Ddf { get; set; }

    }
    class CSplineSubinterval
    {
        public double A { get; }
        public double B { get; }
        public double C { get; }
        public double D { get; }

        private readonly CPoint _p1;
        private readonly CPoint _p2;

        public CSplineSubinterval(CPoint p1, CPoint p2, double df, double ddf)
        {
            _p1 = p1;
            _p2 = p2;

            B = ddf;
            C = df;
            D = p1.Y;
            A = (_p2.Y - B * Math.Pow(_p2.X - _p1.X, 2) - C * (_p2.X - _p1.X) - D) / Math.Pow(_p2.X - _p1.X, 3);
        }

        public double F(double x)
        {
            return A * Math.Pow(x - _p1.X, 3) + B * Math.Pow(x - _p1.X, 2) + C * (x - _p1.X) + D;
        }

        public double Df(double x)
        {
            return 3 * A * Math.Pow(x - _p1.X, 2) + 2 * B * (x - _p1.X) + C;
        }

        public double Ddf(double x)
        {
            return 6 * A * (x - _p1.X) + 2 * B;
        }
        public void Draw(ref List<double> mx, ref List<double> time)
        {
          

            for (double k = _p1.X; k < _p2.X; k+=0.05)
            {
                mx.Add(F(k ));
                mx.Add(F(k + 0.05));
                time.Add(k );
                time.Add(k + 0.05);
            }
        }
    }
    class CSpline
    {
        private readonly CPoint[] _points;
        private readonly CSplineSubinterval[] _splines;

        public double Df1
        {
            get { return _points[0].Df; }
            set { _points[0].Df = value; }
        }
        public double Ddf1
        {
            get { return _points[0].Ddf; }
            set { _points[0].Ddf = value; }
        }
        public double Dfn
        {
            get { return _points[_points.Length - 1].Df; }
            set { _points[_points.Length - 1].Df = value; }
        }
        public double Ddfn
        {
            get { return _points[_points.Length - 1].Ddf; }
            set { _points[_points.Length - 1].Ddf = value; }
        }

        public CSpline(CPoint[] points)
        {
            _points = points;
            _splines = new CSplineSubinterval[points.Length - 1];
        }

        public void GenerateSplines()
        {
            const double x1 = 0;
            var y1 = BuildSplines(x1);
            const double x2 = 10;
            var y2 = BuildSplines(x2);

            _points[0].Ddf = -y1 * (x2 - x1) / (y2 - y1);

            BuildSplines(_points[0].Ddf);

            _points[_points.Length - 1].Ddf = _splines[_splines.Length - 1].Ddf(_points[_points.Length - 1].X);
        }

        private double BuildSplines(double ddf1)
        {
            double df = _points[0].Df, ddf = ddf1;
            for (var i = 0; i < _splines.Length; i++)
            {
                _splines[i] = new CSplineSubinterval(_points[i], _points[i + 1], df, ddf);

                df = _splines[i].Df(_points[i + 1].X);
                ddf = _splines[i].Ddf(_points[i + 1].X);

                if (i < _splines.Length - 1)
                {
                    _points[i + 1].Df = df;
                    _points[i + 1].Ddf = ddf;
                }
            }
            return df - Dfn;
        }
        public void Draw(ref List<double> mx,  ref List<double> time)
        {
            foreach (var spline in _splines)
            {
                spline.Draw( ref mx, ref time );
            }
            
        }
    }
}
  