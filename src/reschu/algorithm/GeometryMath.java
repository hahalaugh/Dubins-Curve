package reschu.algorithm;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import java.awt.*;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.Random;

/**
 * Created by hahalaugh on 1/6/15.
 */

public class GeometryMath {
	
    private static double Eps = 1e-2;
    private static double DistanceEps = 1;
    
    private enum Side
    {
        Left, Right, Linear, InverseLinear
    }

    private static double NormalizeDouble(double value)
    {
        return (Math.round(value*100)/100.00) ;
    }

    private static void NormalizeDouble(double[] valueArray)
    {
        for(int i = 0; i < valueArray.length; i++)
        {
            valueArray[i] = NormalizeDouble(valueArray[i]);
        }
    }

    static private Side GetDestHeading(double[] src, double theta, double[] dest)
    {
        double[] pre = {src[0] + 10 * Math.cos(theta), src[1] + 10 * Math.sin(theta)};

        double flag = (src[0] - dest[0]) * (pre[1] - dest[1]) - (src[1] - dest[1]) * (pre[0] - dest[0]);

        if(Math.abs(NormalizeDouble(flag)) < Eps)
        {
            if(Math.abs(NormalizeDouble(GeometryMath.GetDistance(src, pre) + GeometryMath.GetDistance(pre, dest) - GeometryMath.GetDistance(src, dest))) < Eps)
            {
                return Side.Linear;
            }
            else
            {
                return Side.InverseLinear;
            }
        }
        else
        {
            if (flag > 0)
            {
                return Side.Left;
            }
            else
            {
                return Side.Right;
            }
        }
    }

    public static double GetDistance(double[] src, double[] dest)
    {
        double x1 = src[0];
        double y1 = src[1];
        double x2 = dest[0];
        double y2 = dest[1];

        return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    public static double GetRadian(double[] src, double[] dest)
    {
        double[] vect = new double[]{dest[0]-src[0], dest[1] - src[1]};

        double vSin = vect[1] / Math.sqrt(vect[0] * vect[0] + vect[1] * vect[1]);
        double vCos = vect[0] / Math.sqrt(vect[0] * vect[0] + vect[1] * vect[1]);

        //theta from 0-pi
        double theta = Math.acos(vCos);

        //if vSin >= 0, theta belong to 1D/2D. use theta directly. If vSin < 0, theta belong to 3D/4D. Use 360 - theta.
        return (vSin > 0)? NormalizeDouble(theta):NormalizeDouble(2 * Math.PI -theta);
    }

    public static double GetDegree(double[] src, double[] dest)
    {
        return Math.toDegrees(GetRadian(src, dest));
    }

    //[x, y, radian]
    public static ArrayList<double[]> GetLine(double[] src, double[] dest)
    {

        double x1 = src[0];
        double y1 = src[1];

        double x2 = dest[0];
        double y2 = dest[1];

        //Heading angle in this line keeps the same
        double theta = NormalizeDouble(GetRadian(src, dest));

        ArrayList<double[]> result = new ArrayList<double[]>();

        if (NormalizeDouble(Math.abs(x1 - x2)) < Eps)
        {
            if (NormalizeDouble(y2 - y1) > Eps)
            {
                for (int i = (int) y1; i <= y2; i++)
                {
                    result.add(new double[] { NormalizeDouble(x1), NormalizeDouble(i), theta});
                }
            }
            else
            {
                for (int i = (int) y1; i >= y2; i--)
                {
                    result.add(new double[] { NormalizeDouble(x1), NormalizeDouble(i), theta});
                }
            }
        }
        else
        {
            double k = ( y2 -  y1) / ( x2 -  x1);
            double b =  y1 - k *  x1;

            double start, stop, flag;
            if (Math.abs(x1 - x2) > Math.abs(y1 - y2))
            {
                start = x1;
                stop = x2;
                flag = (x1 - x2) / Math.abs(x1 - x2);

                for (int i = (int) start; i != stop; i -= (flag * 1))
                {
                    result.add(new double[] { NormalizeDouble(i), NormalizeDouble(Math.floor(k * i + b)), theta});
                }

                result.add(new double[] { NormalizeDouble(stop), NormalizeDouble(Math.floor(k * stop + b)), theta});
            }
            else
            {
                start = y1;
                stop = y2;
                flag = (y1 - y2) / Math.abs(y1 - y2);

                for (int i = (int) start; i != stop; i -= (flag * 1))
                {
                    result.add(new double[] {  NormalizeDouble((((float) i - b) / k)), NormalizeDouble(i), theta});
                }

                result.add(new double[] { NormalizeDouble((((float) stop - b) / k)),  NormalizeDouble(stop), theta});
            }
        }

        return result;
    }

    public static ArrayList<double[]> GetCircle(double[] center, double radius)
    {
        return GetCircle(center, radius, 0, 2 * Math.PI - 0.01, 2 * Math.PI / 360, false);
    }

    //[x, y, radian]
    public static ArrayList<double[]> GetCircle(double[] center, double radius, double theta, double endTheta, double step, boolean clockWise) throws InvalidParameterException
    {
        ArrayList<double[]> result = new ArrayList<double[]>();

        theta = theta % (2 * Math.PI);
        endTheta = endTheta % (2 * Math.PI);

        if(((theta >= endTheta) && clockWise) || ((theta <= endTheta) && !clockWise))
        {
            if(clockWise)
            {
                for(double i = theta; i > endTheta; i -= step)
                {
                    double[] p = new double[]{NormalizeDouble(center[0] + radius * Math.cos(i)), NormalizeDouble(center[1] + radius * Math.sin(i)), NormalizeDouble(i)};
                    result.add(p);
                }
            }
            else
            {
                for(double i = theta; i < endTheta; i += step)
                {
                    double[] p = new double[]{NormalizeDouble(center[0] + radius * Math.cos(i)), (center[1] + radius * Math.sin(i)), NormalizeDouble(i)};
                    result.add(p);
                }
            }
        }
        else
        {
            throw new InvalidParameterException("direction incorrect, check your import and rotating direction");
        }

        return result;
    }

    public static ArrayList<double[]> GetDubinsPath(double[] src, double[] dest, double radius, double theta, double step)
    {
        ArrayList<double[]> result = new ArrayList<double[]>();
        
        Eps = 2 * step;
        DistanceEps = 2 * radius * step;
        
        Side heading = GetDestHeading(src, theta, dest);
        double[] temp = new double[]{src[0], src[1], theta};
        if(heading == Side.Right || heading == Side.InverseLinear)
        {
            double[] center = GetRightCenter(src, theta, radius);
            double distance = GeometryMath.GetDistance(center, dest);

            if(Math.abs(NormalizeDouble(distance - radius)) <= DistanceEps)
            {
                
                while(true)
                {
                    double[] tt = Turn(temp, center, radius, temp[2], step, true);
                    NormalizeDouble(tt);
                    result.add(tt);
                    temp = tt;

                    if(GetDistance(temp, dest) <= DistanceEps)
                    {
                        return result;
                    }
                }
            }

            if(distance > radius)
            {
                
                while(true)
                {
                    double[] tt = Turn(temp, center, radius, temp[2], step, true);
                    
                    NormalizeDouble(tt);
                    result.add(tt);
                    temp = tt;
                    if(NormalizeDouble(Math.abs(temp[2] - GetRadian(temp, dest))) <= Eps)
                    {
                        result.remove(result.size() - 1);
                        result.addAll(GetLine(temp, dest));
                        return result;
                    }
                }
            }

            if(distance < radius) {
                
                center = GetLeftCenter(src, theta, radius);
                while(true)
                {
                    double[] tt = Turn(temp, center, radius, temp[2], step, false);
                    
                    NormalizeDouble(tt);
                    result.add(tt);
                    temp = tt;

                    if(NormalizeDouble(GetDistance(GetRightCenter(temp, temp[2], radius), dest)) >= radius)
                    {
                        center = GetRightCenter(temp, temp[2], radius);
                        while(true)
                        {
                            
                            double[] ttt = Turn(temp, center, radius, temp[2], step, true);
                            NormalizeDouble(ttt);
                            result.add(ttt);
                            temp = ttt;
                           
                            if(NormalizeDouble(GetDistance(temp, dest)) <= DistanceEps)
                            {
                                return result;
                            }
                        }
                    }
                }
            }
        }
        else if(heading == Side.Left) {

            double[] center = GetLeftCenter(src, theta, radius);
            double distance = GeometryMath.GetDistance(center, dest);

            if(Math.abs(NormalizeDouble(distance - radius)) <= DistanceEps){
                
                while(true)
                {
                    
                    double[] tt = Turn(temp, center, radius, temp[2], step, false);
                    NormalizeDouble(tt);
                    result.add(tt);
                    temp = tt;

                    if(GetDistance(temp, dest)<= DistanceEps)
                    {
                        return result;
                    }
                }
            }

            if (distance > radius) {
                
                //Circle and Line
                while(true)
                {
                    
                    double[] tt = Turn(temp, center, radius, temp[2], step, false);
                    NormalizeDouble(tt);
                    result.add(tt);
                    temp = tt;

                    if(NormalizeDouble(Math.abs(temp[2] - GetRadian(temp, dest))) <= Eps)
                    {
                        result.remove(result.size() - 1);
                        result.addAll(GetLine(temp, dest));
                        return result;
                    }
                }
            }

            if (distance < radius) {
                
                center = GetRightCenter(src, theta, radius);
                while(true) {

                    
                    double[] tt = Turn(temp, center, radius, temp[2], step, true);
                    NormalizeDouble(tt);
                    result.add(tt);
                    temp = tt;

                    if(NormalizeDouble(GetDistance(GetLeftCenter(temp, temp[2], radius), dest)) >= radius)
                    {
                        center = GetLeftCenter(temp, temp[2], radius);
                        while(true)
                        {
                            
                            double[] ttt = Turn(temp, center, radius, temp[2], step, false);
                            NormalizeDouble(ttt);
                            result.add(ttt);
                            temp = ttt;
                            
                            if(GetDistance(temp, dest) <= DistanceEps)
                            {
                                return result;
                            }
                        }
                    }
                }
            }
        }
        else
        {

            result.addAll(GeometryMath.GetLine(src, dest));
            return result;
        }
        return result;
    }

    private static double[] Turn(double[] src, double[] center, double radius, double theta, double step, boolean clockWise)
    {
        step = Math.abs(step);

        double turnFlag = clockWise?-1:1;
        double diff = step * turnFlag;
        double angular = diff + GetRadian(center, src);
        
        double newTheta = theta + diff;
        if(newTheta > 2* Math.PI)
        {
            newTheta = (newTheta % (2 *Math.PI));
        }
        else if(newTheta < 0)
        {
            newTheta += 2*Math.PI;
        }

        return new double[]{NormalizeDouble(center[0] +  radius * Math.cos(angular)), NormalizeDouble(center[1] + radius * Math.sin(angular)), NormalizeDouble(newTheta)};
    }

    public static double[] GetLeftCenter(double[] src, double theta, double radius)
    {
        return GetCenter(src, theta, radius, false);
    }

    public static double[] GetRightCenter(double[] src, double theta, double radius)
    {
        return GetCenter(src, theta, radius, true);
    }

    private static double[] GetCenter(double[] src, double theta, double radius, boolean clockWise)
    {
        double turnFlag = clockWise?-1:1;

        double oX = (src[0] - turnFlag * radius * Math.sin(theta));
        double oY = (src[1] + turnFlag * radius * Math.cos(theta));

        return new double[]{NormalizeDouble(oX), NormalizeDouble(oY)};
    }
    
    public static void main(String[] args)
    {

        java.awt.EventQueue.invokeLater(new Runnable() {

            public void run() {
                new GUITest().setVisible(true);
            }
        });
    }
}

class GUITest extends JFrame {
    private JPanel jPanel2;

    public GUITest() {
        initComponents();
    }

    private void initComponents() {
        jPanel2 = new Panel2();

        jPanel2.setBackground(new java.awt.Color(255, 255, 255));
        jPanel2.setBorder(BorderFactory.createBevelBorder(BevelBorder.RAISED));

        this.setContentPane(jPanel2);
        // be nice to testers..
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        pack();

    }

    class Panel2 extends JPanel {
        Panel2() {
            // set a preferred size for the custom panel.
            setPreferredSize(new Dimension(420, 420));
        }

        @Override
        public void paintComponent(Graphics g) {
            super.paintComponent(g);


            Random r = new Random();
/*
            g.setColor(Color.black);
            //double[] src = new double[] { r.nextInt(400), r.nextInt(400)};
            //double[] dest = new double[] { r.nextInt(400), r.nextInt(400)};

            double[] src = {164,272};
            double[] dest = {155, 280};
            System.out.print(src[0] + "," + src[1] + "-->" + dest[0] + "," + dest[1] + "   ");
            for (double[] p : GeometryMath.GetDubinsPath(src, dest, 8, 0, 0.01))
            {
                //System.out.println(p[0]+ "," + p[1] + "," + p[2]);
                g.drawLine((int) p[0], (int) p[1], (int) p[0], (int) p[1]);
                g.drawLine((int) p[0] + 1, (int) p[1], (int) p[0] + 1, (int) p[1]);
            }

*/

            long startTime=System.currentTimeMillis();


            for (int i = 0; i < 5000; i++) {
                g.setColor(Color.black);
                double[] src = new double[]{r.nextInt(400), r.nextInt(400)};
                double[] dest = new double[]{r.nextInt(400), r.nextInt(400)};
                double radius = (double)r.nextInt(20) + 1;
                double theta = (double)r.nextInt(6);
                //System.out.println(src[0] + "," + src[1] + "-->" + dest[0] + "," + dest[1] + "   " + radius + "  " + theta);
                for (double[] p : GeometryMath.GetDubinsPath(src, dest, radius, theta, 0.01)) {
                    // System.out.println(p[0]+ "," + p[1] + "," + p[2]);
                    g.drawLine((int) p[0], (int) p[1], (int) p[0], (int) p[1]);
                    g.drawLine((int) p[0] + 1, (int) p[1], (int) p[0] + 1, (int) p[1]);
                }

                g.setColor(Color.red);
                g.fillOval((int) src[0], (int) src[1], 5, 5);

                g.setColor(Color.blue);
                g.fillOval((int) dest[0], (int) dest[1], 5, 5);
            }

            long endTime=System.currentTimeMillis();
            System.out.println("Executing "+(endTime-startTime)+"ms");
            
            
        }
    }
}
    //194.0,191.0-->383.0,196.0   3.0  2.0
    //250.0,83.0-->253.0,86.0   3.0  0.0
    //264.0,206.0-->398.0,231.0   3.0  2.0
    //281.0,42.0-->297.0,40.0   11.0  4.0
    //164.0,272.0-->155.0,280.0   8.0  0.0


