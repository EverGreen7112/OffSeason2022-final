package frc.robot;

public class Point implements Comparable<Point>, Cloneable{

    private double m_x, m_y;

    public Point(double x, double y){
        m_x = x;
        m_y = y;
    }
    @Override
    protected Point clone() throws CloneNotSupportedException {
        return new Point(m_x, m_y);
    }
    public double getX(){
        return m_x;
    }

    public double getY(){
        return m_y;
    }

    @Override
    public int compareTo(Point p) {
        if(p.m_x < this.m_x){
            return 1;
        }
        if(p.m_x > this.m_x){
            return -1;
        }
        if(p.m_y < this.m_y){
            return 1;
        }
        if(p.m_y > this.m_y){
            return -1;
        }
        return 0;
    }

}
