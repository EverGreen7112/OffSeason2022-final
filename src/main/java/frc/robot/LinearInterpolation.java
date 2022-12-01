package frc.robot;

import java.util.Arrays;
import java.util.Collections;

public class LinearInterpolation {
    private Point[] m_data;

    public LinearInterpolation(Point... data) {
        try {
            m_data = data.clone();
            for (int i = 0; i < m_data.length; i++) {
                m_data[i] = data[i].clone();

            }
            Arrays.sort(m_data);
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        }
    }

    public double interpolate(double x) {
        if (x > m_data[m_data.length - 1].getX()) {
            return m_data[m_data.length - 1].getY();
        }
        if (x < m_data[0].getX()) {
            return m_data[0].getY();
        }
        int i = 0;
        while (x < m_data[i].getX()) {
            i++;
        }
        double m = (m_data[i].getY() - m_data[i-1].getY()) / (m_data[i].getX() - m_data[i-1].getX());//m is the slope
        double b = m_data[i].getY() - m_data[i].getX()*m;
        return m*x+b;
    }
}
