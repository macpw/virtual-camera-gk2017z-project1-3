package com.mycompany.virtual_camera.model;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

/**
 *
 * @author Paweł Mac
 */
public class Point3D {
    
    private final RealMatrix coordinates;// homogeneous coordinates
    
    public Point3D() {
        this.coordinates = MatrixUtils.createColumnRealMatrix(new double[]{0,0,0,1});
    }
    
    public Point3D(double x, double y, double z) {
        this.coordinates = new Array2DRowRealMatrix(new double[]{x,y,z,1});
    }
    
    // Getters and Setters
    
    public RealMatrix getCoordinates() {
        return coordinates;
    }
    
    public void setCoordinates(RealMatrix coordinates) {
        this.coordinates.setColumnMatrix(0, coordinates);
    }
    
    public void setCoordinates(double x, double y, double z) {
        this.coordinates.setEntry(0, 0, x);
        this.coordinates.setEntry(1, 0, y);
        this.coordinates.setEntry(2, 0, z);
    }
    
    public double getX() {
        return coordinates.getEntry(0, 0);
    }
    
    public double getY() {
        return coordinates.getEntry(1, 0);
    }
    
    public double getZ() {
        return coordinates.getEntry(2, 0);
    }
    
    public void setX(double x) {
        coordinates.setEntry(0, 0, x);
    }
    
    public void setY(double y) {
        coordinates.setEntry(1, 0, y);
    }
    
    public void setZ(double z) {
        coordinates.setEntry(2, 0, z);
    }
    
    // Methods
    
    public void normalize() {
        if (coordinates.getEntry(3, 0) != 1.0) {
            double x = coordinates.getEntry(0, 0) / coordinates.getEntry(3, 0);
            double y = coordinates.getEntry(1, 0) / coordinates.getEntry(3, 0);
            double z = coordinates.getEntry(2, 0) / coordinates.getEntry(3, 0);
            double w = coordinates.getEntry(3, 0) / coordinates.getEntry(3, 0);
            coordinates.setEntry(0, 0, x);
            coordinates.setEntry(1, 0, y);
            coordinates.setEntry(2, 0, z);
            coordinates.setEntry(3, 0, w);
        }
    }
    
    @Override
    public String toString() {
        return "Point3D{" + "coordinates=" + coordinates + '}';
    }
}
