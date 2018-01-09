package com.mycompany.virtual_camera.model;

import java.awt.Color;
import java.awt.geom.Line2D;

/**
 *
 * @author Paweł Mac
 */
public class Edge3D {
    
    private final Point3D first;
    private final Point3D second;
    private Color color;
    private final Point3D firstMock = new Point3D();
    private final Point3D secondMock = new Point3D();
    private final Line2D line2D = new Line2D.Double();
    
    public Edge3D(Point3D first, Point3D second) throws NullPointerException {
        if (first == null) {
            throw new NullPointerException("first point is null");
        }
        if (second == null) {
            throw new NullPointerException("second point is null");
        }
        this.first = first;
        this.second = second;
        this.color = Color.BLACK;
    }
    
    public Edge3D(Point3D first, Point3D second, Color color) throws NullPointerException {
        if (first == null) {
            throw new NullPointerException("first point is null");
        }
        if (second == null) {
            throw new NullPointerException("second point is null");
        }
        this.first = first;
        this.second = second;
        this.color = color;
    }
    
    // Getters and Setters
    
    public Point3D getFirst() {
        return first;
    }
    
    public Point3D getSecond() {
        return second;
    }
    
    public Color getColor() {
        return color;
    }
    
    public void setColor(Color color) {
        this.color = color;
    }
    
    public Point3D getFirstMock() {
        return firstMock;
    }
    
    public Point3D getSecondMock() {
        return secondMock;
    }
    
    public Line2D getLine2D() {
        return line2D;
    }
    
    // Methods
    
    @Override
    public String toString() {
        return "Edge3D{" + "first=" + first + ", second=" + second + ", color=" + color + '}';
    }
}
