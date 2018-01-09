package com.mycompany.virtual_camera.model;

import java.awt.Color;

/**
 *
 * @author Paweł Mac
 */
public class Edge3D {
    
    private final Point3D first;
    private final Point3D second;
    private Color color;
    
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
    
    // Methods
    
    @Override
    public String toString() {
        return "Edge3D{" + "first=" + first + ", second=" + second + ", color=" + color + '}';
    }
}
