package com.mycompany.virtual_camera.model;

import java.awt.Color;
import java.util.Objects;

/**
 *
 * @author Paweł Mac
 */
public class Edge3D {
    
    private Point3D first;
    private Point3D second;
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
    
    // getters and setters

    public Point3D getFirst() {
        return first;
    }

    public void setFirst(Point3D first) {
        this.first = first;
    }

    public Point3D getSecond() {
        return second;
    }

    public void setSecond(Point3D second) {
        this.second = second;
    }

    public Color getColor() {
        return color;
    }

    public void setColor(Color color) {
        this.color = color;
    }
    
    // methods
    
    public void swapFirstWithSecond() {
        Point3D tmp = first;
        this.first = this.second;
        this.second = tmp;
    }
    
    @Override
    public String toString() {
        return "Edge3D{" + "first=" + first + ", second=" + second + ", color=" + color + '}';
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 97 * hash + Objects.hashCode(this.first);
        hash = 97 * hash + Objects.hashCode(this.second);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Edge3D other = (Edge3D) obj;
        if (!Objects.equals(this.first, other.first)) {
            return false;
        }
        if (!Objects.equals(this.second, other.second)) {
            return false;
        }
        return true;
    }
}
