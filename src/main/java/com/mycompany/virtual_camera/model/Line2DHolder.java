package com.mycompany.virtual_camera.model;

import java.awt.geom.Line2D;

/**
 *
 * @author Paweł Mac
 */
public class Line2DHolder {
    
    final Line2D line2D;
    boolean  firstInFrontOfViewport;
    boolean secondInFrontOfViewport;
    
    public Line2DHolder(Line2D line2D) {
        this.line2D = line2D;
    }
    
    // Getters and Setters
    
    public Line2D getLine2D() {
        return line2D;
    }
    
    public boolean isFirstInFrontOfViewport() {
        return firstInFrontOfViewport;
    }
    
    public void setFirstInFrontOfViewport(boolean firstInFrontOfViewport) {
        this.firstInFrontOfViewport = firstInFrontOfViewport;
    }
    
    public boolean isSecondInFrontOfViewport() {
        return secondInFrontOfViewport;
    }
    
    public void setSecondInFrontOfViewport(boolean secondInFrontOfViewport) {
        this.secondInFrontOfViewport = secondInFrontOfViewport;
    }
}
