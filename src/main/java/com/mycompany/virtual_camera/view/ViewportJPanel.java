package com.mycompany.virtual_camera.view;

import com.mycompany.virtual_camera.model.Line2DHolder;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Collection;
import java.util.Observable;
import java.util.Observer;
import javax.swing.JPanel;

/**
 *
 * @author Paweł Mac
 */
public final class ViewportJPanel extends JPanel implements Observer {
    
    private Collection<Line2DHolder> line2DHoldersCollection;
    
    public ViewportJPanel(int width, int height) {
        this.setPreferredSize(new Dimension(width, height));
        this.setBackground(Color.white);
    }
    
    // Getter and Setter
    
    public Collection<Line2DHolder> getLine2DHoldersCollection() {
        return line2DHoldersCollection;
    }
    
    public void setLine2DHoldersCollection(Collection<Line2DHolder> line2DHoldersCollection) {
        this.line2DHoldersCollection = line2DHoldersCollection;
    }
    
    // Methods
    
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2D = (Graphics2D) g;
        if (line2DHoldersCollection != null) {
            for (Line2DHolder line2DHolder : line2DHoldersCollection) {
                if (line2DHolder.isFirstInFrontOfViewport() || line2DHolder.isSecondInFrontOfViewport()) {
                    g2D.draw(line2DHolder.getLine2D());
                }
            }
        } else {
            System.out.println("line2DHoldersCollection ==  null");
        }
    }
    
    @Override
    public void update(Observable o, Object arg) {
        this.repaint();
    }
}
