package com.mycompany.virtual_camera.view;

import com.mycompany.virtual_camera.model.Edge3D;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.util.Collection;
import java.util.Observable;
import java.util.Observer;
import javax.swing.JPanel;

/**
 *
 * @author Paweł Mac
 */
public final class ViewportJPanel extends JPanel implements Observer {
    
    private Collection<Edge3D> edge3DsCollection;
    
    public ViewportJPanel(int width, int height) {
        this.setPreferredSize(new Dimension(width, height));
        this.setBackground(Color.white);
    }
    
    // Setter
    
    public void setEdge3DsCollection(Collection<Edge3D> edge3DsCollection) {
        this.edge3DsCollection = edge3DsCollection;
    }
    
    // Methods
    
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2D = (Graphics2D) g;
        if (edge3DsCollection != null) {
            Path2D path2D = new Path2D.Double();
            for (Edge3D edge3D : edge3DsCollection) {
                /* 1 way
                if (edge3D.getFirst().isInFrontOfViewport() ||  edge3D.getSecond().isInFrontOfViewport()) {
                    g2D.draw(edge3D.getLine2D());
                }
                //*/
                
                //* 2 way
                if (edge3D.getFirstMock().isInFrontOfViewport() &&  edge3D.getSecondMock().isInFrontOfViewport()) {
                    g2D.draw(edge3D.getLine2D());
                }
                //*/
                
                /* 3 way Path2D
                if (edge3D.getFirstMock().isInFrontOfViewport() &&  edge3D.getSecondMock().isInFrontOfViewport()) {
                    path2D.moveTo(edge3D.getLine2D().getX1(), edge3D.getLine2D().getY1());
                    path2D.lineTo(edge3D.getLine2D().getX2(), edge3D.getLine2D().getY2());
                    g2D.draw(path2D);
                }
                //*/
                
                /* 4 way Path2D
                if (edge3D.getFirstMock().isInFrontOfViewport() &&  edge3D.getSecondMock().isInFrontOfViewport()) {
                    path2D.append(edge3D.getLine2D(), false);
                    g2D.draw(path2D);
                }
                //*/
            }
        } else {
            System.out.println("edge3DsCollection == null");
        }
    }
    
    @Override
    public void update(Observable o, Object arg) {
        this.repaint();
    }
}
