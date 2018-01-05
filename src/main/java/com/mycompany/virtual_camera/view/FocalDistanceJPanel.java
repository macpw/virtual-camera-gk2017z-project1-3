package com.mycompany.virtual_camera.view;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 *
 * @author Paweł Mac
 */
public final class FocalDistanceJPanel extends JPanel {
    
    private static final int JSLIDER_WIDTH = 400;
    private static final int MINIMUM_VALUE = 10;
    
    private final JLabel focalDistanceJLabel = new JLabel("focal distance");
    private final JSlider focalDistanceJSlider = new JSlider();
    
    public FocalDistanceJPanel() {
        this.focalDistanceJSlider.setMaximum(2000);
        this.focalDistanceJSlider.setMinorTickSpacing(50);
        this.focalDistanceJSlider.setMajorTickSpacing(200);
        this.focalDistanceJSlider.setPaintTicks(true);
        this.focalDistanceJSlider.setPaintLabels(true);
        
        this.focalDistanceJSlider.setPreferredSize(
                new Dimension(JSLIDER_WIDTH, focalDistanceJSlider.getPreferredSize().height)
        );
        this.focalDistanceJSlider.setMinimumSize(
                new Dimension(JSLIDER_WIDTH, focalDistanceJSlider.getPreferredSize().height)
        );
        
        this.focalDistanceJSlider.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                if (((JSlider)e.getSource()).getValue() < MINIMUM_VALUE) {
                    ((JSlider)e.getSource()).setValue(MINIMUM_VALUE);
                }
            }
        });
        
        this.setBorder(new TitledBorder("focal distance"));
        this.setLayout(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();
        gbc.gridx = 0;
        this.add(focalDistanceJLabel, gbc);
        this.add(focalDistanceJSlider, gbc);
    }
    
    // Getters
    
    public JLabel getFocalDistanceJLabel() {
        return focalDistanceJLabel;
    }
    
    public JSlider getFocalDistanceJSlider() {
        return focalDistanceJSlider;
    }
    
    // Test FocalDistanceJPanel
    public static void main(String[] args) {
        JFrame jFrame = new JFrame();
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jFrame.setMinimumSize(new Dimension(120, 90));
        
        JPanel mainJPanel = new JPanel();
        jFrame.add(mainJPanel);
        mainJPanel.add(new FocalDistanceJPanel());
        
        jFrame.pack();
        jFrame.setLocationRelativeTo(null);
        jFrame.setVisible(true);
    }
}
