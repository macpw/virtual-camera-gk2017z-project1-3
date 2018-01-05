package com.mycompany.virtual_camera.controller.rotation;

import com.mycompany.virtual_camera.model.ViewportModel;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JTextField;
import javax.swing.Timer;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

/**
 *
 * @author Paweł Mac
 */
public class AngleJTextFieldDocumentListener implements DocumentListener {
    
    private static final int DELAY = 500;
    private final ActionListenerForTimer actionListenerForTimer = new ActionListenerForTimer();
    private final javax.swing.Timer timer = new Timer(DELAY, actionListenerForTimer);
    
    private final ViewportModel viewportModel;
    private final JTextField angleJTextField;
    
    public AngleJTextFieldDocumentListener(ViewportModel viewportModel, JTextField angleJTextField) {
        this.viewportModel = viewportModel;
        this.angleJTextField = angleJTextField;
    }
    
    private void doUpdate() {
        try {
            double parseDouble = Double.parseDouble(angleJTextField.getText());
            viewportModel.setAngleInDegrees(parseDouble);
            angleJTextField.setToolTipText("angle="+viewportModel.getAngleInDegrees());
        } catch (NumberFormatException nfe) {
            angleJTextField.setBackground(Color.orange);
        }
    }
    
    @Override
    public void insertUpdate(DocumentEvent e) {
        angleJTextField.setBackground(Color.lightGray);
        timer.restart();
    }

    @Override
    public void removeUpdate(DocumentEvent e) {
        angleJTextField.setBackground(Color.lightGray);
        timer.restart();
    }

    @Override
    public void changedUpdate(DocumentEvent e) {
        throw new UnsupportedOperationException("Not supported");
    }
    
    private class ActionListenerForTimer implements ActionListener {
        
        @Override
        public void actionPerformed(ActionEvent e) {
            timer.stop();
            angleJTextField.setBackground(Color.white);
            doUpdate();
        }
    }
}
