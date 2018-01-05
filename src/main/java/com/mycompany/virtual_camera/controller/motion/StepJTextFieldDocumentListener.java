package com.mycompany.virtual_camera.controller.motion;

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
public class StepJTextFieldDocumentListener implements DocumentListener {
    
    private static final int DELAY = 500;
    private final ActionListenerForTimer actionListenerForTimer = new ActionListenerForTimer();
    private final javax.swing.Timer timer = new Timer(DELAY, actionListenerForTimer);
    
    private final ViewportModel viewportModel;
    private final JTextField stepJTextField;
    
    public StepJTextFieldDocumentListener(ViewportModel viewportModel, JTextField stepJTextField) {
        this.viewportModel = viewportModel;
        this.stepJTextField = stepJTextField;
    }
    
    private void doUpdate() {
        try {
            double parseDouble = Double.parseDouble(stepJTextField.getText());
            viewportModel.setStep(parseDouble);
            stepJTextField.setToolTipText("step="+viewportModel.getStep());
        } catch (NumberFormatException nfe) {
            stepJTextField.setBackground(Color.orange);
        }
    }
    
    @Override
    public void insertUpdate(DocumentEvent e) {
        stepJTextField.setBackground(Color.lightGray);
        timer.restart();
    }

    @Override
    public void removeUpdate(DocumentEvent e) {
        stepJTextField.setBackground(Color.lightGray);
        timer.restart();
    }

    @Override
    public void changedUpdate(DocumentEvent e) {
        throw new UnsupportedOperationException("Not supported");
    }
    
    private class ActionListenerForTimer implements ActionListener {
        
        @Override
        public void actionPerformed(ActionEvent e) {
            javax.swing.Timer timer = (javax.swing.Timer)e.getSource();
            timer.stop();
            StepJTextFieldDocumentListener.this.stepJTextField.setBackground(Color.white);
            StepJTextFieldDocumentListener.this.doUpdate();
        }
    }
}
