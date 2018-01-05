package com.mycompany.virtual_camera.view;

import com.mycompany.virtual_camera.view.util.FloatingPointDocumentFilter;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.TitledBorder;
import javax.swing.text.AbstractDocument;
import javax.swing.text.Document;

/**
 *
 * @author Paweł Mac
 */
public final class MotionControlJPanel extends JPanel {
    
    private final JLabel stepJLabel = new JLabel("step:");
    private final JTextField stepJTextField = new JTextField(1);
    
    private final JButton moveForwardJButton  = new JButton("↑");
    private final JButton moveBackwardJButton = new JButton("↓");
    private final JButton moveLeftJButton     = new JButton("←");
    private final JButton moveRightJButton    = new JButton("→");
    private final JButton moveUpwardJButton   = new JButton("↥");
    private final JButton moveDownwardJButton = new JButton("↧");
    
    public MotionControlJPanel() {
        this.stepJTextField.setHorizontalAlignment(JTextField.CENTER);
        Document document = this.stepJTextField.getDocument();
        ((AbstractDocument)document).setDocumentFilter(new FloatingPointDocumentFilter());
        
        this.moveForwardJButton .setToolTipText("move forward");
        this.moveBackwardJButton.setToolTipText("move backward");
        this.moveLeftJButton    .setToolTipText("move left");
        this.moveRightJButton   .setToolTipText("move right");
        this.moveUpwardJButton  .setToolTipText("move upward");
        this.moveDownwardJButton.setToolTipText("move downward");
        
        this.setBorder(new TitledBorder("Motion Control"));
        this.setLayout(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();
        this.add(stepJLabel, gbc);
        gbc.fill = GridBagConstraints.BOTH;
        this.add(stepJTextField, gbc);
        JPanel buttonsJPanel = new JPanel(new GridBagLayout());
        gbc.gridy = 1;
        gbc.gridwidth = 2; // buttonsJPanel width
        this.add(buttonsJPanel, gbc);
        gbc.gridwidth = 1; // back to default
        gbc.gridy = 0;
        gbc.gridx = 1;
        buttonsJPanel.add(moveForwardJButton, gbc);
        gbc.gridx = 3;
        buttonsJPanel.add(moveUpwardJButton, gbc);
        gbc.gridy = 1;
        gbc.gridx = GridBagConstraints.RELATIVE;
        buttonsJPanel.add(moveLeftJButton, gbc);
        buttonsJPanel.add(moveBackwardJButton, gbc);
        buttonsJPanel.add(moveRightJButton, gbc);
        buttonsJPanel.add(moveDownwardJButton, gbc);
    }
    
    // Getters
    
    public JTextField getStepJTextField() {
        return stepJTextField;
    }
    
    public JButton getMoveForwardJButton() {
        return moveForwardJButton;
    }
    
    public JButton getMoveBackwardJButton() {
        return moveBackwardJButton;
    }
    
    public JButton getMoveLeftJButton() {
        return moveLeftJButton;
    }
    
    public JButton getMoveRightJButton() {
        return moveRightJButton;
    }
    
    public JButton getMoveUpwardJButton() {
        return moveUpwardJButton;
    }
    
    public JButton getMoveDownwardJButton() {
        return moveDownwardJButton;
    }
    
    // Test MotionControlJPanel
    public static void main(String[] args) {
        JFrame jFrame = new JFrame();
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jFrame.setMinimumSize(new Dimension(120, 90));
        
        JPanel mainJPanel = new JPanel();
        jFrame.add(mainJPanel);
        mainJPanel.add(new MotionControlJPanel());
        
        jFrame.pack();
        jFrame.setLocationRelativeTo(null);
        jFrame.setVisible(true);
    }
}
