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
public final class RotationControlJPanel extends JPanel {
    
    private final JLabel angleJLabel = new JLabel("angle:");
    private final JTextField angleJTextField = new JTextField(1);
    
    private final JButton rotateLeftJButton      = new JButton("↶");
    private final JButton rotateRightJButton     = new JButton("↷");
    private final JButton rotateUpwardJButton    = new JButton("⤴");
    private final JButton rotateDownwardJButton  = new JButton("⤵");
    private final JButton rotateTiltLeftJButton  = new JButton("↺");
    private final JButton rotateTiltRightJButton = new JButton("↻");
    
    public RotationControlJPanel() {
        this.angleJTextField.setHorizontalAlignment(JTextField.CENTER);
        Document document = this.angleJTextField.getDocument();
        ((AbstractDocument)document).setDocumentFilter(new FloatingPointDocumentFilter());
        
        this.rotateLeftJButton     .setToolTipText("rotate left");
        this.rotateRightJButton    .setToolTipText("rotate right");
        this.rotateUpwardJButton   .setToolTipText("rotate upward");
        this.rotateDownwardJButton .setToolTipText("rotate downward");
        this.rotateTiltLeftJButton .setToolTipText("rotate tilt left");
        this.rotateTiltRightJButton.setToolTipText("rotate tilt right");
        
        this.setBorder(new TitledBorder("Rotation Control"));
        this.setLayout(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();
        this.add(angleJLabel, gbc);
        gbc.fill = GridBagConstraints.BOTH;
        this.add(angleJTextField, gbc);
        JPanel buttonsJPanel = new JPanel(new GridBagLayout());
        gbc.gridy = 1;
        gbc.gridwidth = 2; // buttonsJPanel width
        this.add(buttonsJPanel, gbc);
        gbc.gridwidth = 1; // back to default
        gbc.gridy = 0;
        buttonsJPanel.add(rotateTiltLeftJButton, gbc);
        buttonsJPanel.add(rotateUpwardJButton, gbc);
        buttonsJPanel.add(rotateTiltRightJButton, gbc);
        gbc.gridy = 1;
        buttonsJPanel.add(rotateLeftJButton, gbc);
        buttonsJPanel.add(rotateDownwardJButton, gbc);
        buttonsJPanel.add(rotateRightJButton, gbc);
    }
    
    // Getters
    
    public JTextField getAngleJTextField() {
        return angleJTextField;
    }
    
    public JButton getRotateLeftJButton() {
        return rotateLeftJButton;
    }
    
    public JButton getRotateRightJButton() {
        return rotateRightJButton;
    }
    
    public JButton getRotateUpwardJButton() {
        return rotateUpwardJButton;
    }
    
    public JButton getRotateDownwardJButton() {
        return rotateDownwardJButton;
    }
    
    public JButton getRotateTiltLeftJButton() {
        return rotateTiltLeftJButton;
    }
    
    public JButton getRotateTiltRightJButton() {
        return rotateTiltRightJButton;
    }
    
    // Test RotationControlJPanel
    public static void main(String[] args) {
        JFrame jFrame = new JFrame();
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jFrame.setMinimumSize(new Dimension(120, 90));
        
        JPanel mainJPanel = new JPanel();
        jFrame.add(mainJPanel);
        mainJPanel.add(new RotationControlJPanel());
        
        jFrame.pack();
        jFrame.setLocationRelativeTo(null);
        jFrame.setVisible(true);
    }
}
