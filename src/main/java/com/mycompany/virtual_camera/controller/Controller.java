package com.mycompany.virtual_camera.controller;

import com.mycompany.virtual_camera.controller.motion.MoveBackwardAction;
import com.mycompany.virtual_camera.controller.motion.MoveDownwardAction;
import com.mycompany.virtual_camera.controller.motion.MoveForwardAction;
import com.mycompany.virtual_camera.controller.motion.MoveLeftAction;
import com.mycompany.virtual_camera.controller.motion.MoveRightAction;
import com.mycompany.virtual_camera.controller.motion.MoveUpwardAction;
import com.mycompany.virtual_camera.controller.motion.StepJTextFieldDocumentListener;
import com.mycompany.virtual_camera.controller.rotation.AngleJTextFieldDocumentListener;
import com.mycompany.virtual_camera.controller.rotation.RotateDownwardAction;
import com.mycompany.virtual_camera.controller.rotation.RotateLeftAction;
import com.mycompany.virtual_camera.controller.rotation.RotateRightAction;
import com.mycompany.virtual_camera.controller.rotation.RotateTiltLeftAction;
import com.mycompany.virtual_camera.controller.rotation.RotateTiltRightAction;
import com.mycompany.virtual_camera.controller.rotation.RotateUpwardAction;
import com.mycompany.virtual_camera.model.ViewportModel;
import com.mycompany.virtual_camera.view.View;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.KeyStroke;

/**
 *
 * @author Paweł Mac
 */
public class Controller {
    
    private final ViewportModel viewportModel;
    private final View view;
    
    public Controller(ViewportModel viewportModel, View view) {
        this.viewportModel = viewportModel;
        this.view = view;
        this.addListenerToFocalDistanceJPanel();
        this.addMotionActions();
        this.addRotationActions();
    }
    
    private void addListenerToFocalDistanceJPanel() {
        JLabel focalDistanceJLabel = view.getFocalDistanceJPanel().getFocalDistanceJLabel();
        JSlider focalDistanceJSlider = view.getFocalDistanceJPanel().getFocalDistanceJSlider();
        focalDistanceJLabel.setText(Integer.toString((int)viewportModel.getFocalDistance()));
        focalDistanceJSlider.setValue((int)viewportModel.getFocalDistance());
        ChangeListenerForFocalDistanceJSlider changeListenerForFocalDistanceJSlider = new ChangeListenerForFocalDistanceJSlider(viewportModel, focalDistanceJLabel);
        focalDistanceJSlider.addChangeListener(changeListenerForFocalDistanceJSlider);
    }
    
    private void addMotionActions() {
        JTextField stepJTextField = view.getMotionControlJPanel().getStepJTextField();
        stepJTextField.setText(Double.toString(viewportModel.getStep()));
        stepJTextField.setToolTipText("step="+viewportModel.getStep());
        StepJTextFieldDocumentListener stepJTextFieldDocumentListener = new StepJTextFieldDocumentListener(viewportModel, stepJTextField);
        stepJTextField.getDocument().addDocumentListener(stepJTextFieldDocumentListener);
        
        // get buttons
        JButton moveForwardJButton  = view.getMotionControlJPanel().getMoveForwardJButton();
        JButton moveBackwardJButton = view.getMotionControlJPanel().getMoveBackwardJButton();
        JButton moveLeftJButton     = view.getMotionControlJPanel().getMoveLeftJButton();
        JButton moveRightJButton    = view.getMotionControlJPanel().getMoveRightJButton();
        JButton moveUpwardJButton   = view.getMotionControlJPanel().getMoveUpwardJButton();
        JButton moveDownwardJButton = view.getMotionControlJPanel().getMoveDownwardJButton();
        
        // create actions
        MoveForwardAction  moveForwardAction  = new MoveForwardAction(viewportModel);
        MoveBackwardAction moveBackwardAction = new MoveBackwardAction(viewportModel);
        MoveLeftAction     moveLeftAction     = new MoveLeftAction(viewportModel);
        MoveRightAction    moveRightAction    = new MoveRightAction(viewportModel);
        MoveUpwardAction   moveUpwardAction   = new MoveUpwardAction(viewportModel);
        MoveDownwardAction moveDownwardAction = new MoveDownwardAction(viewportModel);
        
        // add actions
        moveForwardJButton .addActionListener(moveForwardAction);
        moveBackwardJButton.addActionListener(moveBackwardAction);
        moveLeftJButton    .addActionListener(moveLeftAction);
        moveRightJButton   .addActionListener(moveRightAction);
        moveUpwardJButton  .addActionListener(moveUpwardAction);
        moveDownwardJButton.addActionListener(moveDownwardAction);
        
        // bind key with actions
        KeyStroke upKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_UP, 0);
        moveForwardJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(upKeyStroke, upKeyStroke.toString());
        moveForwardJButton.getActionMap().put(upKeyStroke.toString(), moveForwardAction);
        KeyStroke downKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_DOWN, 0);
        moveBackwardJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(downKeyStroke, downKeyStroke.toString());
        moveBackwardJButton.getActionMap().put(downKeyStroke.toString(), moveBackwardAction);
        KeyStroke leftKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, 0);
        moveLeftJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(leftKeyStroke, leftKeyStroke.toString());
        moveLeftJButton.getActionMap().put(leftKeyStroke.toString(), moveLeftAction);
        KeyStroke rightKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, 0);
        moveRightJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(rightKeyStroke, rightKeyStroke.toString());
        moveRightJButton.getActionMap().put(rightKeyStroke.toString(), moveRightAction);
        KeyStroke ctrlUpKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_UP, InputEvent.CTRL_DOWN_MASK);
        moveUpwardJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(ctrlUpKeyStroke, ctrlUpKeyStroke.toString());
        moveUpwardJButton.getActionMap().put(ctrlUpKeyStroke.toString(), moveUpwardAction);
        KeyStroke ctrlDownKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_DOWN, InputEvent.CTRL_DOWN_MASK);
        moveDownwardJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(ctrlDownKeyStroke, ctrlDownKeyStroke.toString());
        moveDownwardJButton.getActionMap().put(ctrlDownKeyStroke.toString(), moveDownwardAction);
    }
    
    private void addRotationActions() {
        JTextField angleJTextField = view.getRotationControlJPanel().getAngleJTextField();
        angleJTextField.setText(Double.toString(viewportModel.getAngleInDegrees()));
        angleJTextField.setToolTipText("angle="+viewportModel.getAngleInDegrees());
        AngleJTextFieldDocumentListener angleJTextFieldDocumentListener = new AngleJTextFieldDocumentListener(viewportModel, angleJTextField);
        angleJTextField.getDocument().addDocumentListener(angleJTextFieldDocumentListener);
        
        // get buttons
        JButton rotateLeftJButton      = view.getRotationControlJPanel().getRotateLeftJButton();
        JButton rotateRightJButton     = view.getRotationControlJPanel().getRotateRightJButton();
        JButton rotateUpwardJButton    = view.getRotationControlJPanel().getRotateUpwardJButton();
        JButton rotateDownwardJButton  = view.getRotationControlJPanel().getRotateDownwardJButton();
        JButton rotateTiltLeftJButton  = view.getRotationControlJPanel().getRotateTiltLeftJButton();
        JButton rotateTiltRightJButton = view.getRotationControlJPanel().getRotateTiltRightJButton();
        
        // create actions
        RotateLeftAction      rotateLeftAction      = new RotateLeftAction(viewportModel);
        RotateRightAction     rotateRightAction     = new RotateRightAction(viewportModel);
        RotateUpwardAction    rotateUpwardAction    = new RotateUpwardAction(viewportModel);
        RotateDownwardAction  rotateDownwardAction  = new RotateDownwardAction(viewportModel);
        RotateTiltLeftAction  rotateTiltLeftAction  = new RotateTiltLeftAction(viewportModel);
        RotateTiltRightAction rotateTiltRightAction = new RotateTiltRightAction(viewportModel);
        
        // add actions
        rotateLeftJButton     .addActionListener(rotateLeftAction);
        rotateRightJButton    .addActionListener(rotateRightAction);
        rotateUpwardJButton   .addActionListener(rotateUpwardAction);
        rotateDownwardJButton .addActionListener(rotateDownwardAction);
        rotateTiltLeftJButton .addActionListener(rotateTiltLeftAction);
        rotateTiltRightJButton.addActionListener(rotateTiltRightAction);
        
        // bind key with actions
        KeyStroke shiftLeftKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, InputEvent.SHIFT_DOWN_MASK);
        rotateLeftJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(shiftLeftKeyStroke, shiftLeftKeyStroke.toString());
        rotateLeftJButton.getActionMap().put(shiftLeftKeyStroke.toString(), rotateLeftAction);
        KeyStroke shiftRightKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, InputEvent.SHIFT_DOWN_MASK);
        rotateRightJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(shiftRightKeyStroke, shiftRightKeyStroke.toString());
        rotateRightJButton.getActionMap().put(shiftRightKeyStroke.toString(), rotateRightAction);
        KeyStroke shiftUpKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_UP, InputEvent.SHIFT_DOWN_MASK);
        rotateUpwardJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(shiftUpKeyStroke, shiftUpKeyStroke.toString());
        rotateUpwardJButton.getActionMap().put(shiftUpKeyStroke.toString(), rotateUpwardAction);
        KeyStroke shiftDownKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_DOWN, InputEvent.SHIFT_DOWN_MASK);
        rotateDownwardJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(shiftDownKeyStroke, shiftDownKeyStroke.toString());
        rotateDownwardJButton.getActionMap().put(shiftDownKeyStroke.toString(), rotateDownwardAction);
        KeyStroke shiftCtrlLeftKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, InputEvent.SHIFT_DOWN_MASK | InputEvent.CTRL_DOWN_MASK);
        rotateTiltLeftJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(shiftCtrlLeftKeyStroke, shiftCtrlLeftKeyStroke.toString());
        rotateTiltLeftJButton.getActionMap().put(shiftCtrlLeftKeyStroke.toString(), rotateTiltLeftAction);
        KeyStroke shiftCtrlRightKeyStroke = KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, InputEvent.SHIFT_DOWN_MASK | InputEvent.CTRL_DOWN_MASK);
        rotateTiltRightJButton.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(shiftCtrlRightKeyStroke, shiftCtrlRightKeyStroke.toString());
        rotateTiltRightJButton.getActionMap().put(shiftCtrlRightKeyStroke.toString(), rotateTiltRightAction);
    }
}
