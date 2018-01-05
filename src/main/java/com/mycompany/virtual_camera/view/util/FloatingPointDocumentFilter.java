package com.mycompany.virtual_camera.view.util;

import javax.swing.text.AttributeSet;
import javax.swing.text.BadLocationException;
import javax.swing.text.DocumentFilter;

/**
 *
 * @author Paweł Mac
 */
public class FloatingPointDocumentFilter extends DocumentFilter {
    
    final String regex = "[-+]?\\d*\\.?\\d*";
    
    @Override
    public void replace(FilterBypass fb, int offset, int length, String text, AttributeSet attrs) throws BadLocationException {
        String lastText = fb.getDocument().getText(0, fb.getDocument().getLength());
        String textToCheck = lastText.substring(0, offset) + text + lastText.substring(offset+length, lastText.length());
        if (textToCheck.matches(regex)) {
            super.replace(fb, offset, length, text, attrs);
        }
    }
    
    @Override
    public void remove(FilterBypass fb, int offset, int length) throws BadLocationException {
        String lastText = fb.getDocument().getText(0, fb.getDocument().getLength());
        String textToCheck = lastText.substring(0, offset) + lastText.substring(offset+length, lastText.length());
        if (textToCheck.matches(regex)) {
            super.remove(fb, offset, length);
        }
    }
}
