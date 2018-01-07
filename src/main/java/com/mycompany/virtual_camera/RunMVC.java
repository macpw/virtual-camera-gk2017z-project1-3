package com.mycompany.virtual_camera;

import com.mycompany.virtual_camera.controller.Controller;
import com.mycompany.virtual_camera.model.Edge3D;
import com.mycompany.virtual_camera.model.Point3D;
import com.mycompany.virtual_camera.model.ViewportModel;
import com.mycompany.virtual_camera.model.spatial_shape.Cuboid;
import com.mycompany.virtual_camera.model.spatial_shape.SpatialShapesCollection;
import com.mycompany.virtual_camera.view.View;
import com.mycompany.virtual_camera.view.ViewportJPanel;
import java.util.Set;
import javax.swing.SwingUtilities;

/**
 *
 * @author Paweł Mac
 */
public class RunMVC implements Runnable {
    
    public RunMVC() {
    }
    
    @Override
    public void run() {
        Cuboid cuboid = new Cuboid(-100, 0, 250, 200, 250, 300);
        SpatialShapesCollection spatialShapesCollection = new SpatialShapesCollection();
        spatialShapesCollection.addSpatialShape(cuboid);
        Set<Point3D> point3DsSet = spatialShapesCollection.getPoint3DsSet();
        Set<Edge3D> edge3DsSet = spatialShapesCollection.getEdge3DsSet();
        ViewportModel viewportModel = new ViewportModel(point3DsSet, edge3DsSet, 200, 600, 400);
        View view = new View(viewportModel.getViewportWidth(), viewportModel.getViewportHeight());
        ViewportJPanel viewportJPanel = view.getViewportJPanel();
        viewportJPanel.setLine2DHoldersCollection(viewportModel.getCollectionOfLine2DHolder());
        viewportModel.addObserver(viewportJPanel);
        Controller controller = new Controller(viewportModel, view);
    }
    
    // Test RunMVC
    public static void main(String[] args) {
        SwingUtilities.invokeLater(new RunMVC());
    }
}
