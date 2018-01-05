package com.mycompany.virtual_camera.model.spatial_shape;

/**
 *
 * @author Paweł Mac
 */
public class SpatialShapesCollection extends AbstractSpatialShape {
    
    public SpatialShapesCollection() {
        super();
    }
    
    public void addSpatialShape(AbstractSpatialShape spatialShape) {
        this.point3DsSet.addAll(spatialShape.getPoint3DsSet());
        this.edge3DsSet.addAll(spatialShape.getEdge3DsSet());
    }
}
