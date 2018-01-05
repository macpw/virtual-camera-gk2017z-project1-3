package com.mycompany.virtual_camera.model.spatial_shape;

import com.mycompany.virtual_camera.model.Edge3D;
import com.mycompany.virtual_camera.model.Point3D;
import java.util.HashSet;
import java.util.Set;

/**
 *
 * @author Paweł Mac
 */
public abstract class AbstractSpatialShape {
    
    protected final Set<Point3D> point3DsSet;
    protected final Set<Edge3D> edge3DsSet;
    
    public AbstractSpatialShape() {
        this.point3DsSet = new HashSet<>();
        this.edge3DsSet = new HashSet<>();
    }
    
    public Set<Point3D> getPoint3DsSet() {
        return point3DsSet;
    }
    
    public Set<Edge3D> getEdge3DsSet() {
        return edge3DsSet;
    }
}
