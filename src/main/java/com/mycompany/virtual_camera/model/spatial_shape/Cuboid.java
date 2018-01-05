package com.mycompany.virtual_camera.model.spatial_shape;

import com.mycompany.virtual_camera.model.Edge3D;
import com.mycompany.virtual_camera.model.Point3D;
import java.util.Arrays;

/**
 *
 * @author Paweł Mac
 */
public class Cuboid extends AbstractSpatialShape {
    
    final Point3D p1;
    final Point3D p2;
    final Point3D p3;
    final Point3D p4;
    final Point3D p5;
    final Point3D p6;
    final Point3D p7;
    final Point3D p8;
    
    final Edge3D e1;
    final Edge3D e2;
    final Edge3D e3;
    final Edge3D e4;
    
    final Edge3D e5;
    final Edge3D e6;
    final Edge3D e7;
    final Edge3D e8;
    
    final Edge3D e9;
    final Edge3D e10;
    final Edge3D e11;
    final Edge3D e12;

    public Cuboid(double x, double y, double z, double width, double height, double depth) throws IllegalArgumentException {
        if (Double.compare(width, 0) == 0) {
            throw new IllegalArgumentException("width is 0");
        }
        if (Double.compare(height, 0) == 0) {
            throw new IllegalArgumentException("height is 0");
        }
        if (Double.compare(depth, 0) == 0) {
            throw new IllegalArgumentException("depth is 0");
        }
        this.p1 = new Point3D(x,       y,        z);
        this.p2 = new Point3D(x+width, y,        z);
        this.p3 = new Point3D(x,       y+height, z);
        this.p4 = new Point3D(x+width, y+height, z);
        this.p5 = new Point3D(x,       y,        z+depth);
        this.p6 = new Point3D(x+width, y,        z+depth);
        this.p7 = new Point3D(x,       y+height, z+depth);
        this.p8 = new Point3D(x+width, y+height, z+depth);
        
        this.e1 = new Edge3D(p1, p2);
        this.e2 = new Edge3D(p2, p4);
        this.e3 = new Edge3D(p3, p1);
        this.e4 = new Edge3D(p4, p3);
        
        this.e5 = new Edge3D(p5, p6);
        this.e6 = new Edge3D(p6, p8);
        this.e7 = new Edge3D(p7, p5);
        this.e8 = new Edge3D(p8, p7);
        
        this.e9  = new Edge3D(p1, p5);
        this.e10 = new Edge3D(p2, p6);
        this.e11 = new Edge3D(p3, p7);
        this.e12 = new Edge3D(p4, p8);
        
        Point3D[] point3DsArray = new Point3D[]{p1,p2,p3,p4,p5,p6,p7,p8};
        this.point3DsSet.addAll(Arrays.asList(point3DsArray));
        Edge3D[] edge3DsArray = new Edge3D[]{e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12};
        this.edge3DsSet.addAll(Arrays.asList(edge3DsArray));
    }

    public Cuboid(Point3D first, Point3D second) throws IllegalArgumentException {
        this(
                first.getX(), 
                first.getY(), 
                first.getZ(), 
                second.getX() - first.getX(), 
                second.getY() - first.getY(), 
                second.getZ() - first.getZ()
        );
    }
}
