package se.oru.aass.lucia_meta_csp_lecture.util;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSet;

public class RobotFactory {
	
	private static final int NUM_VERTS = 6;
	private static final float RADIUS = 0.5f;

	public static Variable createSpatioTemporalSetVariable(String id, Vec2 p, float theta, ConstraintSolver cs) {
		Vec2[] verts = new Vec2[NUM_VERTS];
		for (int i = 0; i < NUM_VERTS; i++) {
			verts[i] = new Vec2((float)(RADIUS*Math.cos(2*Math.PI*i/NUM_VERTS+theta)+p.x), (float)(RADIUS*Math.sin(2*Math.PI*i/NUM_VERTS+theta)+p.y));
		}
		Variable ret = (SpatioTemporalSet)cs.createVariable(id);
		((SpatioTemporalSet)ret).getPolygon().setDomain(verts);
		((SpatioTemporalSet)ret).getPolygon().setMovable(true);
		return ret;
	}

}
