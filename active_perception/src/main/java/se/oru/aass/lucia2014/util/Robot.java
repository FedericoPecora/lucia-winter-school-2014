package se.oru.aass.lucia2014.util;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSet;

public class Robot {

	private String id;
	private SpatioTemporalSet variable;
	private static final int NUM_VERTS = 6; 

	public SpatioTemporalSet getspatioTemporalSet() {
		return variable;
	}

	public Robot(String id, Vec2 p, float radius, float theta, ConstraintSolver cs) {
		Vec2[] verts = new Vec2[NUM_VERTS];
		for (int i = 0; i < NUM_VERTS; i++) {
			verts[i] = new Vec2((float)(radius*Math.cos(2*Math.PI*i/NUM_VERTS+theta)+p.x), (float)(radius*Math.sin(2*Math.PI*i/NUM_VERTS+theta)+p.y));
		}
		this.variable = (SpatioTemporalSet)cs.createVariable(id);
		((SpatioTemporalSet)this.variable).getPolygon().setDomain(verts);
	}

}
