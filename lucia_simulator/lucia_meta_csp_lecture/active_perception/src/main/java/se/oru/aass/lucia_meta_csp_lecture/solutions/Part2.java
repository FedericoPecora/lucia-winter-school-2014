package se.oru.aass.lucia_meta_csp_lecture.solutions;

import java.util.Vector;

import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;

import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.utility.UI.PolygonFrame;

public class Part2  {


	
	public static void main(String[] args) {
		
		GeometricConstraintSolver geometricSolver = new GeometricConstraintSolver();
		
		Vec2 p1 = new Vec2(-0.033f, -2.105f);
		Vec2 p2 = new Vec2(-0.311f, -2.463f);
		Variable[] panelVars = PanelFactory.createPolygonVariables("panel1", p1, p2, geometricSolver);
		
		Vec2 robot_center = new Vec2(0.0f, 0.0f);
		Variable turtlebot_1 = RobotFactory.createPolygonVariable("turtlebot_1", robot_center, 0.0f, geometricSolver);
		
		Variable wall = geometricSolver.createVariable("wall");
		Vector<Vec2> vecs1 = new Vector<Vec2>();
		vecs1.add(new Vec2(-2.136f, -0.982f));
		vecs1.add(new Vec2(-3.430f, -0.943f));
		vecs1.add(new Vec2(-3.467f, -1.572f));
		vecs1.add(new Vec2(-2.156f, -1.542f));
		((Polygon)wall).setDomain(vecs1.toArray(new Vec2[vecs1.size()]));
		((Polygon)wall).setMovable(false);
		
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", geometricSolver.getConstraintNetwork());
		
		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		GeometricConstraint inside = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside.setFrom(turtlebot_1);
		inside.setTo(panelVars[0]);
		System.out.println("Added? " + geometricSolver.addConstraint(inside));

		try { Thread.sleep(3000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		GeometricConstraint dc = new GeometricConstraint(GeometricConstraint.Type.DC);
		dc.setFrom(turtlebot_1);
		dc.setTo(wall);
		System.out.println("Added? " + geometricSolver.addConstraint(dc));
		
		
	}

}
