package se.oru.aass.lucia2014;

import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.PolygonFrame;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;

import se.oru.aass.lucia2014.meta.spaceTimeSets.AssignmentMetaConstraint;
import se.oru.aass.lucia2014.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia2014.meta.spaceTimeSets.SimpleMoveBasePlanner;
import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;


public class TestGeometricMetaConstraint {
	
	private static void setupPanels(String[] panels, GeometricConstraintSolver geometricSolver) {
		
		//Make 2 polygons for each panel
		for (int i = 0; i < panels.length; i++) {
			Variable[] geoms = geometricSolver.createVariables(2,"Viewpoints " + panels[i]);
			Vector<Vec2> vecs1 = new Vector<Vec2>();
			vecs1.add(new Vec2(-10+(10*i),-10));
			vecs1.add(new Vec2(-7+(10*i),-3));
			vecs1.add(new Vec2(-1+(10*i),-1));
			vecs1.add(new Vec2(0+(10*i),-10));
			((Polygon)geoms[0]).setDomain(vecs1.toArray(new Vec2[vecs1.size()]));
			((Polygon)geoms[0]).setMovable(false);
			
			Vector<Vec2> vecs2 = new Vector<Vec2>();
			vecs2.add(new Vec2(-10+(10*i),10));
			vecs2.add(new Vec2(-7+(10*i),3));
			vecs2.add(new Vec2(1+(10*i),1));
			vecs2.add(new Vec2(0+(10*i),10));
			((Polygon)geoms[1]).setDomain(vecs2.toArray(new Vec2[vecs2.size()]));
			((Polygon)geoms[1]).setMovable(false);
		}
		
	}
	
	private static void setRobotPositions(Variable[] robots) {
		for (Variable v : robots) {
			Polygon robot = ((SpatioTemporalSet)v).getPolygon();
			Vector<Vec2> vecs = new Vector<Vec2>();
			vecs.add(new Vec2(0,0));
			vecs.add(new Vec2(4,0));
			vecs.add(new Vec2(4,4));
			vecs.add(new Vec2(0,4));
			robot.setDomain(vecs.toArray(new Vec2[vecs.size()]));
			robot.setMovable(true);
		}
	}
	
	public static void main(String[] args) {
		
		//MetaCSPLogging.setLevel(Level.FINE);
		//Symbols represent panels seen by robots
		int numPanels = 1;
		String[] panels = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panels[i] = "P"+i;
			symbols[i] = "P"+i;
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";
		
		LuciaMetaConstraintSolver metaSolver = new LuciaMetaConstraintSolver(0,100000,500,symbols);
		SpatioTemporalSetNetworkSolver spatioTemporalSetSolver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		ActivityNetworkSolver activitySolver = spatioTemporalSetSolver.getActivitySolver();
		GeometricConstraintSolver geometricSolver = spatioTemporalSetSolver.getGeometricSolver();
		SymbolicVariableConstraintSolver setSolver = spatioTemporalSetSolver.getSetSolver();
		
		setupPanels(panels, geometricSolver);
		
		//Vars representing robots and what panels (if any) they see
		Vector<Constraint> initialCondition = new Vector<Constraint>();
		int numRobots = 1;
		String[] robotTimelines = new String[numRobots];
		Variable[] robots = new Variable[numRobots];
		for (int i = 0; i < numRobots; i++) {
			robotTimelines[i] = "State of Robot"+i;
			robots[i] = spatioTemporalSetSolver.createVariable(robotTimelines[i]);
			SymbolicValueConstraint seesNothing = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
			robots[i].setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
			seesNothing.setValue("None");
			seesNothing.setFrom(robots[i]);
			seesNothing.setTo(robots[i]);
			initialCondition.add(seesNothing);
			((SpatioTemporalSet)robots[i]).setTask("Observe");
			AllenIntervalConstraint dur = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3000,APSPSolver.INF));
			dur.setFrom(robots[i]);
			dur.setTo(robots[i]);
			initialCondition.add(dur);
		}
		
		setRobotPositions(robots);
		
		spatioTemporalSetSolver.addConstraints(initialCondition.toArray(new Constraint[initialCondition.size()]));
		
		AssignmentMetaConstraint mc1 = new AssignmentMetaConstraint(null, null);
		mc1.setPanels(panels);
		metaSolver.addMetaConstraint(mc1);
		
		SimpleMoveBasePlanner mc2 = new SimpleMoveBasePlanner(null, null);
		metaSolver.addMetaConstraint(mc2);
		
		metaSolver.backtrack();
		
		System.out.println("Done!");
		
		ConstraintNetwork.draw(spatioTemporalSetSolver.getConstraintNetwork(), "SpatioTemporalSet network");
		ConstraintNetwork.draw(activitySolver.getConstraintNetwork(),"Activity network");
		ConstraintNetwork.draw(geometricSolver.getConstraintNetwork(),"Polygon network");
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", geometricSolver.getConstraintNetwork());

		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)activitySolver, new Bounds(0,6000), true, robotTimelines);
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tp.publish(true, false);
//		tv.startAutomaticUpdate(1000);

	}


}
