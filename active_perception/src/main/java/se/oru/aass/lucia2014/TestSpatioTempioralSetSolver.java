package se.oru.aass.lucia2014;

import java.util.HashSet;
import java.util.Random;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.utility.UI.PolygonFrame;

import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;


public class TestSpatioTempioralSetSolver {
	
	public static void main(String[] args) {
		
		//Symbols represent panels seen by robots
		int numPanels = 15;
		String[] panels = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panels[i] = "P"+i;
			symbols[i] = "P"+i;
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";
		
		SpatioTemporalSetNetworkSolver solver = new SpatioTemporalSetNetworkSolver(0,100000,500,symbols);
		ActivityNetworkSolver groundSolver1 = (ActivityNetworkSolver)solver.getConstraintSolvers()[0];
		GeometricConstraintSolver groundSolver2 = (GeometricConstraintSolver)solver.getConstraintSolvers()[1];
		
		//Vars representing robots and what panels (if any) they see
		int numRobots = 20;
		Variable[] robots = new Variable[numRobots];
		for (int i = 0; i < numRobots; i++) robots[i] = solver.createVariable("Robot"+i+" sees");
		
		//Randomly choose robots (as many as there are panels)
		Random rand = new Random(1234431);
		HashSet<Variable> chosenRobots = new HashSet<Variable>();
		for (int i = 0; i < panels.length; i++)
			while (!chosenRobots.add(robots[rand.nextInt(numRobots)])) {}

		//Force every chosen robot to see one of the panels (w/o deciding which panel)		
		Vector<Constraint> cons = new Vector<Constraint>();
		for (Variable robot : chosenRobots) {
			SymbolicValueConstraint con = new SymbolicValueConstraint(Type.VALUESUBSET);
			con.setValue(panels);
			con.setFrom(robot);
			con.setTo(robot);
			cons.add(con);
		}

		//Force all chosen robots to see a different panel
		SymbolicValueConstraint con = new SymbolicValueConstraint(Type.DIFFERENT);
		con.setScope(chosenRobots.toArray(new Variable[chosenRobots.size()]));
		cons.add(con);

		ConstraintNetwork.draw(solver.getConstraintNetwork(),"SpatioTemporalSets");
		
		ConstraintNetwork.draw(groundSolver1.getConstraintNetwork(),"Activities");
		
		//TODO: Will not work because domains of polygons are not instantiated
		ConstraintNetwork.draw(groundSolver2.getConstraintNetwork(),"Polygons");
		
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", groundSolver2.getConstraintNetwork(), 0.1f);
		
		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		System.out.println("Added constraints? " + solver.addConstraints(cons.toArray(new Constraint[cons.size()])));
		System.out.println("Done.");
		
	}


}
