package se.oru.aass.lucia_meta_csp_lecture.solutions;

import java.util.HashSet;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.utility.logging.MetaCSPLogging;

public class Part4 {
	
	public static void main(String[] args) {
		MetaCSPLogging.setLevel(Level.INFO);
		
		//Symbols represent panels seen by robots
		int numPanels = 4;
		String[] panels = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panels[i] = "P"+i;
			symbols[i] = "P"+i;
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";
		
		//We make a constraint network to hold variables representing
		// robots and what they see
		//ActivityNetworkSolver solver = new ActivityNetworkSolver(0, 1000000, symbols);
		SymbolicVariableConstraintSolver setSolver = new SymbolicVariableConstraintSolver(symbols, 100);
		setSolver.setSingleValue(false);
		setSolver.setEnumerateSets(false);
		
		//Vars representing robots and what panels (if any) they see
		int numRobots = 5;
		Variable[] robots = new Variable[numRobots];
		for (int i = 0; i < numRobots; i++) robots[i] = setSolver.createVariable("Robot"+i+" sees");

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
		
		ConstraintNetwork.draw(setSolver.getConstraintNetwork());
		
		System.out.println("Added constraints? " + setSolver.addConstraints(cons.toArray(new Constraint[cons.size()])));
		System.out.println("Done.");
		
		SymbolicValueConstraint aRobotSeesPanel3 = new SymbolicValueConstraint(Type.VALUEEQUALS);
		Variable aRobot = chosenRobots.iterator().next();
		aRobotSeesPanel3.setValue(panels[2]);
		aRobotSeesPanel3.setFrom(aRobot);
		aRobotSeesPanel3.setTo(aRobot);
		setSolver.addConstraint(aRobotSeesPanel3);

		
//		System.out.println(solver.getDescription());

	}

}

