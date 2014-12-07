package se.oru.aass.lucia_meta_csp_lecture.solutions;


import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;


public class Part1a  {


	
	public static void main(String[] args) {
		
		ActivityNetworkSolver temporalSolver = new  ActivityNetworkSolver(0, 100000000);
		
		Variable var1 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_1");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_base");

		//TODO: remove var2
		Variable var2 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_base");
		
		//adding constraints
		AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(3000,APSPSolver.INF));
		release.setFrom(var1);
		release.setTo(var1);
		temporalSolver.addConstraint(release);
		
		//TODO: remove overlaps
		AllenIntervalConstraint overlaps = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Overlaps, new Bounds(5000,APSPSolver.INF), new Bounds(1,APSPSolver.INF), new Bounds(1,APSPSolver.INF));
		overlaps.setTo(var2);
		overlaps.setFrom(var1);
		temporalSolver.addConstraint(overlaps);
		
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(temporalSolver.getConstraintNetwork(),"Activity network");
		
		//TODO: Show timeline


		
	}

}
