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
		
		ActivityNetworkSolver activityNetworkSolver = new  ActivityNetworkSolver(0, 100000000);
		
		Variable var1 = (SymbolicVariableActivity)activityNetworkSolver.createVariable("turtlebot_1");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_base");
		
		Variable var2 = (SymbolicVariableActivity)activityNetworkSolver.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_base");
		
		//adding constraints
		AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(3000,APSPSolver.INF));
		release.setFrom(var1);
		release.setTo(var1);
		activityNetworkSolver.addConstraint(release);
		
		
		AllenIntervalConstraint overlaps = new AllenIntervalConstraint(AllenIntervalConstraint.Type.OverlappedBy, new Bounds(5000,APSPSolver.INF), new Bounds(1,APSPSolver.INF), new Bounds(1,APSPSolver.INF));
		overlaps.setFrom(var2);
		overlaps.setTo(var1);
		activityNetworkSolver.addConstraint(overlaps);
		
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(activityNetworkSolver.getConstraintNetwork(),"Activity network");


		
	}

}
