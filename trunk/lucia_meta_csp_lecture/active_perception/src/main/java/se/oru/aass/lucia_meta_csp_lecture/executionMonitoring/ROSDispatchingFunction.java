package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;

public class ROSDispatchingFunction extends DispatchingFunction {
	
	private SpatioTemporalSetNetworkSolver solver;
	private SymbolicVariableActivity currentAct = null;
	private boolean isExecuting = false;

	public ROSDispatchingFunction(String component, SpatioTemporalSetNetworkSolver solver) {
		super(component);
		this.solver = solver;
		ROSTopicListener tl = new ROSTopicListener(component, this);
	}
	
	public void finishCurrentActivity() {
		this.finish(currentAct);
		currentAct = null;
	}
	
	public boolean isExecuting() {
		return isExecuting;
	}
	
	public void setExecuting(boolean exec) {
		isExecuting = exec;
	}

	@Override
	public void dispatch(SymbolicVariableActivity act) {
		
		currentAct = act;
		
		String command = act.getSymbolicVariable().getSymbols()[0];
		String location = "";
		
		//Get the two constraint networks (one is low level, one is hi level)
		ConstraintNetwork activityNetwork = this.getConstraintNetwork();
		ConstraintNetwork spatioTemporalSetNetwork = solver.getConstraintNetwork();
		
		//Find observe activity to which this move_base is leading
		Constraint[] cons = activityNetwork.getOutgoingEdges(act);
		SymbolicVariableActivity observeAct = null;
		for (Constraint con : cons) {
			if (((SymbolicVariableActivity)con.getScope()[1]).getSymbolicVariable().getSymbols()[0].equals("Observe")) {
				observeAct = (SymbolicVariableActivity)con.getScope()[1];
				break;
			}
		}
		
		//Find polygon from whcih to compute destiantion
		Polygon destPoly = null;
		for (Variable var : spatioTemporalSetNetwork.getVariables()) {
			SpatioTemporalSet sts = (SpatioTemporalSet)var;
			if (sts.getActivity().equals(observeAct)) {
				destPoly = sts.getPolygon();
				break;
			}
		}
		
		Vec2 destination = destPoly.getPosition();
		location = "" + destination;
		
		isExecuting = true;
		System.out.println(">>>> TO ROS: " + command + " " + destination);

	}

}
