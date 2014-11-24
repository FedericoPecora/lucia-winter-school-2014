package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.ros.node.ConnectedNode;

public class ROSConstraintNetworkAnimator extends ConstraintNetworkAnimator {

	private ConnectedNode rosNode;
	
	public ROSConstraintNetworkAnimator(ActivityNetworkSolver ans, long period, InferenceCallback cb, ConnectedNode rosNode) {
		super(ans, period, cb);
		this.rosNode = rosNode;
	}
	
	@Override
	protected long getCurrentTimeInMillis() {
		return rosNode.getCurrentTime().totalNsecs()/1000000;
	}	

}
