package solutions;

import java.util.Vector;

import lucia_sim_2014.sendGoal;
import lucia_sim_2014.sendGoalRequest;
import lucia_sim_2014.sendGoalResponse;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class part1a  {


	
	public static void main(String[] args) {
		
		ActivityNetworkSolver ans = new  ActivityNetworkSolver(0, 100000000);
		
		Variable var1 = (SymbolicVariableActivity)ans.createVariable("turtlebot_1");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_to");
		
		Variable var2 = (SymbolicVariableActivity)ans.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_to");
		
		
		AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(3000,APSPSolver.INF));
		release.setFrom(var1);
		release.setTo(var1);
		ans.addConstraint(release);
		
		//adding constraint
		AllenIntervalConstraint overlaps = new AllenIntervalConstraint(AllenIntervalConstraint.Type.OverlappedBy);
		overlaps.setFrom(var2);
		overlaps.setTo(var1);
		ans.addConstraint(overlaps);
		
		
		
		InferenceCallback cb = new InferenceCallback() {
			
			@Override
			public void doInference(long timeNow) {
				// TODO Auto-generated method stub
				
			}
		};
		
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(ans, 1000, cb);
		Vector<DispatchingFunction> dispatches = new Vector<DispatchingFunction>();
		
		DispatchingFunction robot1DispatchingFunction = new DispatchingFunction("turtlebot_1") {
			
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {
			}
		};
		dispatches.add(robot1DispatchingFunction);
		
		DispatchingFunction robot2DispatchingFunction = new DispatchingFunction("turtlebot_2") {
			
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				
			}
		};
		dispatches.add(robot2DispatchingFunction);
		
		animator.addDispatchingFunctions(ans, dispatches.toArray(new DispatchingFunction[dispatches.size()]));
		
		
		
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(ans.getConstraintNetwork(),"Activity network");

		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)ans, new Bounds(0,120000), true, "turtlebot_1", "turtlebot_2");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
		
	}

}
