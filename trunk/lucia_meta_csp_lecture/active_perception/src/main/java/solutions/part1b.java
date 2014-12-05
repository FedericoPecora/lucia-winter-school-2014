package solutions;

import java.util.Vector;

import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;

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
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSDispatchingFunction;

public class part1b  extends AbstractNodeMain {


	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	@Override
	public void onStart(ConnectedNode cn) {
		
		this.connectedNode = cn;
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
		
		ROSDispatchingFunction robotDispatchingFunction1 = new ROSDispatchingFunction("turtlebot_1", ans, this.connectedNode) {
			
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {

				ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
				try { serviceClient = connectedNode.newServiceClient(robot+"/sendGoal", sendGoal._TYPE); }
				catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
				final sendGoalRequest request = serviceClient.newMessage();
				request.setX(0.0);
				request.setY(0.0);
				request.setTheta(0.0);
				request.setRotationAfter((byte)0);
				serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {

					@Override
					public void onSuccess(sendGoalResponse arg0) {
						System.out.println("Goal sent");

					}

					@Override
					public void onFailure(RemoteException arg0) { }
				});
 
			}
		};
		dispatches.add(robotDispatchingFunction1);
		
		ROSDispatchingFunction robotDispatchingFunction2 = new ROSDispatchingFunction("turtlebot_2", ans, this.connectedNode) {
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {

				ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
				try { serviceClient = connectedNode.newServiceClient(robot+"/sendGoal", sendGoal._TYPE); }
				catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
				final sendGoalRequest request = serviceClient.newMessage();
				request.setX(0.3);
				request.setY(0.3);
				request.setTheta(0.3);
				request.setRotationAfter((byte)0);
				serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {

					@Override
					public void onSuccess(sendGoalResponse arg0) {
						System.out.println("Goal sent");

					}

					@Override
					public void onFailure(RemoteException arg0) { }
				});
			}
		};
		dispatches.add(robotDispatchingFunction2);
		
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
