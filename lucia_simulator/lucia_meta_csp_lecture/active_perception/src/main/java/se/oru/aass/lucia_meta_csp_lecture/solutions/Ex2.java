package se.oru.aass.lucia_meta_csp_lecture.solutions;



import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;

import org.apache.commons.logging.Log;
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

public class Ex2  extends AbstractNodeMain {


	private ConnectedNode connectedNode;
	private final String nodeName = "Ex2";
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	@Override
	public void onStart(ConnectedNode cn) {
		
		this.connectedNode = cn;
		
		//waiting for ConnectedNode to be up in order to get ROS current time
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");
		
		
		long timeNow = connectedNode.getCurrentTime().totalNsecs()/1000000;

		//Initialize temporal solver
		ActivityNetworkSolver temporalSolver = new  ActivityNetworkSolver(timeNow, 100000000);
		
		//creating variables
		Variable var1 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_1");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_base");
		
		Variable var2 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_base");
		
		
		//adding temporal constraints in order to make robots not start together
		//TODO: remove all temporal constraints
		//TODO: tell students to make robots not start together
		AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(timeNow + 5000,APSPSolver.INF));
		release.setFrom(var1);
		release.setTo(var1);
		temporalSolver.addConstraint(release);
		
		//TODO: remove all temporal constraints
		//TODO: tell students to make robots not start together
		AllenIntervalConstraint overlaps = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Overlaps, new Bounds(5000,APSPSolver.INF), new Bounds(1,APSPSolver.INF), new Bounds(1,APSPSolver.INF));
		overlaps.setTo(var2);
		overlaps.setFrom(var1);
		temporalSolver.addConstraint(overlaps);
		
		InferenceCallback cb = new InferenceCallback() {
			@Override
			public void doInference(long timeNow) {
			}
		};
		
		//creating animator to model the passing of time in the constraint network
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(temporalSolver, 1000, cb);
		
		/*
		creating ROSDispatchingFunction for turtlebot_1 : triggers a ROS service call when a dispatchable variable appears in the constraint network
		triggers a ROS service call when a dispatchable variable
		appears in the constraint network
		 */
		ROSDispatchingFunction robotDispatchingFunction1 = new ROSDispatchingFunction("turtlebot_1", temporalSolver, this.connectedNode) {
			
			@Override
			public boolean skip(SymbolicVariableActivity act) { return false; }
			
			//TODO: remove sendGoal call
			//TODO: tell them to pick locations from rviz
			//TODO: add comment saying that they have private sendGoal
			@Override
			public void dispatch(SymbolicVariableActivity act) {
				currentAct = act;
				//there is a local implementation of robot's move_base ROS service call
				//look at the local method "sendGoal" below
				sendGoal(robot, 3.0f, -1.0f, 0.3f); 
			}
		};
		
		/*
		creating ROSDispatchingFunction for turtlebot_2 : triggers a ROS service call when a dispatchable variable appears in the constraint network
		triggers a ROS service call when a dispatchable variable
		appears in the constraint network
		 */
		ROSDispatchingFunction robotDispatchingFunction2 = new ROSDispatchingFunction("turtlebot_2", temporalSolver, this.connectedNode) {
			@Override
			public boolean skip(SymbolicVariableActivity act) { return false;}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {
				currentAct = act;
				//there is a local implementation of robot's move_base ROS service call
				//look at the local method "sendGoal" below
				sendGoal(robot, -2.0f, -2.0f, 0.0f); 
			}		
		};
		//adding dispatchers to the animator
		animator.addDispatchingFunctions(temporalSolver, robotDispatchingFunction1 ,robotDispatchingFunction2);
		
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(temporalSolver.getConstraintNetwork(),"Activity network");
		//initializing  timeline visualizer and introducing variable annotation to that 
		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)temporalSolver, new Bounds(0,120000), true, "turtlebot_1", "turtlebot_2");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
		
	}
	/*
	Implements the sendGoal services for each robot (e.g., "/turtlebot_1/sendGoal")
	service request is x, y and theta representing the goal pose and orientation (in radian)
	 */
	private void sendGoal(String robot, float x, float y, float theta) {
		ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
		try { serviceClient = connectedNode.newServiceClient("/"+robot+"/sendGoal", sendGoal._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final sendGoalRequest request = serviceClient.newMessage();
		request.setX(x);
		request.setY(y);
		request.setTheta(theta);
		request.setRotationAfter((byte)0);
		serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {
			@Override
			public void onSuccess(sendGoalResponse arg0) {System.out.println("Goal sent");}
			@Override
			public void onFailure(RemoteException arg0) { }
		});		
	}

}
