package se.oru.aass.lucia_meta_csp_lecture.solutions;

import java.util.Vector;
import java.util.logging.Level;

import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;

import org.apache.commons.logging.Log;
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
import org.metacsp.utility.logging.MetaCSPLogging;
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
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.SchedulingMetaConstraint;

public class Part1d  extends AbstractNodeMain {


	private ConnectedNode connectedNode;
	private final String nodeName = "Part1d";
	private Part1dMetaConstraintSolver metaSolver;
	
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	@Override
	public void onStart(ConnectedNode cn) {
		
		MetaCSPLogging.setLevel(Part1dMetaConstraintSolver.class, Level.FINEST);

		this.connectedNode = cn;
		
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");
		
		
		long origin = connectedNode.getCurrentTime().totalNsecs()/1000000;
		metaSolver = new Part1dMetaConstraintSolver(origin,origin+1000000,500);
		ActivityNetworkSolver ans = (ActivityNetworkSolver)metaSolver.getConstraintSolvers()[0];
		
		Variable var1 = (SymbolicVariableActivity)ans.createVariable("turtlebot_1");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_base");
		
		Variable var2 = (SymbolicVariableActivity)ans.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_base");
		
		//adding MetaConstraint
		SchedulingMetaConstraint schedlingMetaConstraint = new SchedulingMetaConstraint(null, null);
		metaSolver.addMetaConstraint(schedlingMetaConstraint);
		
		//add minimum duration
		AllenIntervalConstraint minDuration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(1000, APSPSolver.INF));
		minDuration1.setFrom(var1);
		minDuration1.setTo(var1);
		ans.addConstraint(minDuration1);

		AllenIntervalConstraint minDuration2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(1000, APSPSolver.INF));
		minDuration2.setFrom(var2);
		minDuration2.setTo(var2);
		ans.addConstraint(minDuration2);
	
		
		InferenceCallback cb = new InferenceCallback() {
			@Override
			public void doInference(long timeNow) {
				metaSolver.clearResolvers();
				metaSolver.backtrack();
			}
		};
		
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(ans, 1000, cb){
			@Override
			protected long getCurrentTimeInMillis() {
				return connectedNode.getCurrentTime().totalNsecs()/1000000;
			}
		};
		
		ROSDispatchingFunction robotDispatchingFunction1 = new ROSDispatchingFunction("turtlebot_1", ans, this.connectedNode) {		
			@Override
			public boolean skip(SymbolicVariableActivity act) { return false; }
			
			@Override
			public void dispatch(SymbolicVariableActivity act) { 
				currentAct = act;
				sendGoal(robot, 3.0f, -1.0f, 0.3f);
			}
		};
		
		ROSDispatchingFunction robotDispatchingFunction2 = new ROSDispatchingFunction("turtlebot_2", ans, this.connectedNode) {
			@Override
			public boolean skip(SymbolicVariableActivity act) { return false; }
			
			@Override
			public void dispatch(SymbolicVariableActivity act) { 
				currentAct = act;
				sendGoal(robot, -2.0f, -2.0f, 0.0f);
			}
		};

		animator.addDispatchingFunctions(ans, robotDispatchingFunction1, robotDispatchingFunction2);
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(ans.getConstraintNetwork(),"Activity network");
		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)ans, new Bounds(0,120000), true, "turtlebot_1", "turtlebot_2");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
		
	}
	
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
