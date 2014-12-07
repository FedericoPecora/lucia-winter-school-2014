package se.oru.aass.lucia_meta_csp_lecture.solutions;

import java.util.Vector;

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
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
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
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

public class Part3  extends AbstractNodeMain {


	private ConnectedNode connectedNode;
	private final String nodeName = "Part3";
	
	private SpatioTemporalSetNetworkSolver spatioTemporalSetSolver;
	private ActivityNetworkSolver temporalSolver;
	private GeometricConstraintSolver spatialSolver;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	@Override
	public void onStart(ConnectedNode cn) {
		
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
		spatioTemporalSetSolver = new SpatioTemporalSetNetworkSolver(origin,origin+1000000,500,new String[]{}); 
		temporalSolver = spatioTemporalSetSolver.getActivitySolver();
		spatialSolver = spatioTemporalSetSolver.getGeometricSolver();
		
		//create panel polygon
		Vec2 p1 = new Vec2(-0.129f, 1.284f);
		Vec2 p2 = new Vec2(-0.135f, 0.916f);
		Variable[] panelVars = PanelFactory.createPolygonVariables("panel1", p1, p2, spatialSolver);

		Vec2 robot1_center = new Vec2(0.0f, 0.0f);
		SpatioTemporalSet turtlebot_1 = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable("turtlebot_1", robot1_center, 0.0f, spatioTemporalSetSolver);
		turtlebot_1.getActivity().setSymbolicDomain("move_base");
		
		Vec2 robot2_center = new Vec2(-2.0f, -1.0f);
		SpatioTemporalSet turtlebot_2 = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable("turtlebot_2", robot2_center, 0.0f, spatioTemporalSetSolver);
		turtlebot_2.getActivity().setSymbolicDomain("move_base");

		//TODO: students do this
		//add spatial constraint
		//robots should be inside each polygon on the sides of panels
		GeometricConstraint inside1 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside1.setFrom(turtlebot_1.getPolygon());
		inside1.setTo((Polygon)panelVars[1]);
		spatioTemporalSetSolver.getGeometricSolver().addConstraint(inside1);
		
		GeometricConstraint inside2 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside2.setFrom((Polygon)turtlebot_2.getPolygon());
		inside2.setTo((Polygon)panelVars[0]);
		spatioTemporalSetSolver.getGeometricSolver().addConstraint(inside2);
		
		InferenceCallback cb = new InferenceCallback() {
			@Override
			public void doInference(long timeNow) { }
		};
		
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(temporalSolver, 1000, cb);
		
		ROSDispatchingFunction robotDispatchingFunction1 = new ROSDispatchingFunction("turtlebot_1", temporalSolver, this.connectedNode) {
			
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {	
				Polygon currentPoly = spatioTemporalSetSolver.getPolygonByActivity(act);
				//TODO: students do this
				sendGoal(robot, currentPoly.getPosition().x, currentPoly.getPosition().y, currentPoly.getOrientation());
			}
		};
		
		ROSDispatchingFunction robotDispatchingFunction2 = new ROSDispatchingFunction("turtlebot_2", temporalSolver, this.connectedNode) {
			@Override
			public boolean skip(SymbolicVariableActivity act) {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public void dispatch(SymbolicVariableActivity act) {
				Polygon currentPoly = spatioTemporalSetSolver.getPolygonByActivity(act);
				//TODO: students do this
				sendGoal(robot, currentPoly.getPosition().x, currentPoly.getPosition().y, currentPoly.getOrientation());
			}
		};
		animator.addDispatchingFunctions(temporalSolver, robotDispatchingFunction1, robotDispatchingFunction2);
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(temporalSolver.getConstraintNetwork(),"Activity network");
		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)temporalSolver, new Bounds(0,120000), true, "turtlebot_1", "turtlebot_2");
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
