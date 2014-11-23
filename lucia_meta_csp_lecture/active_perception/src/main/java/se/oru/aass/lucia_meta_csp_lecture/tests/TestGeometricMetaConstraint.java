package se.oru.aass.lucia_meta_csp_lecture.tests;

import java.util.Calendar;
import java.util.Vector;

import lucia_sim_2014.getPanel;
import lucia_sim_2014.getPanelRequest;
import lucia_sim_2014.getPanelResponse;

import org.apache.commons.logging.Log;
import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.PolygonFrame;
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
import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSTopicSensor;
import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSTopicListener;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.AssignmentMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.ObservabilityMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.SimpleMoveBasePlanner;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;
import se.oru.aass.lucia_meta_csp_lecture.*;


public class TestGeometricMetaConstraint extends AbstractNodeMain {

	private ConnectedNode connectedNode;
	
	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver spatioTemporalSetSolver;
	private ActivityNetworkSolver activitySolver;
	private GeometricConstraintSolver geometricSolver;
	private SymbolicVariableConstraintSolver setSolver;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("LuciaCSPNode");
	}

	private void getPanelsFromROSService(final String[] panelNames) {
		ServiceClient<getPanelRequest, getPanelResponse> serviceClient = null;
		try { serviceClient = connectedNode.newServiceClient("getPanel", getPanel._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final getPanelRequest request = serviceClient.newMessage();
		request.setRead((byte) 0);
		serviceClient.call(request, new ServiceResponseListener<getPanelResponse>() {
			
			@Override
			public void onSuccess(getPanelResponse arg0) {
				Variable[] panel1 = PanelFactory.createPolygonVariables(panelNames[0], new Vec2((float)arg0.getPanel1X1(),(float)arg0.getPanel1Y1()), new Vec2((float)arg0.getPanel1X2(),(float)arg0.getPanel1Y2()), geometricSolver);
				Variable[] panel2 = PanelFactory.createPolygonVariables(panelNames[1], new Vec2((float)arg0.getPanel2X1(),(float)arg0.getPanel2Y1()), new Vec2((float)arg0.getPanel2X2(),(float)arg0.getPanel2Y2()), geometricSolver);
				Variable[] panel3 = PanelFactory.createPolygonVariables(panelNames[2], new Vec2((float)arg0.getPanel3X1(),(float)arg0.getPanel3Y1()), new Vec2((float)arg0.getPanel3X2(),(float)arg0.getPanel3Y2()), geometricSolver);
				Variable[] panel4 = PanelFactory.createPolygonVariables(panelNames[3], new Vec2((float)arg0.getPanel4X1(),(float)arg0.getPanel4Y1()), new Vec2((float)arg0.getPanel4X2(),(float)arg0.getPanel4Y2()), geometricSolver);
			}
			
			@Override
			public void onFailure(RemoteException arg0) {
				throw new RosRuntimeException(arg0);				
			}
		});
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");

		//TODO: get panels from ROS topic
		int numPanels = 4;
		String[] panelNames = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panelNames[i] = "P"+i;
			symbols[i] = "P"+i;
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";

		long origin = connectedNode.getCurrentTime().totalNsecs()/1000;
		metaSolver = new LuciaMetaConstraintSolver(origin,origin+1000000,500,symbols);
		spatioTemporalSetSolver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		activitySolver = spatioTemporalSetSolver.getActivitySolver();
		geometricSolver = spatioTemporalSetSolver.getGeometricSolver();
		setSolver = spatioTemporalSetSolver.getSetSolver();
		
		getPanelsFromROSService(panelNames);
		
		InferenceCallback cb = new InferenceCallback() {
			@Override
			public void doInference(long timeNow) {
				metaSolver.clearResolvers();
				metaSolver.backtrack();
				Vector<SymbolicVariableActivity> moveBaseActivities = new Vector<SymbolicVariableActivity>();
				ConstraintNetwork[] cns = metaSolver.getAddedResolvers();
				if (cns != null && cns.length > 0) {
					for (ConstraintNetwork cn : cns) {
						if (cn.getAnnotation() instanceof SimpleMoveBasePlanner) {
							for (Variable var : cn.getVariables()) {
								SymbolicVariableActivity act = (SymbolicVariableActivity)var;
								if (act.getSymbolicVariable().getSymbols()[0].equals("move_base"))
									moveBaseActivities.add(act);
							}
						}
					}
					for (Variable act : moveBaseActivities) {
						AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(timeNow,APSPSolver.INF));
						release.setFrom(act);
						release.setTo(act);
						activitySolver.addConstraint(release);
					}
				}
			}
		};
		
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(activitySolver, 1000, cb);

		//Vars representing robots and what panels (if any) they see
		int numRobots = 5;
		String[] robotTimelines = new String[numRobots];
		for (int i = 0; i < numRobots; i++) {
			robotTimelines[i] = "StateOfRobot"+i;
			ROSDispatchingFunction df = new ROSDispatchingFunction(robotTimelines[i], spatioTemporalSetSolver);
			animator.addDispatchingFunctions(activitySolver, df);
			ROSTopicSensor sensor = new ROSTopicSensor(robotTimelines[i], "/a/topic", animator, spatioTemporalSetSolver, df);
			sensor.registerSensorTrace("sensorTraces/robot" + i + ".st", origin);
		}

		AssignmentMetaConstraint mc1 = new AssignmentMetaConstraint(null, null);
		mc1.setPanels(panelNames);
		metaSolver.addMetaConstraint(mc1);

		SimpleMoveBasePlanner mc2 = new SimpleMoveBasePlanner(null, null);
		metaSolver.addMetaConstraint(mc2);

		ObservabilityMetaConstraint mc3 = new ObservabilityMetaConstraint(null, null);
		metaSolver.addMetaConstraint(mc3);

		ConstraintNetwork.draw(spatioTemporalSetSolver.getConstraintNetwork(), "SpatioTemporalSet network");
		ConstraintNetwork.draw(activitySolver.getConstraintNetwork(),"Activity network");
		ConstraintNetwork.draw(geometricSolver.getConstraintNetwork(),"Polygon network");
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", geometricSolver.getConstraintNetwork()/*,850.0f*/);

		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)activitySolver, new Bounds(0,120000), true, robotTimelines);
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
	}

}
