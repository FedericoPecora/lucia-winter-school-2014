package se.oru.aass.lucia_meta_csp_lecture.tests;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
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
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.PolygonFrame;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Duration;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSDispatchingFunction;
import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSTopicSensor;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.AssignmentMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.ObservabilityMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.SimpleMoveBasePlanner;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelMarkerPublisher;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;
import se.oru.aass.lucia_meta_csp_lecture.*;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;


public class TestGeometricMetaConstraint extends AbstractNodeMain {

	private ConnectedNode connectedNode;
	private final String nodeName = "lucia_meta_csp_lecture";
	
	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver spatioTemporalSetSolver;
	private ActivityNetworkSolver activitySolver;
	private GeometricConstraintSolver geometricSolver;
	private SymbolicVariableConstraintSolver setSolver;
	private ParameterTree params;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
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
				
				//Build the panel polygons
				final ArrayList<Polygon> polygons = new ArrayList<Polygon>();
				final HashMap<Polygon,String> polygonsToPanelNamespaces = new HashMap<Polygon,String>();
				for(int i = 1; i <= panelNames.length; i++) {
					try {
						double x1 = (Double) arg0.getClass().getMethod("getPanel" + i + "X1", new Class[]{}).invoke(arg0, new Object[]{});
						double y1 = (Double) arg0.getClass().getMethod("getPanel" + i + "Y1", new Class[]{}).invoke(arg0, new Object[]{});
						double x2 = (Double) arg0.getClass().getMethod("getPanel" + i + "X2", new Class[]{}).invoke(arg0, new Object[]{});
						double y2 = (Double) arg0.getClass().getMethod("getPanel" + i + "Y2", new Class[]{}).invoke(arg0, new Object[]{});
						Variable[] polyVars = PanelFactory.createPolygonVariables(panelNames[i-1], new Vec2((float)x1,(float)y1), new Vec2((float)x2,(float)y2), geometricSolver);
						for (int t = 0; t < polyVars.length; t++) {
							//if (!(params.has("/" + nodeName + "/exclude_panel_" + (i) + "_side") && params.getInteger("/" + nodeName + "/exclude_panel_" + (i) + "_side") == (t+1))) {
								polygons.add((Polygon)polyVars[t]);
								polygonsToPanelNamespaces.put((Polygon)polyVars[t], "Panel " + panelNames[i-1] + " FoV " + (t+1));
							//}
						}
					}
					catch (Exception e) { e.printStackTrace(); }
				}
				new PanelMarkerPublisher(polygons, polygonsToPanelNamespaces, connectedNode);
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
		
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");

		params = connectedNode.getParameterTree();
				
		//Make symbol names (including panels)
		int numPanels = params.getInteger("/" + nodeName + "/num_used_panels");
		String[] panelNames = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panelNames[i] = "P"+(i);
			symbols[i] = "P"+(i);
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";

		long origin = connectedNode.getCurrentTime().totalNsecs()/1000000;
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
		
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(activitySolver, 1000, cb, true);

		//Vars representing robots and what panels (if any) they see
		int numRobots = params.getInteger("/" + nodeName + "/num_used_robots");
		String[] robotTimelines = new String[numRobots];
		for (int i = 0; i < numRobots; i++) {
			robotTimelines[i] = "turtlebot_"+(i+1);
			ROSDispatchingFunction df = new ROSDispatchingFunction(robotTimelines[i], metaSolver, connectedNode);
			animator.addDispatchingFunctions(activitySolver, df);
			ROSTopicSensor sensor = new ROSTopicSensor(robotTimelines[i], animator, metaSolver, df, connectedNode);
		}

		AssignmentMetaConstraint mc1 = new AssignmentMetaConstraint(null, null);
		mc1.setPanels(panelNames);
		metaSolver.addMetaConstraint(mc1);

		SimpleMoveBasePlanner mc2 = new SimpleMoveBasePlanner(null, null);
		metaSolver.addMetaConstraint(mc2);

		ObservabilityMetaConstraint mc3 = new ObservabilityMetaConstraint(null, null);
		metaSolver.addMetaConstraint(mc3);

//		ConstraintNetwork.draw(spatioTemporalSetSolver.getConstraintNetwork(), "SpatioTemporalSet network");
//		ConstraintNetwork.draw(activitySolver.getConstraintNetwork(),"Activity network");
//		ConstraintNetwork.draw(geometricSolver.getConstraintNetwork(),"Polygon network");
//		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", geometricSolver.getConstraintNetwork()/*,850.0f*/);

		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)activitySolver, new Bounds(0,120000), true, robotTimelines);
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
	}

}
