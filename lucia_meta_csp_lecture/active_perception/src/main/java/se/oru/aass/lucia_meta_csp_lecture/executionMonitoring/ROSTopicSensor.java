package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import java.util.Arrays;
import java.util.Vector;

import lucia_sim_2014.getLocation;
import lucia_sim_2014.getLocationRequest;
import lucia_sim_2014.getLocationResponse;
import lucia_sim_2014.getPanel;
import lucia_sim_2014.getPanelRequest;
import lucia_sim_2014.getPanelResponse;
import lucia_sim_2014.getQR;
import lucia_sim_2014.getQRRequest;
import lucia_sim_2014.getQRResponse;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.Sensor;
import org.metacsp.spatial.geometry.Vec2;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

public class ROSTopicSensor extends Sensor {

	private static final long serialVersionUID = -8096347089406131305L;

	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver solver;
	private float[] pose = null;
	private ROSDispatchingFunction proprioception;
	private ConnectedNode rosNode = null;
	private ServiceClient<getLocationRequest, getLocationResponse> serviceClientLocation = null;
	private ServiceClient<getQRRequest, getQRResponse> serviceClientQR = null;
	private boolean doneOnce = false;
	private int seenQR = -2;
	private String robot;
	
	public ROSTopicSensor(String rob, ConstraintNetworkAnimator animator, LuciaMetaConstraintSolver metaSolver, ROSDispatchingFunction proprioception, final ConnectedNode rosNode) {
		super(rob, animator);
		this.metaSolver = metaSolver;
		this.solver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		this.proprioception = proprioception;
		this.rosNode = rosNode;
		this.robot = rob;

		// Call position service, and model sensor values received as response
		try { serviceClientLocation = rosNode.newServiceClient(robot+"/getLocation", getLocation._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		
		Thread checkPositionThread = new Thread() {
			
			public void run() {
				while (true) {
					try { Thread.sleep(1000); }
					catch (InterruptedException e) { e.printStackTrace(); }
					final getLocationRequest request = serviceClientLocation.newMessage();
					request.setRead((byte) 0);
					serviceClientLocation.call(request, new ServiceResponseListener<getLocationResponse>() {
						@Override
						public void onSuccess(getLocationResponse arg0) {
							String sensorValue = arg0.getX() + "," + arg0.getY() + "," + arg0.getTheta();
							long timeNow = rosNode.getCurrentTime().totalNsecs()/1000000;
							synchronized (ans) {
								postSensorValue(sensorValue, timeNow);	
							}
						}
						
						@Override
						public void onFailure(RemoteException arg0) {
							throw new RosRuntimeException(arg0);				
						}
					});
				}
			}
		};
		checkPositionThread.start();
	}
	
	private void getObservedPanel() {
		seenQR = -2;
		try { serviceClientQR = rosNode.newServiceClient(robot+"/getQR", getQR._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		getQRRequest request = serviceClientQR.newMessage();
		
		request.setRead((byte) 0);
		serviceClientQR.call(request, new ServiceResponseListener<getQRResponse>() {
			@Override
			public void onSuccess(getQRResponse arg0) {
				seenQR = (int)arg0.getQrcode();
			}
			
			@Override
			public void onFailure(RemoteException arg0) {
				throw new RosRuntimeException(arg0);				
			}
		});
	}
	
	private SpatioTemporalSet createPanelObservation(String panel) {
		SpatioTemporalSet act = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable(this.name, new Vec2(0.0f,0.0f), 0.0f, solver);
		act.setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
		((SpatioTemporalSet)act).setTask("Observe");
		
		SymbolicValueConstraint observedPanelConstraint = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
		observedPanelConstraint.setValue(panel);
		observedPanelConstraint.setFrom(act);
		observedPanelConstraint.setTo(act);
		solver.addConstraint(observedPanelConstraint);
		
		System.out.println("%%%%%%%%%%%%%%%%%%%%%% MODELING (" + robot + ") " + act);
		
		return act;
	}
	
	protected Activity createNewActivity(String value) {	
		//Here we need to check whether we can unify with expectation or not
		// -- If we can unify, return current expectation as the new sensor value
		// -- If we cannot, return a new SpatioTemporalSet with value "None"
		
		//Get currently observed panel
		getObservedPanel();
		while (seenQR == -2) { try { Thread.sleep(10); } catch (InterruptedException e) { e.printStackTrace(); } }
				
		//See if there is an expectation
		Variable[] vars = this.metaSolver.getFocused();
		SpatioTemporalSet expectation = null;

		if (vars != null) {
			for (Variable var : vars) {
				if (var.getComponent().equals(this.robot)) {
					expectation = (SpatioTemporalSet)var;
					break;
				}
			}
		}
		
		//If we have no expectation, model a new sensor reading (initial condition)
		if (expectation == null) {
			if (seenQR < 0) return createPanelObservation("None");
			else return createPanelObservation("P"+seenQR);
		}
		
		//If we are executing...
		if (proprioception.isExecuting()) {
			if (seenQR < 0) return createPanelObservation("None");
			else return createPanelObservation("P"+seenQR);
		}

		//If we are here, we have finished executing, thus
		//expectation is certainly != null
//		String expectedPanel = expectation.getSet().getSymbols()[0];
		String newPanel = "P"+seenQR;
		if (seenQR < 0) newPanel = "None";
		SpatioTemporalSet ret = createPanelObservation(newPanel);
		
		//Update focus:
		// -- Remove expectation from focus
		this.metaSolver.removeFromCurrentFocus(expectation);
		// -- Add current observation to focus
		this.metaSolver.focus(ret);
		return ret;
	}

	protected boolean hasChanged(String value) {
		//value represents position of robot
		float[] newPose = new float[3];
		String[] poseS = value.split(",");
		for (int i = 0; i < poseS.length; i++) newPose[i] = Float.parseFloat(poseS[i]);
		
		//If pose has never been set... 
		if (pose == null) {
			System.out.println("DEBUG (" + robot + "): POSE HAS NEVER BEEN SET --> " + value);
			pose = newPose;
			return false;
		}
		//If this is the first sensing since I started moving
		if (proprioception.isExecuting() && positionChanged(newPose) && !doneOnce) {
			System.out.println("DEBUG (" + robot + "): FIRST LOCATION DURING MOVE --> " + value);
			doneOnce = true;
			pose = newPose;
			return true;
		}
		//If this is the first sensing since I stopped moving
		if (!proprioception.isExecuting() && doneOnce) {
			System.out.println("DEBUG (" + robot + "): FIRST LOCATION AFTER MOVE --> " + value);
			doneOnce = false;
			if (positionChanged(newPose)) {
				pose = newPose;
				return true;
			}
			System.out.println("DEBUG (" + robot + "): BUT POS NOT CHANGED! --> " + value);
			return false;
		}
		return false;		
	}
	
	private boolean positionChanged(float[] newPose) {
		float epsilon = 0.01f; //1cm threshold for change
		for (int i = 0; i < pose.length; i++)
			if (Math.abs(pose[i]-newPose[i]) < epsilon) return false;
		return true;
	}


}
