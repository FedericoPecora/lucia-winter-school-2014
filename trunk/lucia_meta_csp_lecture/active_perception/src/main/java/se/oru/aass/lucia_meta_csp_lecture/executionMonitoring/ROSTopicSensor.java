package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import lucia_sim_2014.getLocation;
import lucia_sim_2014.getLocationRequest;
import lucia_sim_2014.getLocationResponse;
import lucia_sim_2014.getPanel;
import lucia_sim_2014.getPanelRequest;
import lucia_sim_2014.getPanelResponse;
import lucia_sim_2014.getQR;
import lucia_sim_2014.getQRRequest;
import lucia_sim_2014.getQRResponse;

import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.Activity;
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

	private SpatioTemporalSetNetworkSolver solver;
	private float[] pose = null;
	private ROSDispatchingFunction proprioception;
	private ConnectedNode rosNode = null;
	private ServiceClient<getLocationRequest, getLocationResponse> serviceClientLocation = null;
	private ServiceClient<getQRRequest, getQRResponse> serviceClientQR = null;
	private boolean doneOnce = false;
	private int seenQR = -2;
	private String robot;
	private long stopTime = -1;
	
	public ROSTopicSensor(String rob, ConstraintNetworkAnimator animator, SpatioTemporalSetNetworkSolver solver, ROSDispatchingFunction proprioception, final ConnectedNode rosNode) {
		super(rob, animator);
		this.solver = solver;
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
	
	protected Activity createNewActivity(String value) {
		SpatioTemporalSet act = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable(this.name, new Vec2(0.0f,0.0f), 0.0f, solver);
		act.setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
		((SpatioTemporalSet)act).setTask("Observe");
		
		getObservedPanel();
		
		SymbolicValueConstraint seesNothing = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
		
		while (seenQR == -2) { try { Thread.sleep(10); } catch (InterruptedException e) { e.printStackTrace(); } }
		String panel = "None";
		if (seenQR >= 0) panel = "P"+seenQR; 
		seesNothing.setValue(panel);
		seesNothing.setFrom(act);
		seesNothing.setTo(act);
		solver.addConstraint(seesNothing);

		System.out.println("%%%%%%%%%%%%%%%%%%%%%% MODELING " + act + " (value: " + value + ")");
		return act;
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
		if (proprioception.isExecuting() && positionChanged(newPose) && !doneOnce) {
			doneOnce = true;
			pose = newPose;
			return true;
		}
		if (!proprioception.isExecuting() && doneOnce) {
			doneOnce = false;
			if (positionChanged(newPose)) {
				pose = newPose;
				return true;
			}
			return false;
		}
		return false;
		
//		
//		//Robot moving, this is the first change
//		if (proprioception.isExecuting() && positionChanged(newPose) && !doneOnce) {
//			System.out.println("DEBUG (" + robot + "): FIRST CHANGE --> " + value);
//			pose = newPose;
//			doneOnce = true;
//			return true;
//		}
//		//If not changed, it is finished (need to finish execution)
//		if (proprioception.isExecuting() && !positionChanged(newPose)) {
//			System.out.println("DEBUG (" + robot + "): MAYBE STILL --> " + value);
//			
//			
//			
//			
//			if (stopTime == -1) {
//				stopTime = rosNode.getCurrentTime().totalNsecs()/1000000;
//				return false;
//			}
//			long currentTime = rosNode.getCurrentTime().totalNsecs()/1000000;
//			if (currentTime-stopTime > 12000) {
//				stopTime = -1;
//				//this will set isExecuting to false, so we will not get back into here
//				proprioception.finishCurrentActivity();
//				doneOnce = false;
//				return true;
//			}
//		}
		
//		//First time we see anything
//		if (!proprioception.isExecuting() && pose == null) {
//			System.out.println("DEBUG (" + robot + "): FIRST TIME --> " + value);
//			pose = newPose;
//			return true;
//		}
//		//We are in a new place, must unify this obs with predicted obs
//		else if (!proprioception.isExecuting() && positionChanged(newPose)) {
//			System.out.println("DEBUG (" + robot + "): IN A NEW PLACE --> " + value);
//			pose = newPose;
//			movingObserveStarted = false;
//			return true;
//		}
//		//We are moving, so we need to interrupt previous reading (but do this only once)
//		else if (proprioception.isExecuting() && positionChanged(newPose) && !movingObserveStarted) {
//			System.out.println("DEBUG (" + robot + "): INTERMDIATE --> " + value);
//			pose = newPose;
//			movingObserveStarted = true;
//			return true;
//		}
//		else {
//			System.out.println("DEBUG (" + robot + "): IGNORED SENSOR READING --> " + value);
//		}
	}
	
	private boolean positionChanged(float[] newPose) {
		float epsilon = 0.01f; //1cm threshold for change
		for (int i = 0; i < pose.length; i++)
			if (Math.abs(pose[i]-newPose[i]) < epsilon) return false;
		return true;
	}


}
