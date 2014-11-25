package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import java.util.List;

import lucia_sim_2014.getLocation;
import lucia_sim_2014.getLocationRequest;
import lucia_sim_2014.getLocationResponse;
import lucia_sim_2014.getStatus;
import lucia_sim_2014.getStatusRequest;
import lucia_sim_2014.getStatusResponse;
import lucia_sim_2014.sendGoal;
import lucia_sim_2014.sendGoalRequest;
import lucia_sim_2014.sendGoalResponse;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;

public class ROSDispatchingFunction extends DispatchingFunction {

	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver solver;
	private SymbolicVariableActivity currentAct = null;
	private boolean isExecuting = false;
	private ConnectedNode rosNode = null;
	private String robot = null;
	private ServiceClient<getStatusRequest, getStatusResponse> serviceClientStatus = null;

	public ROSDispatchingFunction(String rob, LuciaMetaConstraintSolver metaSolver, ConnectedNode rosN) {
		super(rob);
		this.metaSolver = metaSolver;
		this.solver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		this.rosNode = rosN;
		this.robot = rob;
		
		// Call position service, and model sensor values received as response
		try { serviceClientStatus = rosNode.newServiceClient(robot+"/getStatus", getStatus._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }

		Thread statusMonitor = new Thread() {
			boolean hasStartedMoving = false;
			public void run() {
				while (true) {
					try { Thread.sleep(1000); }
					catch (InterruptedException e) { e.printStackTrace(); }
					if (currentAct != null) {
						final getStatusRequest request = serviceClientStatus.newMessage();
						request.setRead((byte) 0);
						serviceClientStatus.call(request, new ServiceResponseListener<getStatusResponse>() {
							@Override
							public void onFailure(RemoteException arg0) { }
	
							@Override
							public void onSuccess(getStatusResponse arg0) {
								//Only believe that it has stopped if we previously saw it moving
								if ((int)arg0.getStatus() < 0) {
									if (hasStartedMoving) {
										finishCurrentActivity();
										hasStartedMoving = false;
									}
								}
								//We saw the robot move, now we can start checking whether it has stopped
								else { hasStartedMoving = true; }
							}
						});
					}
				}
			}
		};
		statusMonitor.start();
	}

	private void sendGoal(final String robot, final String command, final Vec2 destination) {
		System.out.println(">>>> TO ROS: " + command + " " + destination);
		ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
		try { serviceClient = rosNode.newServiceClient(robot+"/sendGoal", sendGoal._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final sendGoalRequest request = serviceClient.newMessage();
		request.setX(destination.x);
		request.setY(destination.y);
		request.setTheta(0);
		serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {

			@Override
			public void onSuccess(sendGoalResponse arg0) { }

			@Override
			public void onFailure(RemoteException arg0) { }
		});

	}

	public void finishCurrentActivity() {
		System.out.println(">>>>>>>>>>>>>>>>>>> (" + robot + ") FINISHED!!!");
		this.finish(currentAct);
		currentAct = null;
		setExecuting(false);
	}

	public boolean isExecuting() {
		return isExecuting;
	}

	public void setExecuting(boolean exec) {
		isExecuting = exec;
	}

	@Override
	public void dispatch(SymbolicVariableActivity act) {
		System.out.println("????????????????????? DISPATCHING " + act.getSymbolicVariable().getSymbols()[0]);
		currentAct = act;
		String robot = act.getComponent();
		String command = act.getSymbolicVariable().getSymbols()[0];

		//Get the two constraint networks (one is low level, one is hi level)
		ConstraintNetwork activityNetwork = this.getConstraintNetwork();
		ConstraintNetwork spatioTemporalSetNetwork = solver.getConstraintNetwork();

		//Find observe activity to which this move_base is leading
		Constraint[] cons = activityNetwork.getOutgoingEdges(act);
		SymbolicVariableActivity observeAct = null;
		for (Constraint con : cons) {
			if (((SymbolicVariableActivity)con.getScope()[1]).getSymbolicVariable().getSymbols()[0].equals("Observe")) {
				observeAct = (SymbolicVariableActivity)con.getScope()[1];
				break;
			}
		}

		//Find polygon from which to compute destination
		Polygon destPoly = null;
		for (Variable var : spatioTemporalSetNetwork.getVariables()) {
			SpatioTemporalSet sts = (SpatioTemporalSet)var;
			if (sts.getActivity().equals(observeAct)) {
				destPoly = sts.getPolygon();
				break;
			}
		}

		Vec2 destination = destPoly.getPosition();

		isExecuting = true;

		this.sendGoal(robot, command, destination);

	}

	@Override
	public boolean skip(SymbolicVariableActivity act) {
		//Do not dispatch "Observe" activities
		if (act.getSymbolicVariable().getSymbols()[0].equals("Observe")) return true;
		return false;
	}

}
