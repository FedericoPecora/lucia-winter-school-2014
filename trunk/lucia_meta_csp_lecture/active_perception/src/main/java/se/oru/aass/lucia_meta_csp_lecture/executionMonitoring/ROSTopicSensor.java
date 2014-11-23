package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.Sensor;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia2014.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia2014.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia2014.util.RobotFactory;

public class ROSTopicSensor extends Sensor {

	private static final long serialVersionUID = -8096347089406131305L;

	private String topicOrServiceName;
	private SpatioTemporalSetNetworkSolver solver;
	private float[] pose = null;
	private ROSDispatchingFunction proprioception;
	
	public ROSTopicSensor(String name, String topicOrServiceName, ConstraintNetworkAnimator animator, SpatioTemporalSetNetworkSolver solver, ROSDispatchingFunction proprioception) {
		super(name, animator);
		this.topicOrServiceName = topicOrServiceName;
		this.solver = solver;
		this.proprioception = proprioception;
		// TODO Subscribe to a topic, and model sensor values coming from it
		// TODO Call a service, and model sensor values received as response
	}
	
	protected Activity createNewActivity(String value) {
		SpatioTemporalSet act = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable(this.name, new Vec2(0.0f,0.0f), 0.0f, solver);
		act.setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
		((SpatioTemporalSet)act).setTask("Observe");
		
		SymbolicValueConstraint seesNothing = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
		seesNothing.setValue("None");
		seesNothing.setFrom(act);
		seesNothing.setTo(act);
		solver.addConstraint(seesNothing);

		return act;
	}
	
	protected boolean hasChanged(String value) {
		//value represents position of robot
		float[] newPose = new float[3];
		String[] poseS = value.split(",");
		for (int i = 0; i < poseS.length; i++) newPose[i] = Float.parseFloat(poseS[i]);
		if (!proprioception.isExecuting() && (pose == null || positionChanged(newPose))) {
			pose = newPose;
			return true;
		}
		return false;
	}
	
	private boolean positionChanged(float[] newPose) {
		for (int i = 0; i < pose.length; i++)
			if (pose[i] != newPose[i]) return true;
		return false;
	}

}
