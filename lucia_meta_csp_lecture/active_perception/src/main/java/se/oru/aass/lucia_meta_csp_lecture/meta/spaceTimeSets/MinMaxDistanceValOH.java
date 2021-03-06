package se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets;

import java.awt.geom.Point2D;
import java.util.Collections;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.spatial.geometry.Vec2;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSTopicSensor;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;

public class MinMaxDistanceValOH extends ValueOrderingH{
	
	
	private Vector<ROSTopicSensor> sensors;
	private ConstraintNetwork noGoodCN = null;

	@Override
	public int compare(ConstraintNetwork arg0, ConstraintNetwork arg1) {
		Constraint[] cons0 = arg0.getConstraints();
		Constraint[] cons1 = arg1.getConstraints();
		
		HashMap<String, String> robotToPanelarg0 = new HashMap<String, String>();
		HashMap<String, String> robotToPanelarg1 = new HashMap<String, String>();
		HashMap<String, String> robotToPanelnoGoodCN = new HashMap<String, String>();

		double maxDis0 = getMinDist(cons0, robotToPanelarg0);
		double maxDis1 = getMinDist(cons1, robotToPanelarg1);
		
		if(noGoodCN != null){
			getMinDist(noGoodCN.getConstraints(), robotToPanelnoGoodCN);
			if(robotToPanelnoGoodCN.equals(robotToPanelarg0)) return 1;
			if(robotToPanelnoGoodCN.equals(robotToPanelarg1)) return -1;
		}
		
		if(maxDis0 < maxDis1) return -1;
		if(maxDis0 > maxDis1) return 1;
		return 0;
	}
	
	private double getMinDist(Constraint[] cons0, HashMap<String, String> robotToPanel){
		Vector<Double> maxDis0 = new Vector<Double>();

		for (int i = 0; i < cons0.length; i++) {
			if(cons0[i] instanceof SymbolicValueConstraint){
				if(((SymbolicValueConstraint)cons0[i]).getType().equals(SymbolicValueConstraint.Type.VALUEEQUALS)){					
					String PanelId = ((SymbolicValueConstraint)cons0[i]).getValue()[0];
					Variable var = ((SymbolicValueConstraint)cons0[i]).getScope()[0];
					String robotName = ((SpatioTemporalSet)var).getActivity().getComponent();
					Vec2 panelPose = PanelFactory.getPanelCenterById(PanelId);
					Vec2 robotCurentPose = null;
					for (int j = 0; j < sensors.size(); j++) {
						if(sensors.get(j).getRobotName().equals(robotName)){
							robotCurentPose = sensors.get(j).getRobotCurrentPose();  
						}							
					}
					robotToPanel.put(robotName, PanelId);
					maxDis0.add(Point2D.Float.distance(panelPose.x, panelPose.y, robotCurentPose.x, robotCurentPose.y));
//					System.out.println(robotName + " "+ PanelId + " "+ Point2D.Float.distance(panelPose.x, panelPose.y, robotCurentPose.x, robotCurentPose.y));
				}
			}				
		}
		return Collections.max(maxDis0);
	}
	
	public void setSensors(Vector<ROSTopicSensor> sensors) {
		this.sensors = sensors;
	}
	
	public void setNoGoodSolution(ConstraintNetwork cn){
		this.noGoodCN = cn;
	}


}
