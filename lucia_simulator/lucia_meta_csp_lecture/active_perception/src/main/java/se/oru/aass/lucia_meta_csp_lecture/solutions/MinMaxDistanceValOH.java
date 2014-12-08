package se.oru.aass.lucia_meta_csp_lecture.solutions;

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
		//TODO: explain contents of arg0 and arg1, example in comment
		
		//arg0 and arg1 are constraint networks representing two 
		Constraint[] cons0 = arg0.getConstraints();
		Constraint[] cons1 = arg1.getConstraints();
		
		//E.g., Robot1 -> P3, Robot2 -> P1


		double maxDis0 = getMaxDist(cons0);
		double maxDis1 = getMaxDist(cons1);
		
		//TODO: separate the two functionalities of getMaxDist
		if(noGoodCN != null){
			HashMap<String, String> robotToPanelarg0 = getAssignments(cons0);
			HashMap<String, String> robotToPanelarg1 = getAssignments(cons1);
			HashMap<String, String> robotToPanelnoGoodCN = getAssignments(noGoodCN.getConstraints());
			if(robotToPanelnoGoodCN.equals(robotToPanelarg0)) return 1;
			if(robotToPanelnoGoodCN.equals(robotToPanelarg1)) return -1;
		}
		
		if(maxDis0 < maxDis1) return -1;
		if(maxDis0 > maxDis1) return 1;
		return 0;
	}
	
	private double getMaxDist(Constraint[] cons){
		Vector<Double> maxDis0 = new Vector<Double>();

		for (int i = 0; i < cons.length; i++) {
			if(cons[i] instanceof SymbolicValueConstraint){
				if(((SymbolicValueConstraint)cons[i]).getType().equals(SymbolicValueConstraint.Type.VALUEEQUALS)){					
					String PanelId = ((SymbolicValueConstraint)cons[i]).getValue()[0];
					Variable var = ((SymbolicValueConstraint)cons[i]).getScope()[0];
					String robotName = ((SpatioTemporalSet)var).getActivity().getComponent();
					Vec2 panelPose = PanelFactory.getPanelCenterById(PanelId);
					Vec2 robotCurentPose = null;
					for (int j = 0; j < sensors.size(); j++) {
						if(sensors.get(j).getRobotName().equals(robotName)){
							robotCurentPose = sensors.get(j).getRobotCurrentPose();  
						}							
					}
					maxDis0.add(Point2D.Float.distance(panelPose.x, panelPose.y, robotCurentPose.x, robotCurentPose.y));
//					System.out.println(robotName + " "+ PanelId + " "+ Point2D.Float.distance(panelPose.x, panelPose.y, robotCurentPose.x, robotCurentPose.y));
				}
			}				
		}
		return Collections.max(maxDis0);
	}
	
	
	private HashMap<String, String> getAssignments(Constraint[] cons){
		HashMap<String, String> robotToPanel = new HashMap<String, String>();
		for (int i = 0; i < cons.length; i++) {
			if(cons[i] instanceof SymbolicValueConstraint){
				if(((SymbolicValueConstraint)cons[i]).getType().equals(SymbolicValueConstraint.Type.VALUEEQUALS)){					
					String PanelId = ((SymbolicValueConstraint)cons[i]).getValue()[0];
					Variable var = ((SymbolicValueConstraint)cons[i]).getScope()[0];
					String robotName = ((SpatioTemporalSet)var).getActivity().getComponent();
					robotToPanel.put(robotName, PanelId);
				}
			}				
		}
		return robotToPanel;
	}
	
	public void setSensors(Vector<ROSTopicSensor> sensors) {
		this.sensors = sensors;
	}
	
	public void setNoGoodSolution(ConstraintNetwork cn){
		this.noGoodCN = cn;
	}


}
