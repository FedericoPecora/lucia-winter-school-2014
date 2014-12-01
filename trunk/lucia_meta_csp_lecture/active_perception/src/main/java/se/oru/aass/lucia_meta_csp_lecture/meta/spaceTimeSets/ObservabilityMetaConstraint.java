package se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.utility.UI.PolygonFrame;

import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;

import com.sun.org.apache.bcel.internal.generic.INSTANCEOF;

public class ObservabilityMetaConstraint extends MetaConstraint {

	private static final long serialVersionUID = -2618200060652687235L;

	public ObservabilityMetaConstraint(VariableOrderingH varOH,	ValueOrderingH valOH) {
		super(varOH, valOH);
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {
		//Get the focused vars first
		Variable[] observeActivities = this.metaCS.getFocused();
		if (observeActivities == null) return null;
		Vector<Variable> predictedObserveActivities = new Vector<Variable>();

		//Filter vars that are in focus but not sensor readings (i.e., just expectations) 
		for (Variable var : observeActivities) {
			if (!((LuciaMetaConstraintSolver)this.metaCS).isSensorReading((SpatioTemporalSet)var)) {
				predictedObserveActivities.add(var);
			}
		}
		if (predictedObserveActivities.isEmpty()) return null;

		HashMap<Variable,Variable[]> observeActivitiesToPolygons = new HashMap<Variable, Variable[]>();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		for (Variable fv : predictedObserveActivities) {
			String panel = ((SpatioTemporalSet)fv).getSet().getSymbols()[0];
			for (Variable panelPoly : this.getGeometricSolver().getVariables()) {
				if (panelPoly.getComponent().equals(panel)) {
					if (!observeActivitiesToPolygons.containsKey(fv)) {
						Variable[] panels = new Variable[2];
						panels[0] = panelPoly;
						observeActivitiesToPolygons.put(fv, panels);
					}
					else {
						Variable[] panels = observeActivitiesToPolygons.get(fv);
						panels[1] = panelPoly;
						observeActivitiesToPolygons.put(fv, panels);
						break;
					}
				}
			}
		}

		for (Variable observes : observeActivitiesToPolygons.keySet()) {
			Polygon obsPoly = ((SpatioTemporalSet)observes).getPolygon();
			Polygon panelPoly1 = (Polygon)observeActivitiesToPolygons.get(observes)[0];
			Polygon panelPoly2 = (Polygon)observeActivitiesToPolygons.get(observes)[1];
			boolean inside1 = false;
			boolean inside2 = false;
			if (panelPoly1 != null && GeometricConstraintSolver.getRelation(obsPoly, panelPoly1).equals(GeometricConstraint.Type.INSIDE))
				inside1 = true;
			else if (panelPoly2 != null && GeometricConstraintSolver.getRelation(obsPoly, panelPoly2).equals(GeometricConstraint.Type.INSIDE))
				inside2 = true;
			if (!(inside1 || inside2)) {
				ConstraintNetwork cn = new ConstraintNetwork(null);
				cn.addVariable(observes);
				if (panelPoly1 != null) cn.addVariable(panelPoly1);
				if (panelPoly2 != null) cn.addVariable(panelPoly2);
				ret.add(cn);
			}
		}

		if (ret.isEmpty()) return null;
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		Variable[] vars = metaVariable.getConstraintNetwork().getVariables();
		//for (Variable var : vars) System.out.println("metaVar is: " + var);
		SpatioTemporalSet observeAction = null;
		Polygon[] panels = new Polygon[2];
		int polyCounter = 0;
		for (Variable var : vars) {
			if (var instanceof SpatioTemporalSet) observeAction = (SpatioTemporalSet)var;
			else if (var instanceof Polygon) panels[polyCounter++] = (Polygon)var;
		}

		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		//Create metavalues (one for each possible panel)
		for (int i = 0; i < polyCounter; i++) {
			ConstraintNetwork cn = new ConstraintNetwork(null);
			GeometricConstraint inside = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
			inside.setFrom(observeAction.getPolygon());
			inside.setTo(panels[i]);
			cn.addConstraint(inside);
			cn.setAnnotation(this);
			ret.add(cn);
		}
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public void markResolvedSub(MetaVariable metaVariable,
			ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub

	}

	@Override
	public void draw(ConstraintNetwork network) {
		// TODO Auto-generated method stub

	}

	@Override
	public ConstraintSolver getGroundSolver() {
		return this.metaCS.getConstraintSolvers()[0];
	}

	public ConstraintSolver getActivityNetworkSolver() {
		return MultiConstraintSolver.getConstraintSolver(this.metaCS, ActivityNetworkSolver.class);
	}

	public ConstraintSolver getGeometricSolver() {
		return MultiConstraintSolver.getConstraintSolver(this.metaCS, GeometricConstraintSolver.class);
	}

	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return this.getClass().getSimpleName();
	}

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		// TODO Auto-generated method stub
		return false;
	}

}
