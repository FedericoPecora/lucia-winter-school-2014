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
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.utility.UI.PolygonFrame;

import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;

import com.sun.org.apache.bcel.internal.generic.INSTANCEOF;

public class ObservabilityMetaConstraint extends MetaConstraint {

	private static final long serialVersionUID = -2618200060652687235L;

	public ObservabilityMetaConstraint(VariableOrderingH varOH,	ValueOrderingH valOH) {
		super(varOH, valOH);
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {
		//Get the focused vars first
		if (((LuciaMetaConstraintSolver)this.metaCS).getCurrentFocus() == null) return null;
		Variable[] observeActivities = ((LuciaMetaConstraintSolver)this.metaCS).getCurrentFocus().getScope();
				
		HashMap<Variable,Variable[]> observeActivitiesToPolygons = new HashMap<Variable, Variable[]>();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		for (Variable fv : observeActivities) {
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
			if (!(GeometricConstraintSolver.getRelation(obsPoly, panelPoly1).equals(GeometricConstraint.Type.INSIDE)
					|| GeometricConstraintSolver.getRelation(obsPoly, panelPoly2).equals(GeometricConstraint.Type.INSIDE))) {
				ConstraintNetwork cn = new ConstraintNetwork(null);
				cn.addVariable(observes);
				cn.addVariable(panelPoly1);
				cn.addVariable(panelPoly2);
				ret.add(cn);
			}
		}
		
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		Variable[] vars = metaVariable.getConstraintNetwork().getVariables();
		//for (Variable var : vars) System.out.println("metaVar is: " + var);
		SpatioTemporalSet observeAction = null;
		Polygon panel1 = null;
		Polygon panel2 = null;
		for (Variable var : vars) {
			if (var instanceof SpatioTemporalSet) observeAction = (SpatioTemporalSet)var;
			else if (var instanceof Polygon && panel1 == null) panel1 = (Polygon)var;
			else panel2 = (Polygon)var;
		}
		
		//Create metavalues (one for each possible panel)
		ConstraintNetwork cn1 = new ConstraintNetwork(null);
		ConstraintNetwork cn2 = new ConstraintNetwork(null);
		GeometricConstraint inside1 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside1.setFrom(observeAction.getPolygon());
		inside1.setTo(panel1);
		GeometricConstraint inside2 = new GeometricConstraint(GeometricConstraint.Type.INSIDE);
		inside2.setFrom(observeAction.getPolygon());
		inside2.setTo(panel2);
		cn1.addConstraint(inside1);
		cn2.addConstraint(inside2);
		
		cn1.setAnnotation(this);
		cn2.setAnnotation(this);
		
		return new ConstraintNetwork[] {cn1,cn2};
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
		return null;
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
