package se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets;

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
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.time.Bounds;

public class SchedulingMetaConstraint extends MetaConstraint {

	public SchedulingMetaConstraint(VariableOrderingH varOH,
			ValueOrderingH valOH) {
		super(varOH, valOH);
		// TODO Auto-generated constructor stub
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {
		//TODO: comment this code, explain what it does
		Variable[] acts = this.getActivityNetworkSolver().getVariables();
		Vector<SymbolicVariableActivity> moveTos = new Vector<SymbolicVariableActivity>(); 
		for (Variable var : acts) {
			if (((SymbolicVariableActivity)var).getSymbolicVariable().getSymbols()[0].equals("move_base")) {
				moveTos.add((SymbolicVariableActivity)var);
			}
		}

		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		for (int i = 0; i < moveTos.size()-1; i++) {
			for (int j = i+1; j < moveTos.size(); j++) {
				Bounds bi = new Bounds(moveTos.get(i).getTemporalVariable().getEST(), moveTos.get(i).getTemporalVariable().getEET());
				Bounds bj = new Bounds(moveTos.get(j).getTemporalVariable().getEST(), moveTos.get(j).getTemporalVariable().getEET());
				if (bi.intersectStrict(bj) != null) {
					ConstraintNetwork cn = new ConstraintNetwork(null);
					cn.addVariable(moveTos.get(i).getVariable());
					cn.addVariable(moveTos.get(j).getVariable());
					ret.add(cn);
				}
			}
		}
		if (!ret.isEmpty()) {
			return ret.toArray(new ConstraintNetwork[ret.size()]);			
		}	
		return (new ConstraintNetwork[0]);
		
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		Variable var1 = conflict.getVariables()[0];
		Variable var2 = conflict.getVariables()[1];
		
		//TODO: erase from here...
		//TODO: provide incomme
		AllenIntervalConstraint before01 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before01.setFrom(var1);			
		before01.setTo(var2);
		ConstraintNetwork resolver0 = new ConstraintNetwork(null);
		resolver0.addConstraint(before01);
		ret.add(resolver0);
	
		AllenIntervalConstraint before10 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before);
		before10.setFrom(var2);			
		before10.setTo(var1);
		ConstraintNetwork resolver = new ConstraintNetwork(null);
		resolver.addConstraint(before10);
		ret.add(resolver);
		//TODO: ... to here.
		
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public void markResolvedSub(MetaVariable metaVariable, ConstraintNetwork metaValue) {
	}

	@Override
	public void draw(ConstraintNetwork network) {
	}

	@Override
	public ConstraintSolver getGroundSolver() {
		return this.metaCS.getConstraintSolvers()[0];
	}

	public ConstraintSolver getActivityNetworkSolver() {
		return MultiConstraintSolver.getConstraintSolver(this.metaCS, ActivityNetworkSolver.class);
	}

	@Override
	public String toString() {
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
