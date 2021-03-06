package se.oru.aass.lucia_meta_csp_lecture.exercises;

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

public class RobotSchedulingMetaConstraint extends MetaConstraint {

	/**
	 * 
	 */
	private static final long serialVersionUID = 4178883694139667765L;

	public RobotSchedulingMetaConstraint(VariableOrderingH varOH,
			ValueOrderingH valOH) {
		super(varOH, valOH);
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {
		
		//get all the "move_base" variables from ground solver
		Variable[] acts = this.getActivityNetworkSolver().getVariables();
		Vector<SymbolicVariableActivity> moveTos = new Vector<SymbolicVariableActivity>(); 
		for (Variable var : acts) {
			if (((SymbolicVariableActivity)var).getSymbolicVariable().getSymbols()[0].equals("move_base")) {
				moveTos.add((SymbolicVariableActivity)var);
			}
		}
		
		//finding pairs of variables that are temporally overlaps and identify each pair as a meta-variable i.e., a constraint network 
		//including two temporally overlapped "move_base"
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
		//get the meta-variable (i.e., a conflict that has to be resolved)
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();
		//Initializing a vector of resolvers
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		
		//each variable here representing two "move_base" that are temporally overlapped
		Variable var1 = conflict.getVariables()[0];
		Variable var2 = conflict.getVariables()[1];
		
		//TODO 
		//a meta-value is a temporal constraint
		//add each constraint to a separate constraint network
		//add your new constraint network to the "ret" vector 
		//you can create a new ConstraintNetwork as follows:
		//ConstraintNetwork resolver0 = new ConstraintNetwork(null);
		
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
		return null;
	}

	@Override
	public Object clone() {
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		return false;
	}

}
