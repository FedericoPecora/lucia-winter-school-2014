package se.oru.aass.lucia2014.multi.spaceTimeSets;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Domain;
import org.metacsp.framework.Variable;
import org.metacsp.framework.multi.MultiVariable;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.symbols.SymbolicVariable;
import org.metacsp.spatial.geometry.Polygon;

import se.oru.aass.lucia2014.meta.spaceTimeSets.LuciaMetaConstraintSolver;

public class SpatioTemporalSet extends MultiVariable {

	public SpatioTemporalSet(ConstraintSolver cs, int id, ConstraintSolver[] internalSolvers, Variable[] internalVars) {
		super(cs, id, internalSolvers, internalVars);
		this.setMarking(LuciaMetaConstraintSolver.Markings.UNSUPPORTED);
	}

	private static final long serialVersionUID = 6109882181961186026L;

	@Override
	public int compareTo(Variable o) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected Constraint[] createInternalConstraints(Variable[] variables) {
		// TODO Auto-generated method stub
		return null;
	}
	
	public void setTask(String task) {
		this.getActivity().setSymbolicDomain(task);
	}
	
	public String getTask() {
		return this.getActivity().getSymbolicVariable().getSymbols()[0];
	}

	@Override
	public void setDomain(Domain d) {
		// TODO Auto-generated method stub
	}

	@Override
	public String toString() {
		String ret = "";
		if (this.getMarking().equals(LuciaMetaConstraintSolver.Markings.UNSUPPORTED)) ret += "*";
		ret += this.getID() + ": ";
		for (int i = 0; i < this.getInternalVariables().length; i++) {
			if (i != this.getInternalVariables().length-1)
				ret += this.getInternalVariables()[i] + " U ";
			else ret += this.getInternalVariables()[i];
		}
		return ret;
	}
	
	public SymbolicVariableActivity getActivity() {
		return (SymbolicVariableActivity)this.getInternalVariables()[0];
	}
	
	public SymbolicVariable getSet() {
		return (SymbolicVariable)this.getInternalVariables()[1];
	}

	public Polygon getPolygon() {
		return (Polygon)this.getInternalVariables()[2];
	}

}
