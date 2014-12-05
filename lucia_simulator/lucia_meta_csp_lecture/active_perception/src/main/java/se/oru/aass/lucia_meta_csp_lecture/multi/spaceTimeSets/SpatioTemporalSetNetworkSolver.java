package se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets;

import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenInterval;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.allenInterval.AllenIntervalNetworkSolver;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;

public class SpatioTemporalSetNetworkSolver extends MultiConstraintSolver {

	private static final long serialVersionUID = -6662473872444687556L;

	protected SpatioTemporalSetNetworkSolver(Class<?>[] constraintTypes, Class<?> variableType, ConstraintSolver[] internalSolvers, int[] ingredients) {
		super(constraintTypes, variableType, internalSolvers, ingredients);
	}
	
	public SpatioTemporalSetNetworkSolver(long origin, long horizon, int numActivities, String[] symbols) {
		super(new Class[] {AllenIntervalConstraint.class, SymbolicValueConstraint.class, GeometricConstraint.class}, SpatioTemporalSet.class, createConstraintSolvers(origin,horizon,numActivities,symbols), new int[] {1,1,1});
	}
	
	protected static ConstraintSolver[] createConstraintSolvers(long origin, long horizon, int numActivities, String[] symbols) {
//		ConstraintSolver[] ret = new ConstraintSolver[] {new ActivityNetworkSolver(origin, horizon, numActivities, symbols), new GeometricConstraintSolver()};
		ConstraintSolver[] ret = new ConstraintSolver[] {new ActivityNetworkSolver(origin, horizon, numActivities), new SymbolicVariableConstraintSolver(symbols, 500, true), new GeometricConstraintSolver()};
//		((SymbolicVariableConstraintSolver)((ActivityNetworkSolver)ret[0]).getConstraintSolvers()[1]).setSingleValue(false);
//		((SymbolicVariableConstraintSolver)((ActivityNetworkSolver)ret[0]).getConstraintSolvers()[1]).setEnumerateSets(false);
		((SymbolicVariableConstraintSolver)ret[1]).setSingleValue(false);
		((SymbolicVariableConstraintSolver)ret[1]).setEnumerateSets(false);
		return ret;
	}

	public AllenIntervalNetworkSolver getTemporalSolver() {
		return (AllenIntervalNetworkSolver) getConstraintSolver(this, AllenIntervalNetworkSolver.class);
	}

	public SymbolicVariableConstraintSolver getSetSolver() {
		return (SymbolicVariableConstraintSolver)this.getConstraintSolvers()[1];
	}

	public ActivityNetworkSolver getActivitySolver() {
		return (ActivityNetworkSolver) getConstraintSolver(this, ActivityNetworkSolver.class);
	}
	
	public GeometricConstraintSolver getGeometricSolver() {
		return (GeometricConstraintSolver) getConstraintSolver(this, GeometricConstraintSolver.class);
	}
	
	@Override
	public boolean propagate() {
		// TODO Auto-generated method stub
		return true;
	}
	
}
