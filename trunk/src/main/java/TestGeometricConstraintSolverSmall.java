
import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.utility.UI.PolygonFrame;


public class TestGeometricConstraintSolverSmall {

	public static void main(String[] args) {

		GeometricConstraintSolver solver = new GeometricConstraintSolver();
		Variable var = solver.createVariable();
		
		Polygon p0 = (Polygon)var;
		p0.setMovable(true);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", solver.getConstraintNetwork(), 0.1f);
	}

}
