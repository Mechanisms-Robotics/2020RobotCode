package frc2020.util.geometry;

/**
 * Defines a state that has curvature
 *
 * @param <S>
 */
public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
