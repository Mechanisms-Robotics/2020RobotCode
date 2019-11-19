package frc2020.util.geometry;

/**
 * Defines a state that contains a rotational element in 2 dimensional space
 *
 * @param <S>
 */
public interface IRotation2d<S> extends State<S> {
    public Rotation2d getRotation();
}
