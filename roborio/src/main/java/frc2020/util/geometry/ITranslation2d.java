package frc2020.util.geometry;

/**
 * Defines a state that has a translational element in 2 dimensional space
 *
 * @param <S>
 */
public interface ITranslation2d<S> extends State<S> {
    public Translation2d getTranslation();
}
