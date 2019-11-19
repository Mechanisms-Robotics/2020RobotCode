package frc2020.util.geometry;

/**
 * Defines a state that represents a pose in 2 dimensional space.
 *
 * @param <S>
 */
public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public Pose2d getPose();

    public S transformBy(Pose2d transform);

    public S mirror();
}
