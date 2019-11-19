package frc2020.util.geometry;

import frc2020.util.CSVWritable;
import frc2020.util.Interpolable;

/**
 * Defines a mathematical state of something
 * (heading and distance for drive tairn, just a rotation for a turret ect.)
 *
 * @param <S>
 * @see Pose2d
 * @see Pose2dWithCurvature
 */
public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
