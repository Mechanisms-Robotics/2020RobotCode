package frc2020.util.geometry;

import frc2020.util.Util;

import java.text.DecimalFormat;

/**
 * Represents a displacement in 1 dimension (x).
 */
public class Displacement1d implements State<Displacement1d> {

    protected final double displacement_;

    public Displacement1d() {
        displacement_ = 0.0;
    }

    public Displacement1d(double displacement) {
        displacement_ = displacement;
    }

    /**
     * @return The displacement
     */
    public double x() {
        return displacement_;
    }

    /**
     * Interpolate betten two displacments
     *
     * @param other The other displacement to interpolate with.
     * @param x     The requested value. Should be between 0 and 1.
     * @return
     */
    @Override
    public Displacement1d interpolate(final Displacement1d other, double x) {
        return new Displacement1d(Util.interpolate(displacement_, other.displacement_, x));
    }

    /**
     * Find the distance to another displacement.
     *
     * @param other The displacement to find the distance to
     * @return The distance to the next displacement.
     */
    @Override
    public double distance(final Displacement1d other) {
        return Math.abs(x() - other.x());
    }

    /**
     * Checks to see if the displacement is equal to aother object/displacement
     *
     * @param other The other object, should be a displacement
     * @return If they are equal true, other wise false
     */
    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Displacement1d)) return false;
        return Util.epsilonEquals(x(), ((Displacement1d) other).x());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format("(" + x() + ")");
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x());
    }
}
