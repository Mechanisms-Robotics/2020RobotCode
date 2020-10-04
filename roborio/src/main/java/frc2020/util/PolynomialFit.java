package frc2020.util;

import java.lang.Math;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class PolynomialFit {
    public static double[] fit(double[][] points) {
        return fit(points, points.length-1);
    }

    public static double[] fit(double[][] points, int pDegree) {
        double[][] coeffecientMatrixArray = new double[points.length][pDegree+1];
        double[] outputVectorArray = new double[points.length];

        for (int i = 0; i < points.length; i++) {
            for (int j = 0; j < pDegree+1; j++) {
               coeffecientMatrixArray[i][j] = Math.pow(points[i][0], pDegree-j);
            }
            outputVectorArray[i] = points[i][1];
        }

        RealMatrix coeffecientMatrix = MatrixUtils.createRealMatrix(coeffecientMatrixArray);
        RealVector outputVector = MatrixUtils.createRealVector(outputVectorArray);

        RealVector solutionVector = (pDegree==points.length-1) ? MatrixUtils.inverse(coeffecientMatrix).operate(outputVector) :
            MatrixUtils.inverse(coeffecientMatrix.transpose().multiply(coeffecientMatrix)).multiply(coeffecientMatrix.transpose()).operate(outputVector);

        return solutionVector.toArray();
    }

    public static double[] fit(double[][] points, double[] derivatives) {
        double[][] coeffecientMatrixArray = new double[points.length+derivatives.length][points.length+derivatives.length];
        double[] outputVectorArray = new double[points.length+derivatives.length];

        for (int i = 0; i < points.length; i++) {
            for (int j = 0; j < points.length+derivatives.length; j++) {
               coeffecientMatrixArray[i][j] = Math.pow(points[i][0], (points.length*2)-(j+1));
            }
            outputVectorArray[i] = points[i][1];
        }

        for (int i = points.length; i < points.length+derivatives.length; i++) {
            for (int j = 0; j < points.length+derivatives.length; j++) {
               coeffecientMatrixArray[i][j] = (j != points.length+derivatives.length-1 ? (points.length*2)-(j+1) : 0)*Math.pow(points[i-points.length][0], (points.length*2)-(j+(j != points.length+derivatives.length-1 ? 2 : 1)));
            }
            outputVectorArray[i] = derivatives[i-points.length];
        }

        RealMatrix coeffecientMatrix = MatrixUtils.createRealMatrix(coeffecientMatrixArray);
        RealVector outputVector = MatrixUtils.createRealVector(outputVectorArray);

        RealVector solutionVector = MatrixUtils.inverse(coeffecientMatrix).operate(outputVector);

        return solutionVector.toArray();
    }

    public static double getValue(double[] coeffecients, double x) {
        double sum = 0.0;
        for (int i = 0; i < coeffecients.length; i++) {
            sum += coeffecients[i]*(Math.pow(x, coeffecients.length-(i+1)));
        }
        return sum;
    }

    public static String coeffecientsToFunctionString(double[] coeffecients) {
        String coeffecientFunctionString = "f(x) = ";
        for (int i = 0; i < coeffecients.length; i++) {
            coeffecientFunctionString += coeffecients[i];
                if (coeffecients.length-i > 1) {
                    coeffecientFunctionString += "x";
                    if (coeffecients.length-i > 2) {
                       coeffecientFunctionString +=  "^"+(coeffecients.length-1-i);
                    }
                    coeffecientFunctionString += " + ";
                }
        }
        return coeffecientFunctionString;
    }
}
