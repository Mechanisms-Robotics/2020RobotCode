package frc2020.util;

public class LoggerNotStartedException extends RuntimeException {

    public LoggerNotStartedException(){};

    public LoggerNotStartedException(String message){
        super(message);
    }
}
