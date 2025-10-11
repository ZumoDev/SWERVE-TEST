package frc.robot;

public class Constants {

    /*
     * Variables utilizadas para reducir la velocidad de los motores.
     */
    public static final class MovementReductions {
        public static final double velPercentage = 0.5;
        public static final double turnPercentage = 0.5;
    }

    public static final class SwerveModules {
        public static final class PIDConstants {
        /*
         * Valores del controlador PID
         * kP = "Proporcionalidad", qué tanto voltaje se envía a los motores, proporcional al margen de "error".
         * kI = "Integral", 
         * kD = "Derivativa", qué tan rápido acercarse al destino.
         */
            public static final double swervekP = 0.05;
            public static final double swervekI = 0;
            public static final double swervekD = 0;
        }

        public static final class FeedforwardConstants {
            /*
             * Valores del controlador Feedforward
             * kS = "Fricción", voltaje mínimo para vencer la fricción estática.
             * kG = "Gravedad", voltaje mínimo para vencer la gravedad.
             * kV = "Velocidad", la ganancia de velocidad.
             * kA = "Aceleración", la ganancia de aceleración
            */
            public static final double swervekS = 0;
            public static final double serverkG = 0;
            public static final double swervekV = 0;
            public static final double swervekA = 0;
        }
    }

    public static final class LimelightController {
        public static final double limelightkP = 0.15;
        public static final double limelightkI = 0.0;
        public static final double limelightkD = 0.0;
        public static final double limelightXError = 0.0;
        public static final class ElevatorController {
            public static final double kP = 0.01;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double limelightYError = 0.0;
        }
    }

    public static final class Limelight3DController {
        public static final double limelight3DkP = 0.0;
        public static final double limelight3DkI = 0.0;
        public static final double limelight3DkD = 0.0;
        public static final double limelight3DXError = 0.0;
        public static final double limelight3DZError = 0.0;
        public static final double limelight3DYaw = 0.0;
    }
    
    public static final class DriverMotorIDs {
        /*
         * Los motores están acomodados como cuadrantes de un mapa cartesiano.
         * 2|1
         * 3|4
         * Si el motor toca los componentes externos, se indica con un "Ex".
         * Si el motor se acerca más a los componentes internos, se indica con un "In".
         * El formato de los nombres es el siguiente: Driver<Ex/In><1/2/3/4>
         * 
         * TODOS LOS MOTORES YA ESTÁN DECLARADOS EN COMMANDSWERVEDRIVETRAIN.JAVA
         */
        public static final int DriverIn1 = 5;
        public static final int DriverEx1 = 6;
        public static final int DriverIn2 = 4;
        public static final int DriverEx2 = 0;
        public static final int DriverIn3 = 1;
        public static final int DriverEx3 = 2;
        public static final int DriverIn4 = 8;
        public static final int DriverEx4 = 7;
    }

    public static final class ElevatorMotorIDs {
        /*
         * Motores relacionados con el movimiento del elevador.
         * Se indica la ubicación de cada motor, usando derecha "R" e izquierda "L", viendo el robot por detrás del plástico.
         */
        public static final int ElevatorL = 18;
        public static final int ElevatorR = 47;
    }

    public static final class IntakeMotorIDs {
        /*
         * Motores relacionados con el movimiento del Intake del robot.
         * Se utiliza el mismo nombramiento que con ElevatorMotorIDs.
         */
        public static final int IntakeMotor = 23;
        public static final int IntakeWrist = 15;
    }

    public static final class MechMotorIDs {
        /*
         * Motores relacinonados con el mecanismo debajo del Intake. No tengo la menor idea de qué sea o cómo se llame.
         * Igualmente, utiliza el mismo nombramiento que ElevatorMotorIDs.
         */
        public static final int WeirdR = 32;
        public static final int WeirdL = 31;
    }
}
