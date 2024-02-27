package frc.robot.extensions;

public class GravityAssistedFeedForward {
    
    private double m_minimum;
    private double m_gravity;
    private double m_offsetAngle;
    private double m_direction;

    /**
     * @param gravityFF Amount of feedforward needed to offset gravity.
     * @param offsetAngle Angle of the mechanism when horisontal (and positive change in angle results in upward motion).
     */
    public GravityAssistedFeedForward(double gravityFF, double offsetAngle) {
        this(0.0, gravityFF, offsetAngle, false);
    }

    /**
     * @param minimumFF Minimum amount of gravity assist regardless of angle.
     * @param gravityFF Amount of additional feedforward needed to offset gravity.
     * @param offsetAngle Angle of the mechanism when horisontal (and positive change in angle results in upward motion).
     */
    public GravityAssistedFeedForward(double minimumFF, double gravityFF, double offsetAngle) {
        this(minimumFF, gravityFF, offsetAngle, false);
    }
 
    /**
     * @param minimumFF Minimum amount of gravity assist regardless of angle. 
     * @param gravityFF Amount of additional feedforward needed to offset gravity.
     * @param offsetAngle Angle of the mechanism when horisontal (and positive change in angle results in upward motion).
     * @param reversed Set this to true if motor voltage and resulting angular change are opposites.
     */
    public GravityAssistedFeedForward(double minimumFF, double gravityFF, double offsetAngle, boolean reversed) {
        this.m_minimum = minimumFF;
        this.m_gravity = gravityFF;
        this.m_offsetAngle = offsetAngle;
        if(reversed) {
            m_direction = -1.0;
        } else {
            m_direction = 1.0;
        }
    }

    /**
     * Calculate the gravity compensating feed forward value for the current angle.
     * This method must be updated periodically.
     * @param angle The current angle of the mechanism as reported by encoders.
     * @return The calculated feed forward value.
     */
    public double calculate(double angle) {
        double relativeRadians = Math.toRadians(angle - m_offsetAngle);
        double cosineValue = Math.cos(relativeRadians);
        return (Math.copySign(m_minimum, cosineValue) +  m_gravity * cosineValue) * m_direction;
    }

    @Override
    public String toString() {
        return "GravityAssistFeedForward[min: " + m_minimum + ", gravity: " + m_gravity + "]";
    }
}
