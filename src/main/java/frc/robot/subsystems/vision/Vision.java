package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;

import swervelib.SwerveDrive;

/**
 * Class that handles vision logic.
 */
public class Vision {
	/**
	 * {@link SwerveDrive} to write vision updates to.
	 */
	private final SwerveDrive swerveDrive;

	/**
	 * Creates a new Vision.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to update.
	 */
	public Vision(SwerveDrive swerveDrive) {
		this.swerveDrive = swerveDrive;
	}

	/**
	 * Sets the AprilTag ID filter to the appropriate set for the given alliance.
	 *
	 * @param alliance
	 *            The alliance to set it for.
	 */
	public void setTagFilterAlliance(DriverStation.Alliance alliance) {}

	/**
	 * Update the pose estimation inside of {@link #swerveDrive} with data from Limelight.
	 */
	public void updatePoseEstimation() {}
}
