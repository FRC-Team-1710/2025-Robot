package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.Drive.VisionParameters;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionIOAlgaeSIM extends VisionIOAlgae {

    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;
    private Supplier<VisionParameters> visionParams;

    // Builds the simulated PhotonVision camera and registers it to a VisionSystemSim
    public VisionIOAlgaeSIM(
            String cameraName,
            Transform3d robotToCamera,
            Supplier<VisionParameters> visionParams) {

        super(cameraName); 
        this.visionParams = visionParams;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("Algae Sim");
        }

        var props = new SimCameraProperties();
        props.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        props.setCalibError(0, 0);

        cameraSim = new PhotonCameraSim(camera, props);
        visionSim.addCamera(cameraSim, robotToCamera);

        cameraSim.enableDrawWireframe(true);
    }

    // Updates the simulation state each loop using the robot’s current pose
    @Override
    public void updateResults() {
        visionSim.update(visionParams.get().robotPose());
    }
    
    // Replaces all simulated vision targets with the provided set of models.
    public void updateModels(VisionTargetSim[] models) {
        visionSim.clearVisionTargets();
        for (int i = 0; i < models.length; i++) {
            visionSim.addVisionTargets("Algae" + i, models[i]);
        }
    }
}
