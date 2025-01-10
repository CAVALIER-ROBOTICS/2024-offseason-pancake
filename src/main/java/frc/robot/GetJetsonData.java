// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class GetJetsonData {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("TAGDATA");

    private static Pose3d getFieldToTagTransform() {
        double x = table.getEntry("fieldtag_x").getDouble(0);
        double y = table.getEntry("fieldtag_y").getDouble(0);
        double z = table.getEntry("fieldtag_z").getDouble(0);
        double rotx = table.getEntry("fieldtag_xRot").getDouble(0);
        double roty = table.getEntry("fieldtag_yRot").getDouble(0);
        double rotz = table.getEntry("fieldtag_zRot").getDouble(0);
        return new Pose3d(x, y, z, new Rotation3d(rotx, roty, rotz));
    }

    private static Transform3d getCamToTagTransform() {
        double x = table.getEntry("camtag_x").getDouble(0);
        double y = table.getEntry("camtag_y").getDouble(0);
        double z = table.getEntry("camtag_z").getDouble(0);
        double rotx = table.getEntry("camtag_xRot").getDouble(0);
        double roty = table.getEntry("camtag_yRot").getDouble(0);
        double rotz = table.getEntry("camtag_zRot").getDouble(0);
        return new Transform3d(x, y, z, new Rotation3d(rotx, roty, rotz));
    }

    public static Pose3d getNVIDIAPose() {
        Pose3d fieldToTag = getFieldToTagTransform();
        Transform3d camToTag = getCamToTagTransform();

        Pose3d p = fieldToTag.transformBy(camToTag.inverse());
        return p;
    }
    public static Pose2d getNVIDIAPose2d() {
        Pose3d fieldToTag = getFieldToTagTransform();
        Transform3d camToTag = getCamToTagTransform();
        // System.out.println(camToTag);
        Pose3d p = fieldToTag.transformBy(camToTag.inverse());
        return new Pose2d(new Translation2d(p.getX(),p.getY()),new Rotation2d(p.getRotation().getY()));
    }
}
