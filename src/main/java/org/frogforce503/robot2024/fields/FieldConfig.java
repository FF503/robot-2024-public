package org.frogforce503.robot2024.fields;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.AllianceFlipUtil;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldConfig {
    
    // CONSTANTS
    public Translation2d FIELD_DIMENSIONS;

    // Field Variables
    private final double FIELD_X = Units.feetToMeters(54);
    private final double FIELD_Y = Units.feetToMeters(27);

    private double BLUE_X_WallToTag6;
    private double BLUE_X_WallToNoteK;
    private double BLUE_X_WallToStartLine;
    private double BLUE_X_NoteIToTruss;
    private double BLUE_X_WingLineToCenterLine;
    private double BLUE_Y_SubwooferToAmpWall;
    private double BLUE_Y_SubwooferToSource;
    private double BLUE_Y_NoteIToNoteJ;
    private double BLUE_Y_NoteJToNoteK;
    private double BLUE_Y_AmpWallToNoteK;
    private double BLUE_Y_AmpWallToTruss;
    private double BLUE_Y_SourceWallToTruss;

    private double Y_SourceWallToNoteD;
    private double Y_NoteDToNoteE;
    private double Y_NoteEToNoteF;
    private double Y_NoteFToNoteG;
    private double Y_NoteGToNoteH;

    private double RED_X_WallToTag5;
    private double RED_X_WallToStartLine;
    private double RED_X_WingLineToCenterLine;
    private double RED_X_WallToNoteC;
    private double RED_X_NoteAToTruss;
    private double RED_Y_NoteAToNoteB;
    private double RED_Y_NoteBToNoteC;
    private double RED_Y_AmpWallToNoteC;
    private double RED_Y_AmpWallToTruss;
    private double RED_Y_SourceWallToTruss;
    private double RED_Y_SubWooferToAmpWall;
    private double RED_Y_SubwooferToSource;

    public Translation2d NOTE_A, NOTE_B, NOTE_C, NOTE_I, NOTE_J, NOTE_K;
    public double BLUE_INITIATION_LINE, BLUE_WING_LINE, RED_INITIATION_LINE, RED_WING_LINE;
    public Translation2d BLUE_STAGE_LEFT, BLUE_CENTER_STAGE, BLUE_STAGE_RIGHT, RED_STAGE_LEFT, RED_CENTER_STAGE, RED_STAGE_RIGHT;
    
    public Translation2d BLUE_SPEAKER, RED_SPEAKER;
    public Translation2d NOTE_D, NOTE_E, NOTE_F, NOTE_G, NOTE_H;

    public AprilTagFieldLayout fieldLayout;

    private static FieldConfig instance;

    private void loadConstants(String file) throws FileNotFoundException, IOException, ParseException {
        // READ JSON FILE AND FILL IN ALL THOSE PRIVATE DOUBLES

        JSONObject field = (JSONObject) new JSONParser().parse(new FileReader(
                Filesystem.getDeployDirectory().getAbsolutePath() + "/fields/" + file));

        BLUE_X_WallToTag6 = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_X_WallToTag6").toString()));
        BLUE_X_WallToNoteK = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_X_WallToNoteK").toString()));
        BLUE_X_WallToStartLine = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_X_WallToStartLine").toString()));
        BLUE_X_NoteIToTruss = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_X_NoteIToTruss").toString()));
        BLUE_X_WingLineToCenterLine = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_X_WingLineToCenterLine").toString()));
        BLUE_Y_SubwooferToAmpWall = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_SubwooferToAmpWall").toString()));
        BLUE_Y_SubwooferToSource = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_SubwooferToSource").toString()));
        BLUE_Y_NoteIToNoteJ = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_NoteIToNoteJ").toString()));
        BLUE_Y_NoteJToNoteK = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_NoteJToNoteK").toString()));
        BLUE_Y_AmpWallToNoteK = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_AmpWallToNoteK").toString()));
        BLUE_Y_AmpWallToTruss = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_AmpWallToTruss").toString()));
        BLUE_Y_SourceWallToTruss = Units.inchesToMeters(Double.parseDouble(field.get("BLUE_Y_SourceWallToTruss").toString()));

        Y_SourceWallToNoteD = Units.inchesToMeters(Double.parseDouble(field.get("Y_SourceWallToNoteD").toString()));
        Y_NoteDToNoteE = Units.inchesToMeters(Double.parseDouble(field.get("Y_NoteDToNoteE").toString()));
        Y_NoteEToNoteF = Units.inchesToMeters(Double.parseDouble(field.get("Y_NoteEToNoteF").toString()));
        Y_NoteFToNoteG = Units.inchesToMeters(Double.parseDouble(field.get("Y_NoteFToNoteG").toString()));
        Y_NoteGToNoteH = Units.inchesToMeters(Double.parseDouble(field.get("Y_NoteGToNoteH").toString()));

        RED_X_WallToTag5 = Units.inchesToMeters(Double.parseDouble(field.get("RED_X_WallToTag5").toString()));
        RED_X_WallToStartLine = Units.inchesToMeters(Double.parseDouble(field.get("RED_X_WallToStartLine").toString()));
        RED_X_WingLineToCenterLine = Units.inchesToMeters(Double.parseDouble(field.get("RED_X_WingLineToCenterLine").toString()));
        RED_X_WallToNoteC = Units.inchesToMeters(Double.parseDouble(field.get("RED_X_WallToNoteC").toString()));
        RED_X_NoteAToTruss = Units.inchesToMeters(Double.parseDouble(field.get("RED_X_NoteAToTruss").toString()));
        RED_Y_NoteAToNoteB = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_NoteAToNoteB").toString()));
        RED_Y_NoteBToNoteC = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_NoteBToNoteC").toString()));
        RED_Y_AmpWallToNoteC = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_AmpWallToNoteC").toString()));
        RED_Y_AmpWallToTruss = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_AmpWallToTruss").toString()));
        RED_Y_SourceWallToTruss = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_SourceWallToTruss").toString()));
        RED_Y_SubWooferToAmpWall = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_SubWooferToAmpWall").toString()));
        RED_Y_SubwooferToSource = Units.inchesToMeters(Double.parseDouble(field.get("RED_Y_SubwooferToSource").toString()));


        // NEXT DEFINE THE TRANSLATION2D
        BLUE_INITIATION_LINE = BLUE_X_WallToStartLine;
        BLUE_WING_LINE = FIELD_X / 2 - BLUE_X_WingLineToCenterLine;
        RED_INITIATION_LINE = FIELD_X - RED_X_WallToStartLine;
        RED_WING_LINE = FIELD_X / 2 + RED_X_WingLineToCenterLine;

        NOTE_K = new Translation2d(BLUE_X_WallToNoteK, FIELD_Y - BLUE_Y_AmpWallToNoteK);
        NOTE_J = NOTE_K.minus(new Translation2d(0, BLUE_Y_NoteJToNoteK));
        NOTE_I = NOTE_J.minus(new Translation2d(0, BLUE_Y_NoteIToNoteJ));
        NOTE_C = new Translation2d(FIELD_X - RED_X_WallToNoteC, FIELD_Y - RED_Y_AmpWallToNoteC);
        NOTE_B = NOTE_C.minus(new Translation2d(0, RED_Y_NoteBToNoteC));
        NOTE_A = NOTE_B.minus(new Translation2d(0, RED_Y_NoteAToNoteB));
        NOTE_D = new Translation2d(FIELD_X / 2, Y_SourceWallToNoteD);
        NOTE_E = NOTE_D.plus(new Translation2d(0, Y_NoteDToNoteE));
        NOTE_F = NOTE_E.plus(new Translation2d(0, Y_NoteEToNoteF));
        NOTE_G = NOTE_F.plus(new Translation2d(0, Y_NoteFToNoteG));
        NOTE_H = NOTE_G.plus(new Translation2d(0, Y_NoteGToNoteH));

        BLUE_STAGE_LEFT = new Translation2d(BLUE_WING_LINE - Units.inchesToMeters(8), FIELD_Y - BLUE_Y_AmpWallToTruss - Units.inchesToMeters(2.275));
        BLUE_CENTER_STAGE = NOTE_I.plus(new Translation2d(BLUE_X_NoteIToTruss + Units.inchesToMeters(6), 0));
        BLUE_STAGE_RIGHT = new Translation2d(BLUE_WING_LINE - Units.inchesToMeters(8), BLUE_Y_SourceWallToTruss + Units.inchesToMeters(2.275));
        RED_STAGE_LEFT = new Translation2d(RED_WING_LINE + Units.inchesToMeters(8), RED_Y_SourceWallToTruss + Units.inchesToMeters(2.275));
        RED_CENTER_STAGE = NOTE_A.minus(new Translation2d(RED_X_NoteAToTruss + Units.inchesToMeters(6), 0));
        RED_STAGE_RIGHT = new Translation2d(RED_WING_LINE + Units.inchesToMeters(8), FIELD_Y - RED_Y_AmpWallToTruss - Units.inchesToMeters(2.275));

        BLUE_SPEAKER = new Translation2d(0, NOTE_J.getY());
        RED_SPEAKER = new Translation2d(FIELD_X, NOTE_B.getY());

        FIELD_DIMENSIONS = new Translation2d(FIELD_X, FIELD_Y);

        fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    // remove RobotContainer.red() as it doesn't belong there, it belongs more in here
    private boolean red() {
        return RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;
    }

    private Translation2d flip(Translation2d red, Translation2d blue) {
        if (red())
            AllianceFlipUtil.override();
        return AllianceFlipUtil.apply(red() ? red : blue);

        // return red() ? red : blue;
    }

    private double flip(double red, double blue) {
        if (red())
            AllianceFlipUtil.override();
        return AllianceFlipUtil.apply(red() ? red : blue);
        // return red() ? red : blue;
    }
    
    // FLIPPED POINTS FOR AUTO GENERATION
    /*
     * BLUE: I, RED: A
     */
    public Translation2d NOTE_SOURCE_SIDE() {
        return flip(NOTE_A, NOTE_I);
    }
    
    /*
     * BLUE: J, RED: B
     */
    public Translation2d NOTE_CENTER() {
        return flip(NOTE_B, NOTE_J);
    }

    /*
     * BLUE: K, RED: C
     */
    public Translation2d NOTE_AMP_SIDE() {
        return flip(NOTE_C, NOTE_K);
    }

    public double INITIATION_LINE_X() {
        return flip(RED_INITIATION_LINE, BLUE_INITIATION_LINE);
    }

    public double WING_LINE_X() {
        return flip(RED_WING_LINE, BLUE_WING_LINE);
    }

    public Translation2d SPEAKER_AUTON() {
        return flip(RED_SPEAKER, BLUE_SPEAKER);
    }

    public Translation2d CENTERSTATE() {
        return flip(RED_CENTER_STAGE, BLUE_CENTER_STAGE);
    }

    public Translation2d STAGE_SOURCE_SIDE() {
        return flip(RED_STAGE_LEFT, BLUE_STAGE_RIGHT);
    }

    public Translation2d STAGE_AMP_SIDE() {
        return flip(RED_STAGE_RIGHT, BLUE_STAGE_LEFT);
    }

    // override if field is measured otherwise
    public Translation2d getFieldDimensions() {
        return new Translation2d(Units.feetToMeters(54), Units.feetToMeters(27));
    }
    
    public void setVenue(VENUE venue) {
        try {
            loadConstants(venue.filePath);
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    /**
     * 2023: https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf
     * 2024: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
     * 2025: 
     * 
     * Current: Set for 2024 tags
     * Future: Set to 2025 tags
     * 
     * @param tagID tag ID to check
     * @return Is the tag part of my current alliance
     */
    public boolean isMyAlliance(int tagID) {
        return MathUtils.equalsOneOf(tagID, 
                red()
                ? new double[] {3, 4, 5, 11, 12, 13} // red tags
                : new double[] {6, 7, 8, 14, 15, 16} // blue tags
            );
    }

    public Pose2d getTagById(int id) {
        var t = fieldLayout.getTagPose(id);
        return t.isPresent() ? t.get().toPose2d() : null;
    }

    public Translation2d getSpeakerFromAlliance() {
        return red() ? RED_SPEAKER : BLUE_SPEAKER;
    }

    public static FieldConfig getInstance() {
        if (instance == null) {
            instance = new FieldConfig();
        }
        return instance;
    }
    

    public enum VENUE {
        SHOP("Shop.json");

        public String filePath;
        VENUE(String p) {
            filePath = p;
        }
    }
}
