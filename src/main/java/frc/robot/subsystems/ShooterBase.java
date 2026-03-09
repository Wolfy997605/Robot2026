package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterBaseConstants;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterBase extends SubsystemBase {
    // initialisation du moteur et de l'encodeur
    private SparkMax shootMotor  = new SparkMax(ShooterBaseConstants.shooterBaseMotorId, MotorType.kBrushless);
    private RelativeEncoder shootEncoder = shootMotor.getEncoder();
    private SparkMaxConfig currentConfig;
    
    // constructeur du sous-système
    public ShooterBase() {

    }

    @Override
    public void periodic() {
        
    }

}