package frc.robot.util;

import java.util.Set;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ApplyModuleStates implements SwerveRequest {
    public SwerveModuleState[] m_moduleStates = new SwerveModuleState[0];
    
    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        var moduleRequest = new SwerveModule.ModuleRequest()
            .withUpdatePeriod(parameters.updatePeriod);
        for (int i = 0; i < modulesToApply.length && i < m_moduleStates.length; ++i) {
            /* apply the SwerveModuleState to the module */
            modulesToApply[i].apply(moduleRequest.withState(m_moduleStates[i]));
        }
        return StatusCode.OK;  
    }

    public void setModuleStates(SwerveModuleState[] moduleStates){
        m_moduleStates = moduleStates;
    }
}