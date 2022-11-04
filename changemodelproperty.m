function [osimmodel]=changemodelproperty(osimmodel,Data)
import org.opensim.modeling.*;
for i=0:1:osimmodel.getMuscles().getSize()-1
        CurrentMuscle=osimmodel.getMuscles().get(i);
%         CurrentMuscle.set_tendon_slack_length();
        RandomTendonStrain=Data.TendonStrainBound(1) + (diff(Data.TendonStrainBound)).*rand();
        DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).set_tendon_strain_at_one_norm_force(RandomTendonStrain);
        RandomPassiveFiber=Data.PassiveFiberBound(1) + (diff(Data.PassiveFiberBound)).*rand();
        DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).set_passive_fiber_strain_at_one_norm_force(RandomPassiveFiber);
        osimmodel.initSystem();
end
end