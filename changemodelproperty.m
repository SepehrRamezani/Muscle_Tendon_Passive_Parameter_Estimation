function [osimmodel]=changemodelproperty(osimmodel,Coordlable,Data,Optimizedflag)
import org.opensim.modeling.*;

if Optimizedflag
    OptData=importdata(Data.(Coordlable).ParamSimulPath);
    Muscname=Data.SimMusclename;
    for m=1:1:osimmodel.getMuscles().getSize()-1
        M_indx=find(contains(OptData.colheaders,Muscname(m))&~contains(OptData.colheaders,'force'));
        CurrentMuscle=osimmodel.getMuscles().get(Muscname(m));
        for y=1:length(M_indx)          
            if contains(OptData.colheaders(M_indx(y)),'tendon_slack')
                OptTSL=OptData.data(1,M_indx(y));
                DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).set_tendon_slack_length(OptTSL);
            elseif contains(OptData.colheaders(M_indx(y)),'passive_fiber_')
                OptPassiveFiber=OptData.data(1,M_indx(y));
                DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).set_passive_fiber_strain_at_one_norm_force(OptPassiveFiber);
            elseif contains(OptData.colheaders(M_indx(y)),'tendon_strain')
                OptTendonStrain=OptData.data(1,M_indx(y));
                DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).set_tendon_strain_at_one_norm_force(OptTendonStrain);
            end
        end
        
    end
    osimmodel.initSystem();
    osimmodel.print(insertBefore(char(Data.(Coordlable).ModelPath),".osim","_optimized"));
else
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
end