function [osimmodel,Muscleinfo]=Modelcreator(Hipangle,Hiplable,Data,osimmodel)
import org.opensim.modeling.*;
% osimmodel = Model('subject_walk_armless_RLeg_justknee.osim');
Kneerange=90*pi()/180;
%% change the name
osimmodel.setName(['Hip_' num2str(Hipangle)])
%% Setup angles
%%% Pelvis
modeljointSet=osimmodel.getJointSet();
Pelvisjoint=modeljointSet.get(0);
Pelvisweldjoint=WeldJoint.safeDownCast(Pelvisjoint);
hipfram=Pelvisweldjoint.get_frames(0);
hipfram.set_orientation(Vec3(0,0,(90-Hipangle)/180*pi()));
%%% Hip_flexion
modelCoordSet = osimmodel.getCoordinateSet();
Hipcoord = modelCoordSet.get(0);
Hipcoord.setDefaultValue(Hipangle/180*pi());
%%% Knee_corrdiante
%% setup muscle properties

if Data.DeGrooteflage
    DeGrooteFregly2016Muscle().replaceMuscles(osimmodel);
end
c=0;
%% Setup Muscles
for m = 0:osimmodel.getMuscles().getSize()-1
    Muscname=osimmodel.getMuscles().get(m).getName();
    frcset = osimmodel.updForceSet().get(Muscname);
    if ~sum(strcmp(char(frcset.getName()), Data.SimMusclename))
        isremove=osimmodel.updForceSet().remove(frcset);
    else
        c=c+1;
            musc=Muscle.safeDownCast(frcset);
            
            musc.set_ignore_activation_dynamics(true);
            
            if Data.DeGrooteflage
%                 if sum(strcmp(char(frcset.getName()), Data.ComplianacMusclename))
                    musc.set_ignore_tendon_compliance(false);
%                 else
%                     musc.set_ignore_tendon_compliance(true);
%                 end
                dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
                dgf.set_min_control(0.0);
                dgf.set_max_control(0.0);
                dgf.set_active_force_width_scale(1);
                dgf.set_tendon_compliance_dynamics_mode('implicit');
                dgf.set_ignore_passive_fiber_force(false);
            else
                musc.set_min_control(0.0);
                musc.set_max_control(0.0);
            end
            
            
            % if strcmp(char(musc.getName()), 'soleus_r')
            %   dgf.set_active_force_width_scale(10);
            %   dgf.set_ignore_passive_fiber_force(true);
            %   musc.set_max_isometric_force(1.5 * musc.get_max_isometric_force());
            %   dgf.set_ignore_passive_fiber_force(true);
            % end
        
    end
    
end
state=osimmodel.initSystem();
for corindx = 1:length(Data.ActiveCoordinates)
osimmodel.updCoordinateSet().get(Data.ActiveCoordinates(corindx)).set_locked(false);
end
osimmodel.initSystem();
KneeCoor=osimmodel.updCoordinateSet().get(Data.ActiveCoordinates(1));

newi=0;

%% change TSL to avoid buckling and save muscle information
for i=0:1:osimmodel.getMuscles().getSize()-1
    k=0;
    u=0;
    %% finding minimum MTL 
        %%Knee deformation
        CurrentMuscle=osimmodel.getMuscles().get(i);
    for q=0:0.3:Kneerange
        k=k+1;
        
        KneeCoor.setValue(state, q);
        osimmodel.realizePosition(state);
        musclelength(k)=CurrentMuscle.getLength(state);
    end
    
    if sum(strcmp(char(CurrentMuscle.getName),Data.ComplianacMusclename))&& sum(contains(Data.ActiveCoordinates,'ankle_angle_r'))
        AnkleCoor=osimmodel.updCoordinateSet().get(Data.ActiveCoordinates(2));
        KneeCoor.setValue(state, Kneerange);
        for q=AnkleCoor.getRangeMin:0.1:AnkleCoor.getRangeMax
            u=u+1;
            AnkleCoor.setValue(state, q);
            osimmodel.realizePosition(state);
            musclelength(k+u)=CurrentMuscle.getLength(state);
        end
    end
    
    Muscleinfo.MinMTCLength(i+1)=min(musclelength);
    
    if Muscleinfo.MinMTCLength(i+1) < CurrentMuscle.get_tendon_slack_length()
        warning('buckeling will be happend in %s',CurrentMuscle.getName())
        TSlack(i+1)=0.98*Muscleinfo.MinMTCLength(i+1);
        CurrentMuscle.set_tendon_slack_length(TSlack(i+1));
        Muscleinfo.TSlack(i+1)=TSlack(i+1);
    else
        
        Muscleinfo.TSlack(i+1)=CurrentMuscle.get_tendon_slack_length();
        Muscleinfo.MaxIso(i+1)=CurrentMuscle.get_max_isometric_force(); 
    end
    %     dgf = DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
    Muscleinfo.Passive(i+1)=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_passive_fiber_strain_at_one_norm_force();
    if sum(strcmp(char(CurrentMuscle.getName()), Data.ComplianacMusclename))
        newi=newi+1;
        Muscleinfo.Tstrain(newi)= DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_tendon_strain_at_one_norm_force();
    end
end
osimmodel.initSystem();

osimmodel.print(Data.(Hiplable).ModelPath);

end
