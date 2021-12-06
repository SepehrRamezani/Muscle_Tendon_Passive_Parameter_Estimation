function [osimmodel,TSlack,Passive,MinMTCLength]=Modelcreator(Hipangle,SimMusclename,ComplianacMusclename,DeGrooteflage,osimmodel)
import org.opensim.modeling.*;
% osimmodel = Model('subject_walk_armless_RLeg_justknee.osim');
Qrange=90*pi()/180;
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

if DeGrooteflage
    DeGrooteFregly2016Muscle().replaceMuscles(osimmodel);
end
c=0;

for m = 0:osimmodel.getForceSet().getSize()-1
    frcset = osimmodel.updForceSet().get(c);
    if ~sum(strcmp(char(frcset.getName()), SimMusclename))
        isremove=osimmodel.updForceSet().remove(c);
    else
        c=c+1;
        if ~strcmp(char(frcset.getName()), 'knee_act')
            musc=Muscle.safeDownCast(frcset);
            
            musc.set_ignore_activation_dynamics(true);
            
            if DeGrooteflage
%                 if sum(strcmp(char(frcset.getName()), ComplianacMusclename))
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
    
end
state=osimmodel.initSystem();
KneeCoor=osimmodel.updCoordinateSet().get(1);
for i=0:1:osimmodel.getMuscles().getSize()-1
    k=0;
    for q=0:0.3:Qrange
        k=k+1;
        
        KneeCoor.setValue(state, q);
        osimmodel.realizePosition(state);
        CurrentMuscle=osimmodel.getMuscles().get(i);
        musclelength(k)=CurrentMuscle.getLength(state);
    end
    MinMTCLength(i+1)=min(musclelength);
    
    if MinMTCLength(i+1) < CurrentMuscle.get_tendon_slack_length()
        warning('buckeling will be happend in %s',CurrentMuscle.getName())
        TSlack(i+1)=0.98*MinMTCLength(i+1);
        CurrentMuscle.set_tendon_slack_length(TSlack(i+1));
    else
        
        TSlack(i+1)=CurrentMuscle.get_tendon_slack_length();
    end
%     dgf = DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
        Passive(i+1)=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_passive_fiber_strain_at_one_norm_force();
end
osimmodel.initSystem();
if DeGrooteflage
    osimmodel.print([cd '\ModelGenerator\OneDOF_Knee_DeGroote' num2str(Hipangle) '.osim']);
else
    osimmodel.print([cd '\ModelGenerator\OneDOF_Knee_Thelen' num2str(Hipangle) '.osim']);
end
end
