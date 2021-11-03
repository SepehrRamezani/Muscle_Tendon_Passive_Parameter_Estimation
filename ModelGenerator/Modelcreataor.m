clear all
import org.opensim.modeling.*;
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];
myLog = JavaLogSink();
Logger.addSink(myLog)
osismmodel = Model('subject_walk_armless_RLeg_justknee.osim');

% osismmodel.finalizeConnections();

Qrange=90*pi()/180;
Hipangle=90;%deg
%% Setup angles
%%% Pelvis
modeljointSet=osismmodel.getJointSet();
Pelvisjoint=modeljointSet.get(0);
Pelvisweldjoint=WeldJoint.safeDownCast(Pelvisjoint);
hipfram=Pelvisweldjoint.get_frames(0);
hipfram.set_orientation(Vec3(0,0,(90-Hipangle)/180*pi()));
%%% Hip_flexion
modelCoordSet = osismmodel.getCoordinateSet();
Hipcoord = modelCoordSet.get(0);
Hipcoord.setDefaultValue(Hipangle/180*pi());
%%% Knee_corrdiante
%% setup muscle properties
DeGrooteflage=1;
if DeGrooteflage
    DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
end
c=0;

for m = 0:osismmodel.getForceSet().getSize()-1
    frcset = osismmodel.updForceSet().get(c);
    if ~sum(strcmp(char(frcset.getName()), SimMusclename))
        isremove=osismmodel.updForceSet().remove(c);
    else
        c=c+1;
        if ~strcmp(char(frcset.getName()), 'knee_act')
            musc=Muscle.safeDownCast(frcset);
            musc.set_ignore_activation_dynamics(true);
 
            if DeGrooteflage
                % musc.set_ignore_tendon_compliance(true);
                dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
                dgf.set_min_control(0.0);
                dgf.set_max_control(0.0);
                dgf.set_active_force_width_scale(1);
                dgf.set_tendon_compliance_dynamics_mode('implicit');
                %  dgf.set_ignore_passive_fiber_force(true);
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
state=osismmodel.initSystem();
KneeCoor=osismmodel.updCoordinateSet().get(1);
for i=0:1:osismmodel.getMuscles().getSize()-1
           k=0;
            for q=0:0.3:Qrange
                k=k+1;
                
                KneeCoor.setValue(state, q);
                osismmodel.realizePosition(state);
                CurrentMuscle=osismmodel.getMuscles().get(i);
                musclelength(k)=CurrentMuscle.getLength(state);
            end
            MinMTCLength=min(musclelength);
            
            if MinMTCLength < CurrentMuscle.get_tendon_slack_length()
                warning('buckeling will be happend in %s',CurrentMuscle.getName())
                CurrentMuscle.set_tendon_slack_length(0.95*MinMTCLength);
            end
end
if DeGrooteflage
    osismmodel.print('OneDOF_Knee_DeGroote.osim');
else
    osismmodel.print('OneDOF_Knee_Thelen.osim');
end

