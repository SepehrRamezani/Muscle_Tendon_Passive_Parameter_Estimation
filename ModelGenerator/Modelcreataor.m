clear all
import org.opensim.modeling.*;
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","sart_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];
Logger.addSink(JavaLogSink());
osismmodel = Model('subject_walk_armless_RLeg_justknee.osim');
osismmodel.finalizeConnections();
%% setup hip_felexion angle
modeljointSet=osismmodel.getJointSet();
Hipjoint=modeljointSet.get(0);
Hipweldjoint=WeldJoint.safeDownCast(Hipjoint);
hipfram=Hipweldjoint.get_frames (0);
hipfram.set_orientation(Vec3(0,0,90/180*pi()));
modelCoordSet = osismmodel.getCoordinateSet();
currentcoord = modelCoordSet.get(0);
currentcoord.setDefaultValue(0/180*pi());
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
%                  musc.set_ignore_tendon_compliance(true);
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
osismmodel.initSystem()
if DeGrooteflage
    osismmodel.print('OneDOF_Knee_DeGroote.osim');
else
    osismmodel.print('OneDOF_Knee_Thelen.osim');
end
