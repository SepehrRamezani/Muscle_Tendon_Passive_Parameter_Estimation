clear all
import org.opensim.modeling.*;
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];
Logger.addSink(JavaLogSink());
osismmodel = Model('subject_walk_armless_RLeg_justknee.osim');
osismmodel.finalizeConnections();
% DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
c=0;
for m = 0:osismmodel.getForceSet().getSize()-1
    frcset = osismmodel.updForceSet().get(c);
    if ~sum(strcmp(char(frcset.getName()), SimMusclename))
        isremove=osismmodel.updForceSet().remove(c);
        
    else
        c=c+1;
        if ~strcmp(char(frcset.getName()), 'knee_act')
            
            musc=Muscle.safeDownCast(frcset);
            musc.set_min_control(0.01);
            musc.set_max_control(0.01);
            musc.set_ignore_activation_dynamics(true);
%             musc.set_ignore_tendon_compliance(true);
%             dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
%             dgf.set_active_force_width_scale(1.5);
%             dgf.set_tendon_compliance_dynamics_mode('implicit');
            
            if strcmp(char(musc.getName()), 'soleus_r')
            %  dgf.set_active_force_width_scale(10);
%             dgf.set_ignore_passive_fiber_force(true);
            % musc.set_max_isometric_force(1.5 * musc.get_max_isometric_force());
            % dgf.set_ignore_passive_fiber_force(true);
            end
        end
        
    end
    
end

osismmodel.initSystem()
osismmodel.print('subject_walk_armless_Thelen.osim');