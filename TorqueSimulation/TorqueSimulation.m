clear all
import org.opensim.modeling.*;
SimMusclename=["bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act","recfem_r","semimem_r"];
% SimMusclename=["knee_act"];

% muscleDrivenModel = getMuscleDrivenModel();
% This initial block of code is identical to the code above.
Logger.addSink(JavaLogSink());
if ~exist('subject_walk_armless_DeGroote.osim', 'file')
    osismmodel = Model('subject_walk_armless.osim');
    osismmodel.finalizeConnections();
    DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
    c=0;
    for m = 0:osismmodel.getForceSet().getSize()-1
        frcset = osismmodel.updForceSet().get(c);
        if ~sum(strcmp(char(frcset.getName()), SimMusclename))
            isremove=osismmodel.updForceSet().remove(c);
            
        else
             c=c+1;
            if ~strcmp(char(frcset.getName()), 'knee_act')
               
                musc=Muscle.safeDownCast(frcset);
                musc.set_ignore_activation_dynamics(false);
                musc.set_ignore_tendon_compliance(true);
                %             musc.set_max_isometric_force(.5 * musc.get_max_isometric_force());
                dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
                dgf.set_active_force_width_scale(1.5);
                dgf.set_tendon_compliance_dynamics_mode('implicit');
                if strcmp(char(musc.getName()), 'recfem_r')&& strcmp(char(musc.getName()), 'recfem_r')
                    %                 dgf.set_active_force_width_scale(10);
                    % Soleus has a very long tendon, so modeling its tendon as rigid
                    % causes the fiber to be unrealistically long and generate
                    % excessive passive fiber force.
                    %    dgf.set_ignore_passive_fiber_force(true);
                    musc.set_max_isometric_force(100 * musc.get_max_isometric_force());
                end
            end
            
        end
        
    end
    
    osismmodel.initSystem()
    osismmodel.print('subject_walk_armless_DeGroote.osim');
else
    osismmodel = Model('subject_walk_armless_DeGroote.osim');
end

track = MocoTrack();
track.setName('kneeTracking');
stateTrackingWeight = 1;
tableProcessor = TableProcessor('referenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));
modelProcessor = ModelProcessor("subject_walk_armless_DeGroote.osim");
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0.0);
track.set_final_time(6);
track.set_mesh_interval(0.05);
study = track.initialize();
problem = study.updProblem();
model = modelProcessor.process();
model.initSystem();
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
controlEffortWeight=10;
effort.setWeight(controlEffortWeight);
problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value', ...
    [0, 1.6]);
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write('Kneeflexion_solution.sto');
% study.visualize(kneeTrackingSolution   );

%%

