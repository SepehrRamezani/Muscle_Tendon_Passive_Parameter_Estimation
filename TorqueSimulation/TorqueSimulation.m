clear all
import org.opensim.modeling.*;
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];

% muscleDrivenModel = getMuscleDrivenModel();
% This initial block of code is identical to the code above.
Logger.addSink(JavaLogSink());
if ~exist('subject_walk_armless_DeGroote.osim', 'file')
    osismmodel = Model('subject_walk_armless_RLeg_justknee.osim');
    osismmodel.finalizeConnections();
%     DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
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
                musc.set_ignore_tendon_compliance(true);
                %             musc.set_max_isometric_force(.5 * musc.get_max_isometric_force());
%                 dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
%                 dgf.set_active_force_width_scale(1.5);
%                 dgf.set_tendon_compliance_dynamics_mode('implicit');
%                 if strcmp(char(musc.getName()), 'recfem_r')&& strcmp(char(musc.getName()), 'recfem_r')
%                     %    dgf.set_active_force_width_scale(10);
%                     %    dgf.set_ignore_passive_fiber_force(true);
%                     musc.set_max_isometric_force(1.5 * musc.get_max_isometric_force());
%                 end
            end
            
        end
        
    end
    
    osismmodel.initSystem()
    osismmodel.print('subject_walk_armless_DeGroote.osim');
else
    osismmodel = Model('subject_walk_armless_DeGroote.osim');
end
w=1/osismmodel.getForceSet().getSize();
pq = 1.0/osismmodel.getNumCoordinates();
track = MocoTrack();
track.setName('kneeTracking');
stateTrackingWeight = 1;
tableProcessor = TableProcessor('referenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));
modelProcessor = ModelProcessor(osismmodel);
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0.0);
track.set_final_time(6);
stateWeights = MocoWeightSet();
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',pq));

track.set_states_weight_set(stateWeights);
% track.set_mesh_interval(0.01);
study = track.initialize();
problem = study.updProblem();
model = modelProcessor.process();
model.initSystem();
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(w);
effort.setExponent(2);
effort.setDivideByDisplacement(false);

problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value',[0, 1.6]);
problem.addParameter(MocoParameter('PenAngle', 'body', 'mass', MocoBounds(0, 10)));


solver = study.initCasADiSolver();

solver.set_num_mesh_intervals(50);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-1);
solver.set_optim_max_iterations(3000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.resetProblem(problem);
% solver.setGuessFile('soln_track_N50_w50.sto');
% solver.setGuess(gaitTrackingSolution);
% problem.setControlInfo('/forceset/actuator',[0.01, 0.01]);
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write('Kneeflexion_solution.sto');
% study.visualize(kneeTrackingSolution   );
%%

