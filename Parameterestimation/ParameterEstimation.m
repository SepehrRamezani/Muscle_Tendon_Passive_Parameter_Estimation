clear all
% pause(30)
import org.opensim.modeling.*;
Logger.addSink(JavaLogSink());
ModelPath=[cd '\..\ModelGenerator\OneDOF_Knee_DeGroote.osim'];
SimulPath=[cd '\..\TorqueSimulation\Kneeflexion_solution_Degroot_Hip90.sto'];
osismmodel = Model(ModelPath);
state = osismmodel.initSystem();
w=1/osismmodel.getForceSet().getSize();
Qrange=pi()/2;
%% find the bondray for tendon slack lenght
% [Musclename,maxlentgh]=FindmaxMTL(osismmodel,1.57)
% pq = 1.0/osismmodel.getNumCoordinates();
pq = 1.0;
%% Define tracking problem
track = MocoTrack();
track.setName('kneestateTracking');
stateTrackingWeight = 0.25;
tableProcessor = TableProcessor(SimulPath);
% tableProcessor.append(TabOpLowPassFilter(2));
modelProcessor = ModelProcessor(osismmodel);
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0);
track.set_final_time(20);
track.set_minimize_control_effort(false);
stateWeights = MocoWeightSet();
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',pq));
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/speed',pq.*0.0001));
track.set_states_weight_set(stateWeights);
study = track.initialize();
problem = study.updProblem();
ContTracking = MocoControlTrackingGoal('kneeControlTracking');
ContTracking.setWeight(w);
% controlsRef = TableProcessor('Kneeflexion_solution.sto');
ContTracking.setReference(tableProcessor);
for i=0:1:osismmodel.getMuscles().getSize()-1
    Musname = osismmodel.updMuscles().get(i).getName();
    MusPath=append('/forceset/',char(Musname));
    ContTracking.setReferenceLabel(MusPath,MusPath);
    c=0;
    % finding maximum bound of tendon slack length
    for q=0:0.1:Qrange
        c=c+1;
        osismmodel.updCoordinateSet().get(1).setValue(state, q);
        osismmodel.realizePosition(state);
        musclelength(c)=osismmodel.getMuscles().get(i).getLength(state);
    end
    maxlentgh=max(musclelength);
    %param = MocoParameter(append('passive_fiber_strain_at_one_norm_force',char(Musname)),MusPath,'passive_fiber_strain_at_one_norm_force', MocoBounds(0.2,0.8));
    param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.2*maxlentgh,maxlentgh));
    problem.addParameter(param);
end

ContTracking.setReferenceLabel('/forceset/knee_act','/forceset/knee_act');
ContTracking.setWeightForControl('/forceset/knee_act',10);
problem.addGoal(ContTracking)
model = modelProcessor.process();
model.initSystem();
%% add cost function
% effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
% effort.setWeight(0.001);
% effort.setExponent(2);
% effort.setDivideByDisplacement(false);
%% define parameter 
problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value',[0, 1.6]);
%% optimal_fiber_length
solver = study.initCasADiSolver();
%% define solver
solver.set_num_mesh_intervals(20);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-1);
solver.set_optim_max_iterations(3000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
% solver.setGuessFile('soln_track_N50_w50.sto');
% solver.setGuess(gaitTrackingSolution);
% problem.setControlInfo('/forceset/actuator',[0.01, 0.01]);
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write('Parameter_Opt.sto');
% study.visualize(kneeTrackingSolution   );
%%

