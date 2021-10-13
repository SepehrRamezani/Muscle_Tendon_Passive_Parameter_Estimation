clear all
% pause(30)
import org.opensim.modeling.*;
Logger.addSink(JavaLogSink());
osismmodel = Model('subject_walk_armless_Thelen.osim');
w=1/osismmodel.getForceSet().getSize();
% pq = 1.0/osismmodel.getNumCoordinates();
pq = 1.0;
%% define problem
track = MocoTrack();
track.setName('kneestateTracking');
stateTrackingWeight = 0.25;

tableProcessor = TableProcessor('Kneeflexion_solution.sto');
tableProcessor.append(TabOpLowPassFilter(6));
modelProcessor = ModelProcessor(osismmodel);
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0);
track.set_final_time(6);
track.set_minimize_control_effort(false);
stateWeights = MocoWeightSet();
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',pq));
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/speed',pq.*0.0001));
track.set_states_weight_set(stateWeights);
study = track.initialize();
problem = study.updProblem();

ContTracking = MocoControlTrackingGoal('kneeControlTracking');
ContTracking.setWeight(w);
controlsRef = TableProcessor('Kneeflexion_solution.sto');
ContTracking.setReference(controlsRef);

ContTracking.setReferenceLabel('/forceset/bflh_r','/forceset/bflh_r');
ContTracking.setReferenceLabel('/forceset/bfsh_r','/forceset/bfsh_r');
ContTracking.setReferenceLabel('/forceset/gaslat_r','/forceset/gaslat_r');
ContTracking.setReferenceLabel('/forceset/gasmed_r','/forceset/gasmed_r');
ContTracking.setReferenceLabel('/forceset/recfem_r','/forceset/recfem_r');
ContTracking.setReferenceLabel('/forceset/semimem_r','/forceset/semimem_r');
ContTracking.setReferenceLabel('/forceset/semiten_r','/forceset/semiten_r');
ContTracking.setReferenceLabel('/forceset/vasint_r','/forceset/vasint_r');
ContTracking.setReferenceLabel('/forceset/vaslat_r','/forceset/vaslat_r');
ContTracking.setReferenceLabel('/forceset/vasmed_r','/forceset/vasmed_r');
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
% param = MocoParameter('Test', '/forceset/bflh_r/geometrypath/bflh_r-P1/','location', MocoBounds(0,1),1);
param = MocoParameter('max_isometric_force_bflh', '/forceset/bflh_r','max_isometric_force', MocoBounds(1000,7000));
param2 = MocoParameter('max_isometric_force_vaslat', '/forceset/vaslat_r', 'max_isometric_force', MocoBounds(1000,7000));
param3 = MocoParameter('max_isometric_force_Semimem', '/forceset/semimem_r', 'max_isometric_force', MocoBounds(1000,7000));
param4 = MocoParameter('max_isometric_force_vasmed', '/forceset/vasmed_r', 'max_isometric_force', MocoBounds(1000,7000));
problem.addParameter(param);
problem.addParameter(param2);
problem.addParameter(param3);
problem.addParameter(param4);
solver = study.initCasADiSolver();
%% define solver
solver.set_num_mesh_intervals(20);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-5);
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

