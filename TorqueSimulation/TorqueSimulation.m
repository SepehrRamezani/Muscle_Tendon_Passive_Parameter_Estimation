clear all
import org.opensim.modeling.*;
ModelPath=[cd '\..\ModelGenerator\subject_walk_armless_DeGroote.osim'];
%% Which muscles we want to keep (need an actuator to estimate external torque)
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];
Logger.addSink(JavaLogSink());
%% initialze parameters
Logger.addSink(JavaLogSink());
osimmodel = Model(ModelPath);
ControlWight=1.0/osimmodel.getForceSet().getSize();
StateWeight = 1.0/osimmodel.getNumCoordinates();
GlobalstateTrackingWeight = 1;
Stime=0;
Etime=6;
Solverinterval=20;
%% Import reference state
tableProcessor = TableProcessor('referenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));
%% make a tracking problem 
track = MocoTrack();
track.setName('kneeTracking');
modelProcessor = ModelProcessor(osimmodel);
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(GlobalstateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(Stime);
track.set_final_time(Etime);
stateWeights = MocoWeightSet();
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',StateWeight));
track.set_states_weight_set(stateWeights);
study = track.initialize();
%% Updating problem
problem = study.updProblem();
model = modelProcessor.process();
model.initSystem();
%% add control costfunction
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(ControlWight);
effort.setExponent(2);
effort.setDivideByDisplacement(false);
%% Bounderies
problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value',[0, 1.6]);
%% Defining Solver 
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-1);
solver.set_optim_max_iterations(3000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.resetProblem(problem);
solver.setGuessFile('soln_track_N50_w50.sto');
solver.setGuess(gaitTrackingSolution);
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write('Kneeflexion_solution.sto');
% study.visualize(kneeTrackingSolution   );
%%

