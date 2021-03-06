function [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Hipangle,Solverinterval,Etime)
% ModelPath=[cd '\..\ModelGenerator\OneDOF_Knee_DeGroote.osim'];
import org.opensim.modeling.*;
% myLog = JavaLogSink();
% Logger.addSink(myLog);
%% Initialze parameters
% osimmodel = Model(ModelPath);
ControlWight=1;
StateWeight = 10.0/osimmodel.getNumCoordinates();
GlobalstateTrackingWeight = 1;
Stime=0;
% Etime=20;
% Solverinterval=10;
%% Import reference state
% tableProcessor = TableProcessor('referenceCoordinates.sto');
% tableProcessor.append(TabOpLowPassFilter(6));
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
% effort.setWeight(1);
effort.setWeightForControl('/forceset/knee_act',ControlWight);
%%% disable muscles
for i=0:1:osimmodel.getMuscles().getSize()-1
    Musname = osimmodel.updMuscles().get(i).getName();
    MusPath=append('/forceset/',char(Musname));
    effort.setWeightForControl(MusPath,0);
end
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
solver.setGuessFile([cd '\TorqueSimulation\Tracking_Initial_Guess.sto']);
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write([cd '\TorqueSimulation\Kneeflexion_solution_Degroot_Hip' num2str(Hipangle) '.sto']);
% study.visualize(kneeTrackingSolution);
end

