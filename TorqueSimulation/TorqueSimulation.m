function [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Coordlable,Data)
% ModelPath=[cd '\..\ModelGenerator\OneDOF_Knee_DeGroote.osim'];
Solverinterval=Data.TorqueSolverinterval;
Etime=Data.Etime;
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
track.set_allow_unused_references(false);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(Stime);
track.set_final_time(Etime);
stateWeights = MocoWeightSet();
if contains(Data.ActiveCoordinates,"knee_angle_r")
    stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',StateWeight));
    stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/speed',StateWeight*0.5));
    
else
    stateWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/value',StateWeight*2));
    stateWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/speed',StateWeight*1));
end
track.set_states_weight_set(stateWeights);
study = track.initialize();
%% Updating problem
problem = study.updProblem();
model = modelProcessor.process();
model.initSystem();
%% add control costfunction
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
% effort.setWeight(1);
for corindx = 1:length(Data.ActiveCoordinates)
CoordinateActuatorName=append('/forceset/',char(Data.ActiveCoordinates(corindx)),'_act');
effort.setWeightForControl(CoordinateActuatorName,ControlWight);
end
% effort.setWeightForControl('/forceset/knee_act',ControlWight);
% effort.setWeightForControl('/forceset/ankle_act',ControlWight);
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
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',[-.5, .5]);
%% Defining Solver 
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-1);
solver.set_optim_max_iterations(3000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
if isfile(Data.(Coordlable).TorqeSimulPath)
     solver.setGuessFile(Data.(Coordlable).TorqeSimulPath);
else
%     solver.setGuessFile([cd '\TorqueSimulation\Tracking_Initial_Guess.sto']);
end
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write(Data.(Coordlable).TorqeSimulPath);
% study.visualize(kneeTrackingSolution);
end

