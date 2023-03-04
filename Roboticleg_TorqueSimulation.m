function [kneeTrackingSolution]=Roboticleg_TorqueSimulation(tableProcessor,osimmodel,Coordlable,Data)
% ModelPath=[cd '\..\ModelGenerator\OneDOF_Knee_DeGroote.osim'];
Solverinterval=Data.TorqueSolverinterval;
 Stime=Data.(Coordlable).Stime;
Etime=Data.(Coordlable).Etime;
import org.opensim.modeling.*;
hiplable='/jointset/Hip_Joint/Hip/value';
kneelable='/jointset/Knee_Joint/Knee/value';
% myLog = JavaLogSink();
% Logger.addSink(myLog);
%% Initialze parameters
% osimmodel = Model(ModelPath);
ControlWight=1;
StateWeight = 10.0/osimmodel.getNumCoordinates();
GlobalstateTrackingWeight = 1;
GlobalControlWeight=0.1;
% Etime=20;
% Solverinterval=10;
%% Import reference state
% tableProcessor = TableProcessor('referenceCoordinates.sto');
% tableProcessor.append(TabOpLowPassFilter(6));
%% make a tracking problem 
track = MocoTrack();
track.setName('Statetracking');
modelProcessor = ModelProcessor(osimmodel);
track.setModel(modelProcessor);
track.setStatesReference(TableProcessor(tableProcessor));
track.set_states_global_tracking_weight(GlobalstateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
% track.set_initial_time(Stime);
track.set_final_time(Etime);
stateWeights = MocoWeightSet();
statelables=tableProcessor.getColumnLabels;
for i=1:statelables.size()
    statelable=string(statelables.get(i-1));
    if contains(statelable,"speed")
    stateWeights.cloneAndAppend(MocoWeight(statelable,StateWeight/4));   
    else
    stateWeights.cloneAndAppend(MocoWeight(statelable,StateWeight));
    end
end

track.set_states_weight_set(stateWeights);
study = track.initialize();
%% Updating problem
problem = study.updProblem();
model = modelProcessor.process();
model.initSystem();
%% add control costfunction
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(GlobalControlWeight);
for corindx = 1:length(Data.ActiveCoordinates)
CoordinateActuatorName=append('/forceset/',Data.ActiveCoordinates(corindx),'_act');
effort.setWeightForControl(CoordinateActuatorName,ControlWight);
%% Bounderies
Currcoord=osimmodel.getCoordinateSet().get(Data.ActiveCoordinates(corindx));
problem.setStateInfo(append('/jointset/',Data.ActiveCoordinates(corindx),'_Joint/',Data.ActiveCoordinates(corindx),'/value'),[Currcoord.getRangeMin(), Currcoord.getRangeMax()]);
end

%%% disable muscles
effort.setExponent(2);
effort.setDivideByDisplacement(false);

%% Defining Solver 
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-6);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_max_iterations(3000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001);
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
if isfile(Data.(Coordlable).TorqeSimulPath)
     solver.setGuessFile(Data.(Coordlable).TorqeSimulPath);
else
%     solver.setGuessFile([cd '\TorqueSimulation\Tracking_Initial_Guess.sto']);
end
study.print(strrep(Data.(Coordlable).TorqeSimulPath,'.sto','.omoco'))
kneeTrackingSolution = study.solve();
kneeTrackingSolution.write(Data.(Coordlable).TorqeSimulPath);
% study.visualize(kneeTrackingSolution);
end

