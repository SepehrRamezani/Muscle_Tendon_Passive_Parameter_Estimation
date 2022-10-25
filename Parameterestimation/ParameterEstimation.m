function [kneeTrackingParamSolution]=ParameterEstimation(StateTrackTable,ControlTrackTable,osimmodel,Coordlable,Data)
import org.opensim.modeling.*;
ComplianacMusclename=Data.ComplianacMusclename;
MinMTCLength=Data.(Coordlable).MuscleInfo.MinMTCLength;
Solverinterval=Data.ParamSolverinterval;
Etime=Data.Etime;
Stime=Data.Stime;
ControlWight=1;
% w=1/osimmodel.getForceSet().getSize();
StateWeight = 10.0/osimmodel.getNumCoordinates();
%% Define tracking problem
track = MocoTrack();
track.setName('kneestateTracking');
stateTrackingWeight = 1;
tableProcessor = TableProcessor(StateTrackTable);
% tableProcessor.append(TabOpLowPassFilter(2));
modelProcessor = ModelProcessor(osimmodel);
track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(Stime);
track.set_final_time(Etime);
track.set_minimize_control_effort(false);
stateWeights = MocoWeightSet();

if contains(Data.ActiveCoordinates,"knee_angle_r")
    stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',StateWeight));
    stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/speed',StateWeight*0.1));
    
else
    stateWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/value',StateWeight*4));
    stateWeights.cloneAndAppend(MocoWeight('/jointset/ankle_r/ankle_angle_r/speed',StateWeight*0.4));
end
track.set_states_weight_set(stateWeights);
study = track.initialize();
problem = study.updProblem();
ContTracking = MocoControlTrackingGoal('kneeControlTracking');
% ContTracking.setWeight(w);
% controlsRef = TableProcessor('Kneeflexion_solution.sto');
ConttableProcessor = TableProcessor(ControlTrackTable);
ContTracking.setReference(ConttableProcessor);
for i=0:1:osimmodel.getMuscles().getSize()-1
    Musname = osimmodel.updMuscles().get(i).getName();
    MusPath=append('/forceset/',char(Musname));
    MaxTendonSlack=MinMTCLength(i+1);
    param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.2*MaxTendonSlack,MaxTendonSlack));
    param1= MocoParameter(append('passive_fiber_',char(Musname)),MusPath,'passive_fiber_strain_at_one_norm_force', MocoBounds(0.2,0.8));
    if sum(strcmp(char(Musname), ComplianacMusclename))
        param2= MocoParameter(append('tendon_strain_',char(Musname)),MusPath,'tendon_strain_at_one_norm_force', MocoBounds(0.01,0.1));
        problem.addParameter(param2);
    else
        problem.addParameter(param1);
    end
        problem.addParameter(param);
end

for corindx = 1:length(Data.ActiveCoordinates)
CoordinateActuatorName=append('/forceset/',char(Data.ActiveCoordinates(corindx)),'_act');
ContTracking.setReferenceLabel(CoordinateActuatorName,CoordinateActuatorName);
ContTracking.setWeightForControl(CoordinateActuatorName,ControlWight);
end
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
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',[-.5, .5]);
%% optimal_fiber_length
solver = study.initCasADiSolver();
%% define solver
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-1);
solver.set_optim_max_iterations(4000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
if isfile(Data.(Coordlable).ParamSimulPath)
    solver.setGuessFile(Data.(Coordlable).ParamSimulPath);
else
%     solver.setGuessFile([cd '\Parameterestimation\Parameter_Initial_Guess_' Coordlable '.sto']);
end
kneeTrackingParamSolution = study.solve();
kneeTrackingParamSolution.write(Data.(Coordlable).ParamSimulPath);
end
