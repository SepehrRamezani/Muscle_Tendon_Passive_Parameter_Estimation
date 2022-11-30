function [kneeTrackingParamSolution]=ParameterEstimation(StateTrackTable,ControlTrackTable,osimmodel,Coordlable,Data)
import org.opensim.modeling.*;
ComplianacMusclename=Data.ComplianacMusclename;
MinMTLength=Data.(Coordlable).MuscleInfo.MinMTLength;
Solverinterval=Data.ParamSolverinterval;
Etime=Data.Etime;
Stime=Data.Stime;

% w=1/osimmodel.getForceSet().getSize();
StateWeight = 10.0/osimmodel.getNumCoordinates();
ControlWight=StateWeight;
osimmodel=changemodelproperty(osimmodel,Coordlable,Data,0);
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

if contains(Data.ActiveCoordinates,"knee_angle")
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/walker_knee_',Data.whichleg,'/knee_angle_',Data.whichleg,'/value'),StateWeight));
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/walker_knee_',Data.whichleg,'/knee_angle_',Data.whichleg,'/speed'),StateWeight*0.01));
    
else
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/ankle_',Data.whichleg,'/ankle_angle_',Data.whichleg,'/value'),StateWeight*4));
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/ankle_',Data.whichleg,'/ankle_angle_',Data.whichleg,'/speed'),StateWeight*0.04));
end
track.set_states_weight_set(stateWeights);
study = track.initialize();
problem = study.updProblem();
ContTracking = MocoControlTrackingGoal('kneeControlTracking');
% ContTracking.setWeight(w);
% controlsRef = TableProcessor('Kneeflexion_solution.sto');
ConttableProcessor = TableProcessor(ControlTrackTable);
ContTracking.setReference(ConttableProcessor);
ContTracking.setWeight(5);
% param3 = MocoParameter('WrappingR','/bodyset/tibia_r/wrapobjectset/GasMed_at_shank_r','radius', MocoBounds(0.2,0.8));
for i=0:1:osimmodel.getMuscles().getSize()-1
    Musname = osimmodel.updMuscles().get(i).getName();
    MusPath=append('/forceset/',char(Musname));
    MaxTendonSlack=MinMTLength(i+1);
    param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.05*MaxTendonSlack,MaxTendonSlack));
    param1= MocoParameter(append('passive_fiber_',char(Musname)),MusPath,'passive_fiber_strain_at_one_norm_force', MocoBounds(Data.PassiveFiberBound(1),Data.PassiveFiberBound(2)));
    if sum(strcmp(char(Musname), ComplianacMusclename))
        param2= MocoParameter(append('tendon_strain_',char(Musname)),MusPath,'tendon_strain_at_one_norm_force', MocoBounds(Data.TendonStrainBound(1),Data.TendonStrainBound(2)));
        problem.addParameter(param2);
        problem.addParameter(param1);
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
problem.setStateInfo(append('/jointset/walker_knee_',Data.whichleg,'/knee_angle_',Data.whichleg,'/value'),[0, 1.6]);
problem.setStateInfo(append('/jointset/ankle_',Data.whichleg,'/ankle_angle_',Data.whichleg,'/value'),[-.5, .5]);
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
