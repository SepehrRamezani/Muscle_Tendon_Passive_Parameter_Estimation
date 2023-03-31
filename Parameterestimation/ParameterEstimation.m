function [kneeTrackingParamSolution]=ParameterEstimation(StateTrackTable,ControlTrackTable,osimmodel,combinedname,Coordlable,Data)
import org.opensim.modeling.*;
ComplianceTendon=Data.ComplianceTendon;
SimMusclename=Data.(combinedname).SimMusclename;
MinMTLength=Data.(combinedname).MuscleInfo.MinMTLength;
TSL=Data.(combinedname).MuscleInfo.TSlack;
MaxIsoF=Data.(combinedname).MuscleInfo.MaxIso;
OptFiberL=Data.(combinedname).MuscleInfo.OptFiberL;
Solverinterval=Data.ParamSolverinterval;
Etime=Data.(Coordlable).Etime;
Stime=Data.(Coordlable).Stime;
% w=1/osimmodel.getForceSet().getSize();
% StateWeight = 1/(osimmodel.getNumCoordinates()+length(SimMusclename));%0.0909
StateWeight=0.0909;
% ControlWight=1/(osimmodel.getNumCoordinates()+length(SimMusclename));
ControlWight=StateWeight;
TotalControlWieght=Data.TotalControlWeight;
osimmodel=changemodelproperty(osimmodel,combinedname,Data,0);
%% Define tracking problem
track = MocoTrack();
track.setName('kneestateTracking');
stateTrackingWeight = 10;
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
study = track.initialize();
problem = study.updProblem();
if contains(Data.ActiveCoordinates,"knee_angle")
    kneelable=append('knee_angle_',Data.whichleg);
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/walker_knee_',Data.whichleg,'/knee_angle_',Data.whichleg,'/value'),StateWeight));
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/walker_knee_',Data.whichleg,'/knee_angle_',Data.whichleg,'/speed'),StateWeight*0.1));   
    Kneecoord =  osimmodel.getCoordinateSet().get(kneelable);
    problem.setStateInfo(append('/jointset/walker_knee_',Data.whichleg,'/knee_angle_',Data.whichleg,'/value'),[Kneecoord.getRangeMin(), Kneecoord.getRangeMax()]);

else
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/ankle_',Data.whichleg,'/ankle_angle_',Data.whichleg,'/value'),StateWeight*4));
    stateWeights.cloneAndAppend(MocoWeight(append('/jointset/ankle_',Data.whichleg,'/ankle_angle_',Data.whichleg,'/speed'),StateWeight*0.4));
    anklelable=append('ankle_angle_',Data.whichleg);
    Anklecoord =  osimmodel.getCoordinateSet().get(anklelable);
    problem.setStateInfo(append('/jointset/ankle_',Data.whichleg,'/',anklelable,'/value'),[Anklecoord.getRangeMin(), Anklecoord.getRangeMax()]);
end
track.set_states_weight_set(stateWeights);
study = track.initialize();
problem = study.updProblem();
ContTracking = MocoControlTrackingGoal('kneeControlTracking');
% ContTracking.setWeight(w);
% controlsRef = TableProcessor('Kneeflexion_solution.sto');
ConttableProcessor = TableProcessor(ControlTrackTable);
ContTracking.setReference(ConttableProcessor);
ContTracking.setWeight(TotalControlWieght);
% param3 = MocoParameter('WrappingR','/bodyset/tibia_r/wrapobjectset/GasMed_at_shank_r','radius', MocoBounds(0.2,0.8));
for i=1:1:length(Data.muscle4opt)
    Musname=Data.muscle4opt(i);
%     Musname = osimmodel.updMuscles().get(Data.muscle4opt(i));
    MusPath=append('/forceset/',char(Musname));
    MaxTendonSlack=MinMTLength(contains(SimMusclename,Musname));
    TendonSlack=TSL(contains(SimMusclename,Musname));
    MaxIso=MaxIsoF((contains(SimMusclename,Musname)));
    OptFL=OptFiberL((contains(SimMusclename,Musname)));

    param1= MocoParameter(append('passive_fiber_',char(Musname)),MusPath,'passive_fiber_strain_at_one_norm_force', MocoBounds(Data.PassiveFiberBound(1),Data.PassiveFiberBound(2)));
    param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.5*TendonSlack,MaxTendonSlack));
    param2= MocoParameter(append('tendon_strain_',char(Musname)),MusPath,'tendon_strain_at_one_norm_force', MocoBounds(Data.TendonStrainBound(1),Data.TendonStrainBound(2)));

    if sum(strcmp(char(Musname), ComplianceTendon))
        K=Data.TendonStiffness((contains(Data.ComplianceTendon,Musname)),:);
        epsilonbound=(1.97.*MaxIso)./(K.*TendonSlack);
        
        problem.addParameter(param2);
%         problem.addParameter(param1);
        problem.addParameter(param);
    else
        paramz= MocoParameter(append('OptL_',char(Musname)),MusPath,'optimal_fiber_length', MocoBounds(0.6*OptFL,OptFL*1.4));
maxtsl=0.9*MaxTendonSlack;
        param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.5*TendonSlack,maxtsl));
        problem.addParameter(param);
        problem.addParameter(param1);
        problem.addParameter(param2);
    end
        
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
% effort.setWeight(0);
% effort.setExponent(2);
% effort.setDivideByDisplacement(false);

%% optimal_fiber_length
solver = study.initCasADiSolver();
%% define solver
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(Data.convergencetolerance);
solver.set_optim_constraint_tolerance(Data.constrainttolerance);
solver.set_optim_max_iterations(6000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
study.print(strrep(Data.(Coordlable).ParamSimulPath,'.sto','.omoco'))
if isfile(Data.(Coordlable).ParamSimulPath)
    solver.setGuessFile(Data.(Coordlable).ParamSimulPath);
else
%   solver.setGuessFile([cd '\Parameterestimation\Parameter_Initial_Guess_' Coordlable '.sto']);
end
kneeTrackingParamSolution = study.solve();

if ~kneeTrackingParamSolution.success()
    kneeTrackingParamSolution.unseal()
    fprintf(2,'\n\n\n\n\n\n\n Optimization of %s is sealed \n\n\n\n\n\n\n',Coordlable);
else
    fprintf('\n\n\n\n\n\n Optimization of %s is done \n\n\n\n\n\n',Coordlable);
end 

kneeTrackingParamSolution.write(Data.(Coordlable).ParamSimulPath);
end
