function [kneeTrackingParamSolution]=Roboticleg_ParamOpt(StatecontrolTable,osimmodel,Coordlable,Data)
% ModelPath=[cd '\..\ModelGenerator\OneDOF_Knee_DeGroote.osim'];

ControlTable=StatecontrolTable.clone();
StateTable=StatecontrolTable.clone();
k=0;
for i=0:StatecontrolTable.getNumColumns()-1
    clname=string(StatecontrolTable.getColumnLabel(i));
    if contains(clname,"jointset")
        ControlTable.removeColumn(clname)
 
    elseif contains(clname,"forceset")
        StateTable.removeColumn(clname)
    else
        StateTable.removeColumn(clname)
        ControlTable.removeColumn(clname)

    end
end

Solverinterval=Data.ParamSolverinterval;
Stime=Data.(Coordlable).Stime;
% Etime=Data.(Coordlable).Etime;
import org.opensim.modeling.*;
hiplable='/jointset/Hip_Joint/Hip/value';
kneelable='/jointset/Knee_Joint/Knee/value';
% myLog = JavaLogSink();
% Logger.addSink(myLog);
%% Initialze parameters
% osimmodel = Model(ModelPath);
ControlWight=1;
StateWeight = 2/osimmodel.getNumCoordinates();
GlobalstateTrackingWeight = 1;
TotalControlWight=20;
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
track.setStatesReference(TableProcessor(StateTable));
track.set_states_global_tracking_weight(GlobalstateTrackingWeight);
track.set_allow_unused_references(false);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
% track.set_initial_time(Stime);
% track.set_final_time(Etime);
stateWeights = MocoWeightSet();
statelables=StateTable.getColumnLabels;
for i=1:statelables.size()
    statelable=string(statelables.get(i-1));
    if contains(statelable,"jointset")&contains(statelable,"speed")
        stateWeights.cloneAndAppend(MocoWeight(statelable,StateWeight/4));
%  stateWeights.cloneAndAppend(MocoWeight(statelable,0/4));
    elseif contains(statelable,"jointset")&contains(statelable,"value")
        stateWeights.cloneAndAppend(MocoWeight(statelable,StateWeight));
    else
    end
end

track.set_states_weight_set(stateWeights);
study = track.initialize();
%% Updating problem
problem = study.updProblem();


ContTracking = MocoControlTrackingGoal('kneeControlTracking');
% ContTracking.setWeight(w);
% controlsRef = TableProcessor('Kneeflexion_solution.sto');
ConttableProcessor = TableProcessor(ControlTable);
ContTracking.setReference(ConttableProcessor);
ContTracking.setWeight(TotalControlWight);

Minrestpos=Data.restingpos-Data.Maxdeformation;
Minrestpos(find(Minrestpos<(Data.restingpos*.70)))=Data.restingpos(find(Minrestpos<(Data.restingpos*.70)))*.7;
for i=1:length(Data.ActiveAct)
    Springname=string(osimmodel.getForceSet().get(Data.ActiveAct(i)).getName());
    Maxrestpos=Data.(Coordlable).(Springname).Minlenght;

    %     Minrestpos

    MusPath=append('/forceset/',char(Springname));
    
    param = MocoParameter(append('stiffness_',char(Springname)),MusPath,'stiffness', MocoBounds(Data.Stiffnessboundary(i,1),Data.Stiffnessboundary(i,2)));
    param1 = MocoParameter(append('restpos_',char(Springname)),MusPath,'resting_length', MocoBounds(Minrestpos(i),Maxrestpos));
    problem.addParameter(param);
    problem.addParameter(param1);
end


% param = MocoParameter('Leg_mass','/bodyset/Shank','mass', MocoBounds(0.5,2));
% param1 = MocoParameter('Foot_mass','/bodyset/Foot','mass', MocoBounds(0.3,.9));
% 
% problem.addParameter(param);
% problem.addParameter(param1);

for corindx = 1:length(Data.ActiveCoordinates)
    %% force reference labaling
    CoordinateActuatorName=append('/forceset/',char(Data.ActiveCoordinates(corindx)),'_act');
    ContTracking.setReferenceLabel(CoordinateActuatorName,CoordinateActuatorName);
    ContTracking.setWeightForControl(CoordinateActuatorName,ControlWight);
    %% Bounderies
    Currcoord=osimmodel.getCoordinateSet().get(Data.ActiveCoordinates(corindx));
    problem.setStateInfo(append('/jointset/',Data.ActiveCoordinates(corindx),'_Joint/',Data.ActiveCoordinates(corindx),'/value'),[Currcoord.getRangeMin(), Currcoord.getRangeMax()]);
end
problem.addGoal(ContTracking)
model = modelProcessor.process();
model.initSystem();


effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(0);

%% Defining Solver
solver=study.initTropterSolver();
% solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-2);
solver.set_optim_max_iterations(1000);
% solver.set_implicit_auxiliary_derivatives_weight(0.00001);
% solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
if isfile(Data.(Coordlable).ParmSimulPath)
    solver.setGuessFile(Data.(Coordlable).ParmSimulPath);
else
    %     solver.setGuessFile([cd '\TorqueSimulation\Tracking_Initial_Guess.sto']);
end
study.print(strrep(Data.(Coordlable).ParmSimulPath,'.sto','.omoco'))
kneeTrackingParamSolution = study.solve();
if ~kneeTrackingParamSolution.success()
    kneeTrackingParamSolution.unseal()
    fprintf(2,'\n\n\n\n\n\n\n Optimization of %s is sealed \n\n\n\n\n\n\n',Coordlable);
else
    fprintf('\n\n\n\n\n\n Optimization of %s is done \n\n\n\n\n\n',Coordlable);
end 
kneeTrackingParamSolution.write(Data.(Coordlable).ParmSimulPath);
% study.visualize(kneeTrackingSolution);
end

