function [kneeTrackingParamSolution]=ParameterEstimation(StateTrackTable,ControlTrackTable,osimmodel,ComplianacMusclename,Hipangle,MinMTCLength,Solverinterval,Etime)
% pause(30)
import org.opensim.modeling.*;
% myLog = JavaLogSink();
% Logger.addSink(myLog)
% ModelPath=[cd '\..\ModelGenerator\OneDOF_Knee_DeGroote.osim'];
% SimulPath=[cd '\..\TorqueSimulation\Kneeflexion_solution_Degroot.sto'];
% osimmodel = Model(ModelPath);
state = osimmodel.initSystem();
ControlWight=1;
% w=1/osimmodel.getForceSet().getSize();
StateWeight = 10.0/osimmodel.getNumCoordinates();
Qrange=pi()/2;
Stime=0;
% Etime=20;
% Solverinterval=10;
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
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/value',StateWeight));
stateWeights.cloneAndAppend(MocoWeight('/jointset/walker_knee_r/knee_angle_r/speed',StateWeight*0.1));
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
%     ContTracking.setReferenceLabel(MusPath,MusPath);
%     c=0;
    % finding maximum bound of tendon slack length
%     for q=0:0.1:Qrange
%         c=c+1;
%         osimmodel.updCoordinateSet().get(1).setValue(state, q);
%         osimmodel.realizePosition(state);
%         musclelength(c)=osimmodel.getMuscles().get(i).getLength(state);
%     end
    MaxTendonSlack=MinMTCLength(i+1);
    param = MocoParameter(append('tendon_slack_',char(Musname)),MusPath,'tendon_slack_length', MocoBounds(0.2*MaxTendonSlack,MaxTendonSlack));
    param1= MocoParameter(append('passive_fiber_',char(Musname)),MusPath,'passive_fiber_strain_at_one_norm_force', MocoBounds(0.2,0.8));
    param2= MocoParameter(append('tendon_strain',char(Musname)),MusPath,'tendon_strain_at_one_norm_force', MocoBounds(0.01,0.06));
    if sum(strcmp(char(Musname), ComplianacMusclename))
        problem.addParameter(param2);
    end
    problem.addParameter(param);
    problem.addParameter(param1);
end

ContTracking.setReferenceLabel('/forceset/knee_act','/forceset/knee_act');
ContTracking.setWeightForControl('/forceset/knee_act',ControlWight);
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
solver.set_num_mesh_intervals(Solverinterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-1);
solver.set_optim_max_iterations(4000);
solver.set_implicit_auxiliary_derivatives_weight(0.00001)
solver.set_parameters_require_initsystem(false);
solver.resetProblem(problem);
% solver.setGuessFile([cd '\Parameterestimation\Parameter_Initial_Guess.sto']);
kneeTrackingParamSolution = study.solve();
kneeTrackingParamSolution.write([cd '\Parameterestimation\Parameter_Opt_Hip' num2str(Hipangle) '.sto']);
end
