clear all
import org.opensim.modeling.*;
% SimMusclename=["bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act","recfem_r","semimem_r"];
SimMusclename=["knee_act"];

% muscleDrivenModel = getMuscleDrivenModel();
% This initial block of code is identical to the code above.
Logger.addSink(JavaLogSink());
if ~exist('subject_walk_armless_DeGroote.osim', 'file')
    osismmodel = Model('subject_walk_armless.osim');
    osismmodel.finalizeConnections();
    DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
    c=0;
    for m = 0:osismmodel.getForceSet().getSize()-1
        frcset = osismmodel.updForceSet().get(c);
        if ~sum(strcmp(char(frcset.getName()), SimMusclename))
            isremove=osismmodel.updForceSet().remove(c);
            
        else
             c=c+1;
            if ~strcmp(char(frcset.getName()), 'knee_act')
               
                musc=Muscle.safeDownCast(frcset);
                musc.set_ignore_activation_dynamics(false);
                musc.set_ignore_tendon_compliance(true);
                %             musc.set_max_isometric_force(.5 * musc.get_max_isometric_force());
                dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
                dgf.set_active_force_width_scale(1.5);
                dgf.set_tendon_compliance_dynamics_mode('implicit');
                if strcmp(char(musc.getName()), 'recfem_r')&& strcmp(char(musc.getName()), 'recfem_r')
                    %                 dgf.set_active_force_width_scale(10);
                    % Soleus has a very long tendon, so modeling its tendon as rigid
                    % causes the fiber to be unrealistically long and generate
                    % excessive passive fiber force.
                    %    dgf.set_ignore_passive_fiber_force(true);
                    musc.set_max_isometric_force(100 * musc.get_max_isometric_force());
                end
            end
            
        end
        
    end
    
    osismmodel.initSystem()
    osismmodel.print('subject_walk_armless_DeGroote.osim');
else
    osismmodel = Model('subject_walk_armless_DeGroote.osim');
end
study = MocoStudy();
problem = study.updProblem();
problem.setModel(osismmodel);
problem.setTimeBounds(0, .1);
problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value', ...
    [0, 1.7], 0, 1.5);

% problem.setStateInfoPattern('/jointset/.*/speed', [0 0], 0, 0);
problem.setStateInfoPattern('/jointset/walker_knee_r/knee_angle_r/speed', [0.11 0.11], 0.11, 0.11);
% problem.setStateInfoPattern('/jointset/patellofemoral_r/knee_angle_r_beta/speed', [0.11 0.11], 0.11, 0.11);
problem.addGoal(MocoControlGoal('myeffort'));
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(25);
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
predictSolution = study.solve();
predictSolution.write('predictSolution.sto');
%%
% inverse = MocoInverse();
% 
% % osismmodel = Model('subject_walk_armless.osim');
% modelProcessor = ModelProcessor(osismmodel);
% 
% % modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
% modelProcessor.append(ModOpIgnoreTendonCompliance());
% % modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% % modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1));
% % modelProcessor.append(ModOpAddReserves(100.0));
% inverse.setModel(modelProcessor);
% tableProcessor = TableProcessor('coordinates.sto');
% 
% 
% inverse.setKinematics(tableProcessor);
% inverse.set_initial_time(0.5);
% inverse.set_final_time(.7);
% inverse.set_mesh_interval(0.02);
% inverse.set_kinematics_allow_extra_columns(true);
% inverse.set_minimize_sum_squared_activations(true);
% 
% % study = inverse.initialize();
% % problem = study.updProblem();
% %
% % problem.addGoal
% % solution = study.solve();
% 
% solution = inverse.solve();
% ww=solution.unseal();
% solution.getMocoSolution().write('inverseSolution.sto');
% 
% % Generate a report with plots for the solution trajectory.
% model = modelProcessor.process();
% report = osimMocoTrajectoryReport(model, ...
%     'inverseSolution.sto', 'bilateral', true);
% % The report is saved to the working directory.
% reportFilepath = report.generate();
% open(reportFilepath);
