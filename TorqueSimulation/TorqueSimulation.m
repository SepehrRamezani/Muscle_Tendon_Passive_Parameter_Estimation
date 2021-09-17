clear all
import org.opensim.modeling.*;
SimMusclename=["bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","soleus_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
% muscleDrivenModel = getMuscleDrivenModel();
% This initial block of code is identical to the code above.
inverse = MocoInverse();
Logger.addSink(JavaLogSink());


if ~exist('subject_walk_armless_DeGroote.osim', 'file')
    osismmodel = Model('subject_walk_armless.osim');
    osismmodel.finalizeConnections();
    DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
    for m = 0:osismmodel.getMuscles().getSize()-1
        musc = osismmodel.updMuscles().get(m);
     if sum(strcmp(char(musc.getName()), SimMusclename))
        musc.setMinControl(0);
        musc.set_ignore_activation_dynamics(false);
        musc.set_ignore_tendon_compliance(false);
        musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
       
        dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
        dgf.set_active_force_width_scale(1.5);
        dgf.set_tendon_compliance_dynamics_mode('implicit');
        if strcmp(char(musc.getName()), 'soleus_r')
            % Soleus has a very long tendon, so modeling its tendon as rigid
            % causes the fiber to be unrealistically long and generate
            % excessive passive fiber force.
            dgf.set_ignore_passive_fiber_force(true);
        end
     else
          musc.set_appliesForce(false)
     end
    end
    osismmodel.print('subject_walk_armless_DeGroote.osim');
else
    osismmodel = Model('subject_walk_armless_DeGroote.osim');
end

modelProcessor = ModelProcessor(osismmodel);

% modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
% modelProcessor.append(ModOpIgnoreTendonCompliance());
% modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpAddReserves(2.0));
inverse.setModel(modelProcessor);
inverse.setKinematics(TableProcessor('coordinates.sto'));
inverse.set_initial_time(7);
inverse.set_final_time(7.2);
inverse.set_mesh_interval(0.02);
inverse.set_kinematics_allow_extra_columns(true);
inverse.set_minimize_sum_squared_activations(true);
solution = inverse.solve();
solution.getMocoSolution().write('inverseSolution.sto');

% Generate a report with plots for the solution trajectory.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
    'example3DWalking_MocoInverse_solution.sto', 'bilateral', true);
% The report is saved to the working directory.
reportFilepath = report.generate();
open(reportFilepath);
