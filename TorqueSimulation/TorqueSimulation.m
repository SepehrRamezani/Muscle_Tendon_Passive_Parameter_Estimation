% clear all
import org.opensim.modeling.*;

% This initial block of code is identical to the code above.
inverse = MocoInverse();
Logger.addSink(JavaLogSink());
modelProcessor = ModelProcessor('subject_walk_armless.osim');
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpAddReserves(1.0));
inverse.setModel(modelProcessor);
inverse.setKinematics(TableProcessor('coordinates.sto'));
inverse.set_initial_time(3);
inverse.set_final_time(4);
inverse.set_mesh_interval(0.02);
inverse.set_kinematics_allow_extra_columns(true);

solution = inverse.solve();
solution.getMocoSolution().write('example3DWalking_MocoInverse_solution.sto');
% Generate a report with plots for the solution trajectory.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
        'example3DWalking_MocoInverse_solution.sto', 'bilateral', true);
% The report is saved to the working directory.
reportFilepath = report.generate();
open(reportFilepath);