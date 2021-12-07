clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)
% path=[cd '\TorqueSimulation']
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];
Data.SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
Data.ComplianacMusclename=["gaslat_r","gasmed_r"];
Data.DeGrooteflage=1;
% Hipangle=0;%deg
Data.RefModelpath=append(cd,'\ModelGenerator\subject_walk_armless_RLeg_justknee.osim');
Data.RefStatepath=append(cd,'\TorqueSimulation\referenceCoordinates.sto');
tableProcessor=TableProcessor(Data.RefStatepath);
Data.TorqueSolverinterval=20;
Data.ParamSolverinterval=40;
Data.Etime=20;
Data.Stime=0;
ce=0;
for Hipangle=0:45:90
ce=ce+1;
Data.Hipangle(ce)=Hipangle;
Hiplable=append('Hip',num2str(Hipangle));
Data.(Hiplable).SimulPath=[cd '\TorqueSimulation\Kneeflexion_solution_Degroot_Hip' num2str(Hipangle) '.sto'];
DataTable=TableProcessor(Data.(Hiplable).SimulPath);
kneeTrackingSolutionTable=DataTable.process;
Refmmodel = Model(Data.RefModelpath);
[osimmodel,Data.(Hiplable)]=Modelcreator(Hipangle,Data,Refmmodel);
% [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Hipangle,Data);
% [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Hipangle,Data);
% [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolutionTable,kneeTrackingSolutionTable,osimmodel,Hiplable,Data);

end
save([cd '\SimData.mat'],'Data');
% Save(Data)
% bar(Error)
Plotting()
