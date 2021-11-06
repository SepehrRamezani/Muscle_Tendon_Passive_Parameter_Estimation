clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)
% path=[cd '\TorqueSimulation']
% SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","sart_r","semimem_r","semiten_r","tfl_r","vasint_r","vaslat_r","vasmed_r"];
% SimMusclename=["knee_act"];
SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
DeGrooteflage=1;
% Hipangle=0;%deg
RefModelpath=append(cd,'\ModelGenerator\subject_walk_armless_RLeg_justknee.osim');
RefStatepath=append(cd,'\TorqueSimulation\referenceCoordinates.sto');
tableProcessor=TableProcessor(RefStatepath);
TorqueSolverinterval=20;
ParamSolverinterval=40;
Etime=20;
ce=0;
for Hipangle=0:45:90
% Hipangle=90-Hipangle;
SimulPath=[cd '\TorqueSimulation\Kneeflexion_solution_Degroot_Hip' num2str(Hipangle) '.sto'];
DataTable=TableProcessor(SimulPath);
kneeTrackingSolutionTable=DataTable.process;
Refmmodel = Model(RefModelpath);
ce=ce+1;
[osimmodel,TSlack(ce,:),Passive,MinMTCLength(ce,:)]=Modelcreator(Hipangle,SimMusclename,DeGrooteflage,Refmmodel);
% [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Hipangle,TorqueSolverinterval,Etime);
% [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Hipangle,MinMTCLength,ParamSolverinterval,Etime);
[kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolutionTable,kneeTrackingSolutionTable,osimmodel,Hipangle,MinMTCLength,ParamSolverinterval,Etime);

for t=1:kneeTrackingParamSolution.getNumParameters()
    OptimziedPar=kneeTrackingParamSolution.getParameters().get(t-1);  
    OptParam1(ce,t)=OptimziedPar(1);
    OptParam2(ce,t)=OptimziedPar(2);
end
Error1(ce,:)=(OptParam1(ce,:)-TSlack(ce,:))./TSlack(ce,:).*100;
Error2(ce,:)=(OptParam2(ce,:)-TSlack(ce,:))./TSlack(ce,:).*100;
ParName=kneeTrackingParamSolution.getParameterNames();
clear osimmodel kneeTrackingSolution kneeTrackingParamSolution
end
% bar(Error)
