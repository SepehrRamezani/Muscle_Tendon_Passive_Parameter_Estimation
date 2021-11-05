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
% SimulPath=[cd '\..\TorqueSimulation\Kneeflexion_solution_Degroot.sto'];
for Hipangle=90:45:90
Refmmodel = Model(RefModelpath);
ce=ce+1;
[osimmodel,TSlack(ce,:),MinMTCLength(ce,:)]=Modelcreator(Hipangle,SimMusclename,DeGrooteflage,Refmmodel);
[kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Hipangle,TorqueSolverinterval,Etime);
[kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Hipangle,MinMTCLength,ParamSolverinterval,Etime);
for t=1:kneeTrackingParamSolution.getNumParameters()
    OptimziedPar(ce,t)=kneeTrackingParamSolution.getParameters().get(t-1);  
end
Error(ce,:)=(OptimziedPar(ce,:)-TSlack(ce,:))./TSlack(ce,:).*100;
ParName=kneeTrackingParamSolution.getParameterNames();
% clear osimmodel kneeTrackingSolution kneeTrackingParamSolution
end
bar(Error)
